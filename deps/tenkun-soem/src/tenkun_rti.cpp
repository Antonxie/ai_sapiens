#include "tenkun_rti.hpp"

#include "tenkun_gateway_pdo.h"

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <set>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

extern "C" {
#include "ethercat.h"
#include "motor_control.h"
#include "tenkun_cih408.h"
}

namespace XJDLRobot {

std::atomic<TenKunRti*> TenKunRti::s_active_{nullptr};

namespace {

/** 与旧版「motor_id」语义对齐：用于健康检查里与配置的 can_tx_id 比对。 */
uint16_t TenkunRxBindCompareId(const TenkunCanFdRxInfo& r) {
  switch (r.kind) {
    case TENKUN_RX_PARSE_JOINT_ID_RSP:
      return static_cast<uint16_t>(r.joint_id);
    case TENKUN_RX_PARSE_SET_JOINT_ID_RSP:
      return static_cast<uint16_t>(r.set_joint_new_id);
    default:
      return r.can_id_11;
  }
}

bool TenkunRxSlotHasFeedback(const TenkunCanFdRxInfo& r) { return TenkunRxBindCompareId(r) != 0u; }

int EcProcessSlaveLimit() {
  int n = ec_slavecount;
  if (n > SLAVE_NUMBER) {
    n = SLAVE_NUMBER;
  }
  return n > 0 ? n : 0;
}

bool FileStartsWithJsonObject(const std::string& path) {
  std::ifstream in(path.c_str());
  if (!in) {
    return false;
  }
  int c = 0;
  while ((c = in.get()) != EOF) {
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
      continue;
    }
    return c == '{';
  }
  return false;
}

/** 与 ParseRtiJson 相同 doc 根（含可选 rti 包裹），读取 EtherCAT 网卡名。 */
std::string ReadEtherCatInterfaceFromRtiJson(const std::string& path, std::string* err_out) {
  try {
    boost::property_tree::ptree root;
    boost::property_tree::read_json(path, root);
    boost::property_tree::ptree doc = root;
    const auto wrap = root.get_child_optional("rti");
    if (wrap) {
      doc = *wrap;
    }
    boost::optional<std::string> ifc = doc.get_optional<std::string>("interface");
    if (!ifc || ifc->empty()) {
      *err_out = "RTI JSON 缺少非空字段 \"interface\"（EtherCAT 网卡名，如 enp3s0）";
      return {};
    }
    return *ifc;
  } catch (const std::exception& ex) {
    *err_out = ex.what();
    return {};
  }
}

std::string ResolveEtherCatInterface(const std::string& rti_config_path, std::string* err_out) {
  if (!rti_config_path.empty() && FileStartsWithJsonObject(rti_config_path)) {
    return ReadEtherCatInterfaceFromRtiJson(rti_config_path, err_out);
  }
  const char* env = std::getenv("TENKUN_EC_IF");
  if (env != nullptr && env[0] != '\0') {
    return std::string(env);
  }
  if (rti_config_path.empty()) {
    *err_out = "未指定配置文件且未设置环境变量 TENKUN_EC_IF";
  } else {
    *err_out = "非 JSON 拓扑/配置未包含网卡名：请设置环境变量 TENKUN_EC_IF";
  }
  return {};
}

void SynthesizeCatalogFromLegacyConfigs(const std::vector<TenkunSlaveStationConfig>& cfgs,
                                        std::vector<TenkunRtiJointInfo>* joints_out) {
  joints_out->clear();
  int jidx = 0;
  for (const auto& cfg : cfgs) {
    for (const auto& b : cfg.motors) {
      TenkunRtiJointInfo j;
      j.joint_index = jidx++;
      j.name = "s" + std::to_string(cfg.slave_index) + "_ch" + std::to_string(b.channel_1_to_12) + "_can" +
               std::to_string(static_cast<unsigned>(b.can_tx_id));
      j.slave_index = cfg.slave_index;
      j.channel_1_to_12 = b.channel_1_to_12;
      j.can_tx_id = b.can_tx_id;
      joints_out->push_back(std::move(j));
    }
  }
}

bool ParseRtiJson(const std::string& path, std::vector<TenkunSlaveStationConfig>* stations_out,
                  std::vector<TenkunRtiJointInfo>* joints_out, std::vector<std::string>* joints_cmd_order_out,
                  std::vector<float>* default_pos_out, std::string* err_out) {
  stations_out->clear();
  joints_out->clear();
  if (joints_cmd_order_out != nullptr) {
    joints_cmd_order_out->clear();
  }

  try {
    boost::property_tree::ptree root;
    boost::property_tree::read_json(path, root);

    boost::property_tree::ptree doc = root;
    const auto wrap = root.get_child_optional("rti");
    if (wrap) {
      doc = *wrap;
    }

    const boost::property_tree::ptree& slaves_pt = doc.get_child("slaves");

    std::unordered_set<std::string> joint_names;
    std::unordered_set<int> slave_indices_seen;
    int max_slave_index = -1;
    int joint_seq = 0;

    for (const auto& item : slaves_pt) {
      const boost::property_tree::ptree& slave = item.second;
      const int sidx = slave.get<int>("index");
      if (sidx < 0 || sidx >= SLAVE_NUMBER) {
        *err_out = "slaves[].index 越界或无效";
        return false;
      }
      if (!slave_indices_seen.insert(sidx).second) {
        *err_out = "重复的 slaves[].index";
        return false;
      }
      max_slave_index = std::max(max_slave_index, sidx);

      TenkunSlaveStationConfig sc;
      sc.slave_index = sidx;
      std::unordered_set<unsigned> ch_seen_slave;

      const boost::property_tree::ptree& joints_pt = slave.get_child("joints");
      for (const auto& jitem : joints_pt) {
        const boost::property_tree::ptree& j = jitem.second;
        const std::string name = j.get<std::string>("name");
        const unsigned can_id = j.get<unsigned>("can_id");
        const unsigned channel = j.get<unsigned>("channel");
        if (name.empty()) {
          *err_out = "关节 name 不能为空";
          return false;
        }
        if (joint_names.count(name) != 0u) {
          *err_out = "重复的关节 name: " + name;
          return false;
        }
        joint_names.insert(name);
        if (channel < 1u || channel > static_cast<unsigned>(TENKUN_MOTORS_PER_SLAVE)) {
          *err_out = "关节 channel 须在 1.." + std::to_string(TENKUN_MOTORS_PER_SLAVE) + ": " + name;
          return false;
        }
        if (can_id == 0u || can_id > 0x7ffu) {
          *err_out = "关节 can_id 须在 1..2047（11 位）: " + name;
          return false;
        }
        if (!ch_seen_slave.insert(channel).second) {
          *err_out = "同从站内 channel 重复: " + name;
          return false;
        }
        TenkunMotorBinding b;
        b.channel_1_to_12 = channel;
        b.can_tx_id = static_cast<uint16_t>(can_id);
        sc.motors.push_back(b);

        TenkunRtiJointInfo info;
        info.joint_index = joint_seq++;
        info.name = name;
        info.slave_index = sidx;
        info.channel_1_to_12 = channel;
        info.can_tx_id = static_cast<uint16_t>(can_id);
        joints_out->push_back(std::move(info));
      }
      stations_out->push_back(std::move(sc));
    }

    if (joints_cmd_order_out != nullptr) {
      const auto jn_in_doc = doc.get_child_optional("joints_names");
      const auto jn_in_root = root.get_child_optional("joints_names");
      const boost::property_tree::ptree* jn_pt = nullptr;
      if (jn_in_doc) {
        jn_pt = &(*jn_in_doc);
      } else if (jn_in_root) {
        jn_pt = &(*jn_in_root);
      }
      if (jn_pt == nullptr) {
        *err_out = "RTI JSON 缺少 joints_names 数组（须与 slaves 关节集合一致且列出顺序即指令顺序）";
        return false;
      }
      std::vector<std::string> order;
      std::unordered_set<std::string> seen_in_order;
      order.reserve(joints_out->size());
      for (const auto& item : *jn_pt) {
        const std::string nm = item.second.get_value<std::string>();
        if (nm.empty()) {
          *err_out = "joints_names 含空字符串";
          return false;
        }
        if (!seen_in_order.insert(nm).second) {
          *err_out = "joints_names 重复: " + nm;
          return false;
        }
        if (joint_names.count(nm) == 0u) {
          *err_out = "joints_names 中的关节未在 slaves 中定义: " + nm;
          return false;
        }
        order.push_back(nm);
      }
      if (order.size() != joint_names.size()) {
        *err_out = "joints_names 数量与 slaves 中关节总数不一致（应为 " + std::to_string(joint_names.size()) +
                   " 个）";
        return false;
      }
      *joints_cmd_order_out = std::move(order);
    }

    if (default_pos_out != nullptr) {
      default_pos_out->clear();
      const auto dp_doc = doc.get_child_optional("default_pos");
      const auto dp_root = root.get_child_optional("default_pos");
      const boost::property_tree::ptree* dp_pt = nullptr;
      if (dp_doc) {
        dp_pt = &(*dp_doc);
      } else if (dp_root) {
        dp_pt = &(*dp_root);
      }
      if (dp_pt != nullptr) {
        for (const auto& item : *dp_pt) {
          try {
            default_pos_out->push_back(item.second.get_value<float>());
          } catch (const std::exception&) {
            *err_out = "default_pos 含无法解析为浮点数的元素";
            return false;
          }
        }
        const size_t expect =
            (joints_cmd_order_out != nullptr) ? joints_cmd_order_out->size() : joints_out->size();
        if (default_pos_out->size() != expect) {
          *err_out = "default_pos 长度须与关节指令顺序数量一致（期望 " + std::to_string(expect) + "，实际 " +
                     std::to_string(default_pos_out->size()) + "）";
          return false;
        }
      }
    }

    std::sort(stations_out->begin(), stations_out->end(),
              [](const TenkunSlaveStationConfig& a, const TenkunSlaveStationConfig& b) {
                return a.slave_index < b.slave_index;
              });

    return true;
  } catch (const std::exception& ex) {
    *err_out = ex.what();
    return false;
  }
}

void TenkunLogMappedChannelHealthImpl(int settle_ms,
                                      const std::vector<std::unique_ptr<TenkunSlaveStation>>& stations) {
  if (stations.empty()) {
    return;
  }
  int ms_total = settle_ms < 400 ? 400 : settle_ms;
  const int wait1 = (ms_total * 2 / 5) < 100 ? 100 : (ms_total * 2 / 5);
  const int wait2_raw = ms_total - wait1;
  const int wait2 = wait2_raw < 200 ? 200 : wait2_raw;

  std::this_thread::sleep_for(std::chrono::milliseconds(wait1));
  for (const auto& st : stations) {
    if (st) {
      st->EnqueueAllMappedChannelQueries();
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(wait2));

  const int smax = EcProcessSlaveLimit();
  const int ec = ec_slavecount;
  std::printf(
      "[Tenkun] 拓扑绑定 — 阶段 A：仅看 0x17 读状态后的反馈（期望 16B 0x80 状态帧）。\n"
      "  使能后 %d ms → 下发各绑定通道 0x17 → 再等 %d ms 采样；ec_slavecount=%d，应用有效从站数=%d\n"
      "  若电机只应答「关节 ID 广播」的 2B 帧、暂不答 0x17，此处 motor_id 可能为 0，属正常现象，以阶段 B 为准。\n",
      wait1, wait2, ec, smax);

  int warned = 0;
  for (const auto& st : stations) {
    const int s = st->SlaveIndex();
    for (const auto& b : st->Bindings()) {
      const unsigned ch = b.channel_1_to_12;
      const uint16_t expect = b.can_tx_id;

      if (s >= ec) {
        std::printf(
            "  [WARNING] A: 从站 %d  逻辑通道 %u  配置 CAN ID=%u — 总线上无此从站 (ec_slavecount=%d)\n", s, ch,
            static_cast<unsigned>(expect), ec);
        warned++;
        continue;
      }
      if (s >= smax) {
        std::printf(
            "  [WARNING] A: 从站 %d  逻辑通道 %u  配置 CAN ID=%u — 超出应用从站范围 (effective_slaves=%d，检查 "
            "slaves 配置)\n",
            s, ch, static_cast<unsigned>(expect), smax);
        warned++;
        continue;
      }

      const TenkunCanFdRxInfo rv = st->LastFeedbackForChannel(ch);
      const uint16_t got = TenkunRxBindCompareId(rv);
      if (got == 0u) {
        std::printf(
            "  [WARNING] A: 从站 %d  逻辑通道 %u  配置 CAN ID=%u — 0x17 后反馈仍为 motor_id=0（若总线通，"
            "常见是未回 16B 状态，请看阶段 B/C）\n",
            s, ch, static_cast<unsigned>(expect));
        warned++;
      } else if (got != expect) {
        std::printf(
            "  [WARNING] A: 从站 %d  逻辑通道 %u  配置 CAN ID=%u — 反馈 motor_id=%u 与配置不一致\n", s, ch,
            static_cast<unsigned>(expect), static_cast<unsigned>(got));
        warned++;
      } else {
        std::printf("  OK      A: 从站 %d  逻辑通道 %u  配置 CAN ID=%u — 0x17 后 motor_id 一致\n", s, ch,
                    static_cast<unsigned>(expect));
      }
    }
  }

  if (warned > 0) {
    std::printf("[Tenkun] 阶段 A 结束：%d 条 WARNING（↑ 不表示离线；阶段 B 广播后会再复核）\n", warned);
  } else {
    std::printf("[Tenkun] 阶段 A：0x17 后各已绑定通道反馈与 map 一致。\n");
  }

  std::printf(
      "\n[Tenkun] 阶段 B：关节 ID 广播（CAN ID=0、8B 全 0，经逻辑通道 1 → 物理 CAN FD0），打印总线应答关节 ID\n");
  constexpr int kBroadcastSettleMs = 700;
  for (const auto& st : stations) {
    const int s = st->SlaveIndex();
    if (s >= smax || s >= ec) {
      continue;
    }
    std::printf("[Tenkun] 从站 %d：下发广播并等待 %d ms…\n", s, kBroadcastSettleMs);
    Tenkun_ResetCanFdIdScanResults(s);
    EtherCAT_Msg_ptr bmsg(new EtherCAT_Msg);
    std::memset(bmsg->raw, 0, sizeof(bmsg->raw));
    Tenkun_FillBroadcastIdQueryAllCanFdSlots(bmsg.get());
    SoemSendToQueue(s, bmsg);
    std::this_thread::sleep_for(std::chrono::milliseconds(kBroadcastSettleMs));
    Tenkun_PrintCanFdIdScanResults(s);
  }

  std::printf("\n[Tenkun] 阶段 C：广播完成后再次读取各槽反馈（2B 0x80+关节 ID 会写入 motor_id）\n");
  int warned_c = 0;
  for (const auto& st : stations) {
    const int s = st->SlaveIndex();
    if (s >= ec || s >= smax) {
      continue;
    }
    for (const auto& b : st->Bindings()) {
      const unsigned ch = b.channel_1_to_12;
      const uint16_t expect = b.can_tx_id;
      const TenkunCanFdRxInfo rv = st->LastFeedbackForChannel(ch);
      const uint16_t got_c = TenkunRxBindCompareId(rv);
      if (got_c == 0u) {
        std::printf(
            "  [WARNING] C 复核: 从站 %d  逻辑通道 %u  配置 CAN ID=%u — 广播后反馈仍为 motor_id=0\n", s, ch,
            static_cast<unsigned>(expect));
        warned_c++;
      } else if (got_c != expect) {
        std::printf(
            "  [WARNING] C 复核: 从站 %d  逻辑通道 %u  配置 CAN ID=%u — 广播后 motor_id=%u 与配置不一致\n", s, ch,
            static_cast<unsigned>(expect), static_cast<unsigned>(got_c));
        warned_c++;
      } else {
        std::printf("  OK      C 复核: 从站 %d  逻辑通道 %u  配置 CAN ID=%u — 广播后 motor_id 一致\n", s, ch,
                    static_cast<unsigned>(expect));
      }
    }
  }
  if (warned_c > 0) {
    std::printf("[Tenkun] 阶段 C 结束：%d 条 WARNING。\n", warned_c);
  } else {
    std::printf("[Tenkun] 阶段 C：广播后反馈与绑定 CAN ID 一致。\n");
  }
}

}  // namespace

void TenKunRti::PostStartHealthCheck() {
  TenkunLogMappedChannelHealthImpl(800, tenkun_stations_);
}

TenKunRti* TenKunRti::ActiveInstance() {
  return s_active_.load(std::memory_order_acquire);
}

void TenKunRti::RegisterStation(int slave_idx, TenkunSlaveStation* p) {
  TenKunRti* a = ActiveInstance();
  if (a == nullptr || slave_idx < 0 || slave_idx >= SLAVE_NUMBER) {
    return;
  }
  a->station_slots_[static_cast<size_t>(slave_idx)] = p;
}

void TenKunRti::UnregisterStation(int slave_idx, TenkunSlaveStation* p) {
  TenKunRti* a = ActiveInstance();
  if (a == nullptr || slave_idx < 0 || slave_idx >= SLAVE_NUMBER) {
    return;
  }
  const size_t i = static_cast<size_t>(slave_idx);
  if (a->station_slots_[i] == p) {
    a->station_slots_[i] = nullptr;
  }
}

TenkunSlaveStation* TenKunRti::FindStation(int slave_idx) {
  TenKunRti* a = ActiveInstance();
  if (a == nullptr || slave_idx < 0 || slave_idx >= SLAVE_NUMBER) {
    return nullptr;
  }
  return a->station_slots_[static_cast<size_t>(slave_idx)];
}

void TenKunRti::DispatchProcessInputs(int slave_idx, const EtherCAT_Msg* rx) {
  if (rx == nullptr || slave_idx < 0 || slave_idx >= SLAVE_NUMBER) {
    return;
  }
  TenKunRti* a = ActiveInstance();
  if (a == nullptr) {
    return;
  }
  TenkunSlaveStation* st = a->station_slots_[static_cast<size_t>(slave_idx)];
  if (st != nullptr) {
    st->FeedProcessInputs(*rx);
  }
}

uint8_t TenKunRti::NextTxSeq(int slave_idx, unsigned channel_1_to_12) {
  if (slave_idx < 0 || slave_idx >= SLAVE_NUMBER || channel_1_to_12 < 1u ||
      channel_1_to_12 > static_cast<unsigned>(TENKUN_MOTORS_PER_SLAVE)) {
    return 0;
  }
  TenKunRti* a = ActiveInstance();
  if (a == nullptr) {
    return 0;
  }
  const size_t si = static_cast<size_t>(slave_idx);
  TenkunSlaveStation* st = a->station_slots_[si];
  if (st != nullptr) {
    return st->BumpSeq(channel_1_to_12);
  }
  const unsigned mi = channel_1_to_12 - 1u;
  a->fallback_seq_[si][mi]++;
  return a->fallback_seq_[si][mi];
}

bool MotorIndexForChannel(const TenkunSlaveStation& st, unsigned channel_1_to_12, size_t* motor_index_out) {
  const auto& b = st.Bindings();
  for (size_t m = 0; m < b.size(); ++m) {
    if (b[m].channel_1_to_12 == channel_1_to_12) {
      *motor_index_out = m;
      return true;
    }
  }
  return false;
}

TenKunRti::TenKunRti(const std::string& rti_config_path) {
  std::string err_if;
  ifname_ = ResolveEtherCatInterface(rti_config_path, &err_if);
  if (ifname_.empty()) {
    std::fprintf(stderr, "[TenKunRti] %s\n", err_if.c_str());
    std::exit(1);
  }

  std::printf("[tenkun_rti] interface: %s\n", ifname_.c_str());

  std::vector<TenkunSlaveStationConfig> station_cfgs;

  if (!rti_config_path.empty() && FileStartsWithJsonObject(rti_config_path)) {
    std::string err;
    if (!ParseRtiJson(rti_config_path, &station_cfgs, &joints_, &joints_cmd_order_, &default_pos_rad_, &err)) {
      std::fprintf(stderr, "[TenKunRti] RTI JSON 无效 (%s): %s\n", rti_config_path.c_str(), err.c_str());
      std::exit(1);
    }
    if (!default_pos_rad_.empty()) {
      std::printf("[tenkun_rti] default_pos: %zu value(s) (rad, same order as joints_names)\n",
                  default_pos_rad_.size());
    }
  }
  for (const auto& j : joints_) {
    const auto ord_it = std::find(joints_cmd_order_.begin(), joints_cmd_order_.end(), j.name);
    const size_t ord_i = static_cast<size_t>(ord_it - joints_cmd_order_.begin());
    char def_col[20];
    if (ord_it != joints_cmd_order_.end() && ord_i < default_pos_rad_.size()) {
      std::snprintf(def_col, sizeof(def_col), "%10.4f", static_cast<double>(default_pos_rad_[ord_i]));
    } else {
      std::snprintf(def_col, sizeof(def_col), "%10s", "-");
    }
    std::printf("[tenkun_rti] joint[%2d] %-26s  slave=%2d  ch=%2u  can_id=%4u  def_rad=%s\n", j.joint_index,
                j.name.c_str(), j.slave_index, j.channel_1_to_12, static_cast<unsigned>(j.can_tx_id), def_col);
  }

  EtherCAT_Init(ifname_.c_str());

  s_active_.store(this, std::memory_order_release);

  for (auto& cfg : station_cfgs) {
    tenkun_stations_.push_back(std::make_unique<TenkunSlaveStation>(std::move(cfg)));
  }
  for (const std::string& name : joints_cmd_order_) {
    const TenkunRtiJointInfo* jinfo = FindJointByName(name);
    if (jinfo == nullptr) {
      std::printf("[tenkun_rti] joint[%s] not found\n", name.c_str());
      exit(1);
    }
    TenkunSlaveStation* st = GetTenkunStation(jinfo->slave_index);
    if (st == nullptr) {
      std::printf("[tenkun_rti] station[%d] not found\n", jinfo->slave_index);
      exit(1);
    }
    size_t motor_index = 0;
    if (!MotorIndexForChannel(*st, jinfo->channel_1_to_12, &motor_index)) {
      std::printf("[tenkun_rti] motor[%d] not found\n", jinfo->channel_1_to_12);
      exit(1);
    }
    st->SetHybridImpedance(motor_index, 0, 0, 0, 0, 0);
  }

  startRun();
  // TenkunLogMappedChannelHealth(800);
  for (const std::string& name : joints_cmd_order_) {
    const TenkunRtiJointInfo* jinfo = FindJointByName(name);
    if (jinfo == nullptr) {
      std::printf("[tenkun_rti] joint[%s] not found\n", name.c_str());
      exit(1);
    }
    TenkunSlaveStation* st = GetTenkunStation(jinfo->slave_index);
    if (st == nullptr) {
      std::printf("[tenkun_rti] station[%d] not found\n", jinfo->slave_index);
      exit(1);
    }
    size_t motor_index = 0;
    if (!MotorIndexForChannel(*st, jinfo->channel_1_to_12, &motor_index)) {
      std::printf("[tenkun_rti] motor[%d] not found\n", jinfo->channel_1_to_12);
      exit(1);
    }
    printf("Initializing joint %s .............\n", name.c_str());
    int count = 1;
    while (st->GetMotorStatus(motor_index).recv_cnt < 10) {
      auto rx = st->GetMotorStatus(motor_index);
      if (rx.recv_cnt > 0) {
        st->SetHybridImpedance(motor_index, 0, 0, rx.pos_rad, 0, 0);
      }
      st->QueryMotorStatus(motor_index);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      if (count++ % 100 == 0) {
        std::printf("[tenkun_rti] joint[%s] waiting for feedback ...\n", name.c_str());
      }
      if (count > 1000) {
        std::printf("[tenkun_rti] joint[%s] initialization failed\n", name.c_str());
        if (name.compare("right_wrist_yaw_joint") != 0) //临时屏蔽损坏关节
          exit(1);
        else 
          break;
      }
    }
    std::printf("[tenkun_rti] joint[%s] initialized successfully\n", name.c_str());
    PrintTenkunRxFeedback(name.c_str(), st->GetMotorStatus(motor_index));
   
  }

}

TenKunRti::~TenKunRti() {
  stopRun();
  tenkun_stations_.clear();
  s_active_.store(nullptr, std::memory_order_release);
  std::printf("[TenKunRti] EtherCAT cyclic thread stopped.\n");
}

const TenkunRtiJointInfo* TenKunRti::FindJointByName(const std::string& name) const {
  for (const auto& j : joints_) {
    if (j.name == name) {
      return &j;
    }
  }
  return nullptr;
}

bool TenKunRti::SetJointsCmdOrdered(const std::vector<MotorCmdType>& cmds) {
  if (cmds.size() != joints_cmd_order_.size()) {
    std::fprintf(stderr, "[TenKunRti] SetJointsCmdOrdered: 期望 %zu 个命令，实际 %zu\n", joints_cmd_order_.size(),
                 cmds.size());
    return false;
  }
  for (size_t i = 0; i < cmds.size(); ++i) {
    const TenkunRtiJointInfo* jinfo = FindJointByName(joints_cmd_order_[i]);
    if (jinfo == nullptr) {
      std::fprintf(stderr, "[TenKunRti] SetJointsCmdOrdered: 未找到关节 \"%s\"\n", joints_cmd_order_[i].c_str());
      return false;
    }
    TenkunSlaveStation* st = GetTenkunStation(jinfo->slave_index);
    if (st == nullptr) {
      std::fprintf(stderr, "[TenKunRti] SetJointsCmdOrdered: 从站 %d 无 TenkunStation\n", jinfo->slave_index);
      return false;
    }
    size_t motor_index = 0;
    if (!MotorIndexForChannel(*st, jinfo->channel_1_to_12, &motor_index)) {
      std::fprintf(stderr, "[TenKunRti] SetJointsCmdOrdered: 从站 %d 无逻辑通道 %u\n", jinfo->slave_index,
                   jinfo->channel_1_to_12);
      return false;
    }
    st->SetHybridImpedance(motor_index, cmds[i].kp, cmds[i].kd, cmds[i].pos, cmds[i].spd, cmds[i].tor);
  }
  return true;
}

std::vector<TenkunCanFdRxInfo> TenKunRti::GetJointsStatusOrdered() const {
  std::vector<TenkunCanFdRxInfo> out;
  out.reserve(joints_cmd_order_.size());
  for (const std::string& name : joints_cmd_order_) {
    const TenkunRtiJointInfo* jinfo = FindJointByName(name);
    if (jinfo == nullptr) {
      out.push_back(TenkunCanFdRxInfo{});
      continue;
    }
    const TenkunSlaveStation* st = GetTenkunStation(jinfo->slave_index);
    if (st == nullptr) {
      out.push_back(TenkunCanFdRxInfo{});
      continue;
    }
    size_t motor_index = 0;
    if (!MotorIndexForChannel(*st, jinfo->channel_1_to_12, &motor_index)) {
      out.push_back(TenkunCanFdRxInfo{});
      continue;
    }
    out.push_back(st->GetMotorStatus(motor_index));
  }
  return out;
}

TenkunSlaveStation* TenKunRti::GetTenkunStation(int slave_index) {
  for (auto& p : tenkun_stations_) {
    if (p && p->SlaveIndex() == slave_index) {
      return p.get();
    }
  }
  return nullptr;
}

const TenkunSlaveStation* TenKunRti::GetTenkunStation(int slave_index) const {
  for (const auto& p : tenkun_stations_) {
    if (p && p->SlaveIndex() == slave_index) {
      return p.get();
    }
  }
  return nullptr;
}

void TenKunRti::RunControlAllSlaves() {
  for (int s = 0; s < SLAVE_NUMBER; ++s) {
    if (TenkunSlaveStation* st = GetTenkunStation(s)) {
      st->RunControlCycle();
    }
  }
}

void TenKunRti::SetMotorEnabledAll(bool enabled) {
  printf("===========SetMotorEnabledAll: %d\n", enabled);
  for (int s = 0; s < SLAVE_NUMBER; ++s) {
    if (TenkunSlaveStation* st = GetTenkunStation(s)) {
      for (size_t i = 0; i < st->MotorCount(); ++i) {
        st->SetMotorEnabled(i, enabled);
      }
    }
  }
}


void TenKunRti::TenkunScanCanFdMotorIds(int slave_idx) {
  if (slave_idx < 0 || slave_idx >= ec_slavecount) {
    std::printf("[TenKunRti] TenkunScanCanFdMotorIds: slave_idx=%d 无效 (ec_slavecount=%d)\n", slave_idx,
                ec_slavecount);
    return;
  }
  Tenkun_ResetCanFdIdScanResults(slave_idx);
  EtherCAT_Msg_ptr msg(new EtherCAT_Msg);
  std::memset(msg->raw, 0, sizeof(msg->raw));
  Tenkun_FillBroadcastIdQueryAllCanFdSlots(msg.get());
  if (TenkunSlaveStation* st = GetTenkunStation(slave_idx)) {
    st->EnqueueTx(msg);
  } else {
    SoemSendToQueue(slave_idx, msg);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  Tenkun_PrintCanFdIdScanResults(slave_idx);
}

void TenKunRti::TenkunBroadcastJointIdAllCanFdSlots(int slave_idx) {
  if (slave_idx < 0 || slave_idx >= ec_slavecount) {
    std::printf("[TenKunRti] TenkunBroadcastJointIdAllCanFdSlots: slave_idx=%d 无效 (ec_slavecount=%d)\n",
                slave_idx, ec_slavecount);
    return;
  }
  Tenkun_ResetCanFdIdScanResults(slave_idx);
  EtherCAT_Msg_ptr msg(new EtherCAT_Msg);
  std::memset(msg->raw, 0, sizeof(msg->raw));
  Tenkun_FillBroadcastJointIdQueryAllCanFdChannels(msg.get());
  if (TenkunSlaveStation* st = GetTenkunStation(slave_idx)) {
    st->EnqueueTx(msg);
  } else {
    SoemSendToQueue(slave_idx, msg);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  Tenkun_PrintCanFdIdScanResults(slave_idx);
}

void TenKunRti::TenkunEnqueueCih408SetJointId(int slave_idx, unsigned channel_1_to_15, uint16_t can_tx_id,
                                              uint8_t old_joint_id, uint8_t new_joint_id) {
  if (slave_idx < 0 || slave_idx >= ec_slavecount) {
    std::printf("[TenKunRti] TenkunEnqueueCih408SetJointId: slave_idx=%d 无效\n", slave_idx);
    return;
  }
  if (channel_1_to_15 < 1u || channel_1_to_15 > static_cast<unsigned>(TENKUN_MOTORS_PER_SLAVE)) {
    std::printf("[TenKunRti] TenkunEnqueueCih408SetJointId: channel 须在 1..%u\n",
                static_cast<unsigned>(TENKUN_MOTORS_PER_SLAVE));
    return;
  }
  EtherCAT_Msg_ptr msg(new EtherCAT_Msg);
  std::memset(msg->raw, 0, sizeof(msg->raw));
  TenkunGatewaySlotView sv;
  TenkunGatewayGetSlot(msg->raw, channel_1_to_15, &sv);
  if (sv.bytes == nullptr) {
    std::printf("[TenKunRti] TenkunEnqueueCih408SetJointId: 逻辑通道 %u 不是 CAN FD 槽\n", channel_1_to_15);
    return;
  }
  auto* fd = reinterpret_cast<TenkunGatewayCanFdSlot*>(sv.bytes);
  fd->id_le = can_tx_id;
  fd->type = TENKUN_TYPE_DATA_STD;
  fd->length = 3;
  TenkunCih408PackSetJointId(fd->data, sizeof(fd->data), old_joint_id, new_joint_id);
  if (TenkunSlaveStation* st = GetTenkunStation(slave_idx)) {
    st->EnqueueTx(msg);
  } else {
    SoemSendToQueue(slave_idx, msg);
  }
}

bool TenKunRti::TenkunZeroCalibrateMotorOrdered(size_t motor_cmd_order_index) {
  constexpr int kRetryPeriodMs = 100;
  constexpr int kMaxAttempts = 20;
  constexpr int kPollStepMs = 2;

  if (motor_cmd_order_index >= joints_cmd_order_.size()) {
    std::fprintf(stderr, "[TenKunRti] TenkunZeroCalibrateMotorOrdered: 下标 %zu 越界（joints_names 共 %zu 个）\n",
                 motor_cmd_order_index, joints_cmd_order_.size());
    return false;
  }
  const std::string& name = joints_cmd_order_[motor_cmd_order_index];
  const TenkunRtiJointInfo* jinfo = FindJointByName(name);
  if (jinfo == nullptr) {
    std::fprintf(stderr, "[TenKunRti] TenkunZeroCalibrateMotorOrdered: 未找到关节 \"%s\"\n", name.c_str());
    return false;
  }
  TenkunSlaveStation* st = GetTenkunStation(jinfo->slave_index);
  if (st == nullptr) {
    std::fprintf(stderr, "[TenKunRti] TenkunZeroCalibrateMotorOrdered: 从站 %d 无 TenkunStation\n", jinfo->slave_index);
    return false;
  }
  size_t motor_index = 0;
  if (!MotorIndexForChannel(*st, jinfo->channel_1_to_12, &motor_index)) {
    std::fprintf(stderr, "[TenKunRti] TenkunZeroCalibrateMotorOrdered: 从站 %d 无逻辑通道 %u\n", jinfo->slave_index,
                 jinfo->channel_1_to_12);
    return false;
  }

  st->ClearZeroCalibrateAckState(motor_index);

  auto send_zero_cal = [&]() {
    EtherCAT_Msg_ptr msg(new EtherCAT_Msg);
    std::memset(msg->raw, 0, sizeof(msg->raw));
    TenkunGatewaySlotView sv{};
    TenkunGatewayGetSlot(msg->raw, jinfo->channel_1_to_12, &sv);
    if (sv.bytes == nullptr) {
      std::fprintf(stderr, "[TenKunRti] TenkunZeroCalibrateMotorOrdered: 通道 %u 非 CAN FD 槽\n",
                   jinfo->channel_1_to_12);
      return false;
    }
    auto* fd = reinterpret_cast<TenkunGatewayCanFdSlot*>(sv.bytes);
    fd->id_le = jinfo->can_tx_id;
    fd->type = TENKUN_TYPE_DATA_STD;
    fd->length = 1;
    TenkunCih408PackSetZero(fd->data, sizeof(fd->data));
    st->EnqueueTx(msg);
    return true;
  };

  for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
    if (!send_zero_cal()) {
      return false;
    }
    const auto period_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(kRetryPeriodMs);
    while (std::chrono::steady_clock::now() < period_end) {
      const TenkunCanFdRxInfo rx = st->GetMotorStatus(motor_index);
      if (rx.kind == TENKUN_RX_PARSE_SET_ZERO_RSP && rx.zero_set_ok != 0u) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(kPollStepMs));
    }
  }

  std::fprintf(stderr, "[TenKunRti] TenkunZeroCalibrateMotorOrdered: 关节 \"%s\" 经 %d 次（每 %d ms）仍未收到零点标定成功应答\n",
                 name.c_str(), kMaxAttempts, kRetryPeriodMs);
  return false;
}

void TenKunRti::TenkunLogMappedChannelHealth(int settle_ms) {
  TenkunLogMappedChannelHealthImpl(settle_ms, tenkun_stations_);
}

}  // namespace XJDLRobot
