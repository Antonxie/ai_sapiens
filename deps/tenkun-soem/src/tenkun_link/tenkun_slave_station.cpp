#include "tenkun_slave_station.hpp"

#include "tenkun_rti.hpp"
#include "transmit_soem_core.h"
extern "C" {
#include "tenkun_canfd_parse.h"
#include "tenkun_cih408.h"
#include "tenkun_gateway_pdo.h"
}

#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <unordered_set>
#include <unistd.h>

extern "C" {
#include "ethercat.h"
}


namespace {

void TrimInPlace(std::string* s) {
  if (s == nullptr) {
    return;
  }
  while (!s->empty() && std::isspace(static_cast<unsigned char>((*s)[0]))) {
    s->erase(s->begin());
  }
  while (!s->empty() && std::isspace(static_cast<unsigned char>(s->back()))) {
    s->pop_back();
  }
}

int EcProcessSlaveLimit() {
  int n = ec_slavecount;
  if (n > SLAVE_NUMBER) {
    n = SLAVE_NUMBER;
  }
  return n > 0 ? n : 0;
}

float TenkunClampOrDefault(float v, float def_lo, float def_hi) {
  return v > def_hi ? def_hi : v < def_lo ? def_lo : v;
}
}  // namespace


namespace XJDLRobot {

TenkunSlaveStation::TenkunSlaveStation(TenkunSlaveStationConfig config) : slave_index_(config.slave_index) {
  if (slave_index_ < 0 || slave_index_ >= SLAVE_NUMBER) {
    slave_index_ = 0;
  }
  std::unordered_set<unsigned> seen_ch;
  for (const auto& m : config.motors) {
    if (m.channel_1_to_12 < 1u || m.channel_1_to_12 > static_cast<unsigned>(TENKUN_MOTORS_PER_SLAVE) ||
        m.can_tx_id == 0u || m.can_tx_id > 0x7ffu) {
      continue;
    }
    if (seen_ch.count(m.channel_1_to_12) != 0u) {
      continue;
    }
    seen_ch.insert(m.channel_1_to_12);
    motors_.push_back(m);
  }
  RebuildCommandArrays();
  TenKunRti::RegisterStation(slave_index_, this);
}

TenkunSlaveStation::~TenkunSlaveStation() { TenKunRti::UnregisterStation(slave_index_, this); }

void TenkunSlaveStation::RebuildCommandArrays() {
  const size_t n = motors_.size();
  enabled_desired_.assign(n, 0);
  enable_pending_.assign(n, 0);
  hybrid_kp_.assign(n, 0.0F);
  hybrid_kd_.assign(n, 0.0F);
  hybrid_pos_.assign(n, 0.0F);
  hybrid_vel_.assign(n, 0.0F);
  hybrid_torque_.assign(n, 0.0F);
}



uint16_t TenkunSlaveStation::ResolveCanIdForChannel(unsigned channel_1_to_12) const {
  for (const auto& m : motors_) {
    if (m.channel_1_to_12 == channel_1_to_12) {
      return m.can_tx_id;
    }
  }
  return 0u;
}

TenkunCanFdRxInfo TenkunSlaveStation::LastFeedbackForChannel(unsigned channel_1_to_12) const {
  TenkunCanFdRxInfo z{};
  if (channel_1_to_12 < 1u || channel_1_to_12 > static_cast<unsigned>(TENKUN_MOTORS_PER_SLAVE)) {
    return z;
  }
  std::lock_guard<std::mutex> lock(rx_mu_);
  return slot_rx_[channel_1_to_12 - 1u];
}

TenkunCanFdRxInfo TenkunSlaveStation::GetMotorStatus(size_t motor_index) const {
  if (motor_index >= motors_.size()) {
    return TenkunCanFdRxInfo{};
  }
  const unsigned ch = motors_[motor_index].channel_1_to_12;
  if (ch < 1u || ch > static_cast<unsigned>(TENKUN_MOTORS_PER_SLAVE)) {
    return TenkunCanFdRxInfo{};
  }
  std::lock_guard<std::mutex> lock(rx_mu_);
  return slot_rx_[ch - 1u];
}

void TenkunSlaveStation::ClearZeroCalibrateAckState(size_t motor_index) {
  if (motor_index >= motors_.size()) {
    return;
  }
  const unsigned ch = motors_[motor_index].channel_1_to_12;
  if (ch < 1u || ch > static_cast<unsigned>(TENKUN_MOTORS_PER_SLAVE)) {
    return;
  }
  std::lock_guard<std::mutex> lock(rx_mu_);
  slot_rx_[ch - 1u].zero_set_ok = 0u;
}

bool TenkunSlaveStation::IsMotorFeedbackEnabled(size_t motor_index) const {
  const TenkunCanFdRxInfo rx = GetMotorStatus(motor_index);
  return rx.enable_on != 0u;
}

void TenkunSlaveStation::UnpackOneSlot(unsigned channel_1_to_15, const TenkunGatewaySlotView* sv) {
  if (sv == nullptr || sv->bytes == nullptr) {
    return;
  }
  const unsigned mi = channel_1_to_15 - 1u;
  const auto* fd = reinterpret_cast<const TenkunGatewayCanFdSlot*>(sv->bytes);
  const uint32_t id_can = fd->id_le;
  const uint8_t len = fd->length;
  const uint8_t* data = fd->data;

  std::lock_guard<std::mutex> lock(rx_mu_);
  TenkunCanFdRxInfo* rv = &slot_rx_[mi];

  if (id_can == 0 && len == 0) {
    return;
  }

  TenkunCanFdRxInfo info;
  TenkunCanFdParseSlotPayload(id_can, data, len, &info);

  switch (info.kind) {
    case TENKUN_RX_PARSE_JOINT_ID_RSP:
      // *rv = info;
      id_scan_hit_[mi] = 1u;
      id_scan_joint_id_[mi] = static_cast<uint8_t>(info.joint_id);
      return;
    case TENKUN_RX_PARSE_ENABLE_RSP:
      if (info.can_id_11 == ResolveCanIdForChannel(channel_1_to_15)) {
        rv->enable_on = info.enable_on;
      }
      return;
    case TENKUN_RX_PARSE_SET_JOINT_ID_RSP:
      if (info.can_id_11 == ResolveCanIdForChannel(channel_1_to_15)) {
        *rv = info;
      }
      id_scan_hit_[mi] = 1u;
      id_scan_joint_id_[mi] = static_cast<uint8_t>(info.set_joint_new_id);
      return;
    case TENKUN_RX_PARSE_SET_ZERO_RSP:
      if (info.can_id_11 == ResolveCanIdForChannel(channel_1_to_15)) {
        rv->zero_set_ok = info.zero_set_ok;
        rv->zero_pos_rad = info.zero_pos_rad;
        rv->kind = TENKUN_RX_PARSE_SET_ZERO_RSP;
      }
      return;
    case TENKUN_RX_PARSE_STATUS_16: {
      /* STATUS_16 解析不携带 enable_on；保留槽内上次 ENABLE 应答的 enable_on，便于持续判断使能状态。 */
      if (info.can_id_11 == ResolveCanIdForChannel(channel_1_to_15)) {
        info.recv_cnt = rv->recv_cnt + 1;
        info.zero_set_ok = rv->zero_set_ok;
        info.enable_on = rv->enable_on;
        *rv = info;
        if (query_armed_[mi] != 0u) {
          query_armed_[mi] = 0u;
          query_complete_[mi] = 1u;
        }
      }
      
      // std::printf(
      //     "[TenkunSlaveStation] slave=%d ch=%u STATUS_16 rx_can_id=%u pos_rad=%.5f vel_rad_s=%.5f "
      //     "I_A=%.4f motor_temp_C≈%.1f mos_temp_C≈%.1f bus_V≈%.2f ctl_mode=%u err12=0x%03x err_lo=0x%02x\n",
      //     slave_index_, channel_1_to_15, static_cast<unsigned>(info.can_id_11),
      //     static_cast<double>(info.pos_rad), static_cast<double>(info.vel_rad_s),
      //     static_cast<double>(info.cur_a), static_cast<double>(info.motor_temp_c),
      //     static_cast<double>(info.mos_temp_c), static_cast<double>(info.bus_v),
      //     static_cast<unsigned>(info.mode_nibble), static_cast<unsigned>(info.error12 & 0xfffu),
      //     static_cast<unsigned>(info.error12 & 0xffu));
      return;
    }
    default:
      *rv = info;
      return;
  }
}

void TenkunSlaveStation::FeedProcessInputs(const EtherCAT_Msg& rx) {
  for (unsigned ch = 1u; ch <= static_cast<unsigned>(TENKUN_MOTORS_PER_SLAVE); ++ch) {
    TenkunGatewaySlotView sv{};
    TenkunGatewayGetSlot(const_cast<uint8_t*>(rx.raw), ch, &sv);
    UnpackOneSlot(ch, &sv);
  }
}

uint8_t TenkunSlaveStation::BumpSeq(unsigned channel_1_to_12) {
  if (channel_1_to_12 < 1u || channel_1_to_12 > static_cast<unsigned>(TENKUN_MOTORS_PER_SLAVE)) {
    return 0;
  }
  unsigned mi = channel_1_to_12 - 1u;
  seq_tx_[mi]++;
  return seq_tx_[mi];
}

void TenkunSlaveStation::FillAllQuerySlots(EtherCAT_Msg* tx) const {
  if (tx == nullptr) {
    return;
  }
  std::memset(tx->raw, 0, sizeof(tx->raw));
  for (const auto& m : motors_) {
    TenkunGatewaySlotView sv{};
    TenkunGatewayGetSlot(tx->raw, m.channel_1_to_12, &sv);
    if (sv.bytes == nullptr) {
      continue;
    }
    auto* fd = reinterpret_cast<TenkunGatewayCanFdSlot*>(sv.bytes);
    fd->id_le = static_cast<uint32_t>(m.can_tx_id);
    fd->type = TENKUN_TYPE_DATA_STD;
    fd->length = 1;
    TenkunCih408PackQuery(fd->data, sizeof(fd->data));
  }
}

void TenkunSlaveStation::ResetJointIdScan() {
  std::lock_guard<std::mutex> lock(rx_mu_);
  id_scan_hit_.fill(0);
  id_scan_joint_id_.fill(0);
}

void TenkunSlaveStation::PrintJointIdScanResults() const {
  std::lock_guard<std::mutex> lock(rx_mu_);
  std::printf("[Tenkun] CAN FD 关节 ID 扫描结果（从站 %d，逻辑通道 1..%d）\n", slave_index_, TENKUN_MOTORS_PER_SLAVE);
  int any = 0;
  for (unsigned ch = 1; ch <= static_cast<unsigned>(TENKUN_MOTORS_PER_SLAVE); ch++) {
    const unsigned mi = ch - 1u;
    if (!id_scan_hit_[mi]) {
      continue;
    }
    any = 1;
    const unsigned jid = static_cast<unsigned>(id_scan_joint_id_[mi]);
    const char* bus = "CAN_FD";
    unsigned ch_end = ch;
    while (ch_end < static_cast<unsigned>(TENKUN_MOTORS_PER_SLAVE)) {
      const unsigned next = ch_end + 1u;
      const unsigned mi_n = next - 1u;
      if (!id_scan_hit_[mi_n]) {
        break;
      }
      if (static_cast<unsigned>(id_scan_joint_id_[mi_n]) != jid) {
        break;
      }
      ch_end = next;
    }
    if (ch == ch_end) {
      std::printf("  网关逻辑通道 %2u  (%s)  ->  关节 ID = %u\n", ch, bus, jid);
    } else {
      std::printf(
          "  网关逻辑通道 %2u–%2u  (%s)  ->  关节 ID = %u（多槽内容相同，视为网关镜像或同总线应答）\n", ch, ch_end,
          bus, jid);
    }
    ch = ch_end;
  }
  if (!any) {
    std::printf("  （无应答：请确认电机已上电、CiH408 协议一致，且每路 CAN FD 上建议仅单关节。）\n");
  }
}

static bool PackEnableSlot(uint8_t* raw, const TenkunMotorBinding& b, bool on) {
  TenkunGatewaySlotView sv{};
  TenkunGatewayGetSlot(raw, b.channel_1_to_12, &sv);
  if (sv.bytes == nullptr) {
    return false;
  }
  auto* fd = reinterpret_cast<TenkunGatewayCanFdSlot*>(sv.bytes);
  fd->id_le = static_cast<uint32_t>(b.can_tx_id);
  fd->type = TENKUN_TYPE_DATA_STD;
  fd->length = 2;
  TenkunCih408PackEnable(fd->data, on ? 1 : 0);
  return true;
}

void TenkunSlaveStation::SetMotorEnabled(size_t motor_index, bool enabled) {
  std::lock_guard<std::mutex> lock(mu_);
  if (motor_index >= motors_.size()) {
    return;
  }
  const uint8_t want = enabled ? 1u : 0u;
  if (enabled_desired_[motor_index] != want) {
    enabled_desired_[motor_index] = want;
    enable_pending_[motor_index] = 1u;
  }
}

void TenkunSlaveStation::SetHybridImpedance(size_t motor_index, float kp, float kd, float pos_rad,
                                            float vel_rad_s, float torque_ff_nm) {
  std::lock_guard<std::mutex> lock(mu_);
  if (motor_index >= motors_.size()) {
    return;
  }
  hybrid_kp_[motor_index] = kp;
  hybrid_kd_[motor_index] = kd;
  hybrid_pos_[motor_index] = pos_rad;
  hybrid_vel_[motor_index] = vel_rad_s;
  hybrid_torque_[motor_index] = torque_ff_nm;
}

void TenkunSlaveStation::QueryMotorStatus(size_t motor_index) {
  // SetMotorEnabled(motor_index, true);
  std::lock_guard<std::mutex> lock(mu_);
  if (motor_index >= motors_.size()) {
    return;
  }
  const int slave_hi = EcProcessSlaveLimit();
  if (slave_index_ < 0 || slave_index_ >= slave_hi) {
    return;
  }
  
  EtherCAT_Msg_ptr msg(new EtherCAT_Msg);
  std::memset(msg->raw, 0, sizeof(msg->raw));
  TenkunGatewaySlotView sv;
  TenkunGatewayGetSlot(msg->raw, motors_[motor_index].channel_1_to_12, &sv);
  auto* fd = reinterpret_cast<TenkunGatewayCanFdSlot*>(sv.bytes);
  fd->id_le = motors_[motor_index].can_tx_id;
  fd->type = TENKUN_TYPE_DATA_STD;
  fd->length = 16;
  TenkunCih408PackVelocity(fd->data, 0, 0., 0);
  EnqueueTx(msg);

}

void TenkunSlaveStation::RunControlCycle() {
  std::lock_guard<std::mutex> lock(mu_);
  const int slave_hi = EcProcessSlaveLimit();
  if (slave_index_ < 0 || slave_index_ >= slave_hi) {
    printf("RunControlCycle slave_index_ < 0 || slave_index_ >= slave_hi %d\n", slave_index_);
    return;
  }

  EtherCAT_Msg_ptr msg(new EtherCAT_Msg);
  std::memset(msg->raw, 0, sizeof(msg->raw));
  bool any = false;

  /* 期望使能与 ENABLE 应答 enable_on 不一致时置 pending，本周期重发 ENABLE（含应关未关）。 */
  for (size_t i = 0; i < motors_.size(); ++i) {
    const bool want_enabled = enabled_desired_[i] != 0u;
    if (IsMotorFeedbackEnabled(i) != want_enabled) {
      enable_pending_[i] = 1u;
    }
  }

  std::vector<uint8_t> skip_hybrid_this_cycle(motors_.size(), 0);
  for (size_t i = 0; i < motors_.size(); ++i) {
    if (enable_pending_[i] == 0u) {
      continue;
    }
    if (PackEnableSlot(msg->raw, motors_[i], enabled_desired_[i] != 0u)) {
      any = true;
      skip_hybrid_this_cycle[i] = 1u;
    }
    enable_pending_[i] = 0u;
  }

  for (size_t i = 0; i < motors_.size(); ++i) {
    if (skip_hybrid_this_cycle[i] != 0u) {
      continue;
    }
    // if (enabled_desired_[i] == 0u) {
    //   continue;
    // }
    MotorCmdType cmd{};
    cmd.slave = slave_index_;
    cmd.chanel_id = motors_[i].channel_1_to_12;
    cmd.can_tx_id = motors_[i].can_tx_id;
    cmd.kp = hybrid_kp_[i];
    cmd.kd = hybrid_kd_[i];
    cmd.pos = hybrid_pos_[i];
    cmd.spd = hybrid_vel_[i];
    cmd.tor = hybrid_torque_[i];
    PackMotorCmdInto(msg, cmd, false);
    any = true;
  }

  if (any) {
    // printf("EnqueueTx %d\n", slave_index_);
    EnqueueTx(msg);
  }
}

void TenkunSlaveStation::EnqueueTx(const std::shared_ptr<EtherCAT_Msg>& msg) { SoemSendToQueue(slave_index_, msg); }

void TenkunSlaveStation::PackMotorCmdInto(const std::shared_ptr<EtherCAT_Msg>& msg, MotorCmdType cmd,
                                          bool submit_to_queue) {
  if (cmd.slave != slave_index_) {
    std::printf("[Tenkun SOEM] PackMotorCmdInto: cmd.slave=%d 与本对象从站 %d 不一致，已按本从站处理\n", cmd.slave,
                slave_index_);
  }
  if (cmd.chanel_id < 1 || cmd.chanel_id > TENKUN_MOTORS_PER_SLAVE) {
    std::printf("[Tenkun SOEM] PackMotorCmdInto: motor_id out of range: %u\n", static_cast<unsigned>(cmd.chanel_id));
    return;
  }

  TenkunGatewaySlotView sv;
  TenkunGatewayGetSlot(msg->raw, cmd.chanel_id, &sv);
  if (sv.bytes == nullptr) {
    std::printf("[Tenkun SOEM] PackMotorCmdInto: no PDO slot for channel %u\n", static_cast<unsigned>(cmd.chanel_id));
    return;
  }

  auto* fd = reinterpret_cast<TenkunGatewayCanFdSlot*>(sv.bytes);
  fd->id_le = cmd.can_tx_id;
  fd->type = TENKUN_TYPE_DATA_STD;
  fd->length = 16;
  const float kp =  TenkunClampOrDefault(cmd.kp,  0.0F, 5000.0F);
  const float kd =  TenkunClampOrDefault(cmd.kd,  0.0F, 500.0F);
  const float pos = TenkunClampOrDefault(cmd.pos, -12.5F, 12.5F);
  const float spd = TenkunClampOrDefault(cmd.spd, -18.0F, 18.0F);
  const float tor = TenkunClampOrDefault(cmd.tor, -200.0F, 200.0F);

  const uint16_t kp_u = static_cast<uint16_t>(kp + 0.5F);
  const uint16_t kd_u = static_cast<uint16_t>(kd + 0.5F);
  const  int16_t ff_i = static_cast<int16_t>(tor);

  const uint8_t seq = TenKunRti::NextTxSeq(slave_index_, cmd.chanel_id);

  TenkunCih408PackHybrid(fd->data, kp_u, kd_u, pos, spd, ff_i, seq);

  if (submit_to_queue) {
    EnqueueTx(msg);
  }
}

void TenkunSlaveStation::EnqueueAllMappedChannelQueries() {
  const int s_end = EcProcessSlaveLimit();
  if (slave_index_ < 0 || slave_index_ >= s_end || slave_index_ >= ec_slavecount) {
    return;
  }
  if (motors_.empty()) {
    return;
  }
  EtherCAT_Msg_ptr m(new EtherCAT_Msg);
  FillAllQuerySlots(m.get());
  EnqueueTx(m);
}

}  // namespace XJDLRobot



void SoemProtocol_EtherCAT_Data_Get(void) {
  for (int slave = 0; slave < ec_slavecount; ++slave) {
    auto* slave_src = reinterpret_cast<EtherCAT_Msg*>(ec_slave[slave + 1].inputs);
    if (slave_src == nullptr) {
      continue;
    }
    EtherCAT_Msg rx = *slave_src;
    read_num++;
    XJDLRobot::TenKunRti::DispatchProcessInputs(slave, &rx);
    const uint8_t ack_status = 0;
    if (is_config[slave]) {
      is_config[slave] = false;
      Rv_Message_Print(slave, ack_status);
    }
  }
}

extern "C" {

void Tenkun_FillBroadcastIdQueryAllCanFdSlots(EtherCAT_Msg* tx) {
  if (tx == nullptr) {
    return;
  }
  const unsigned ch = 1u;
  if (ch < 1u || ch > static_cast<unsigned>(TENKUN_MOTORS_PER_SLAVE)) {
    return;
  }
  TenkunGatewaySlotView sv{};
  TenkunGatewayGetSlot(tx->raw, ch, &sv);
  if (sv.bytes == nullptr) {
    return;
  }
  auto* fd = reinterpret_cast<TenkunGatewayCanFdSlot*>(sv.bytes);
  fd->id_le = 0u;
  fd->type = TENKUN_TYPE_DATA_STD;
  fd->length = static_cast<uint8_t>(TENKUN_CIH408_JOINT_ID_QUERY_TX_DLC);
  TenkunCih408PackJointIdQueryBroadcast(fd->data);
}

void Tenkun_FillBroadcastJointIdQueryAllCanFdChannels(EtherCAT_Msg* tx) {
  if (tx == nullptr) {
    return;
  }
  for (unsigned ch = 1u; ch <= static_cast<unsigned>(TENKUN_MOTORS_PER_SLAVE); ch++) {
    TenkunGatewaySlotView sv{};
    TenkunGatewayGetSlot(tx->raw, ch, &sv);
    if (sv.bytes == nullptr) {
      continue;
    }
    auto* fd = reinterpret_cast<TenkunGatewayCanFdSlot*>(sv.bytes);
    fd->id_le = 0u;
    fd->type = TENKUN_TYPE_DATA_STD;
    fd->length = static_cast<uint8_t>(TENKUN_CIH408_JOINT_ID_QUERY_TX_DLC);
    TenkunCih408PackJointIdQueryBroadcast(fd->data);
  }
}

void Tenkun_ResetCanFdIdScanResults(int slave_idx) {
  XJDLRobot::TenkunSlaveStation* st = XJDLRobot::TenKunRti::FindStation(slave_idx);
  if (st != nullptr) {
    st->ResetJointIdScan();
  }
}

void Tenkun_PrintCanFdIdScanResults(int slave_idx) {
  XJDLRobot::TenkunSlaveStation* st = XJDLRobot::TenKunRti::FindStation(slave_idx);
  if (st != nullptr) {
    st->PrintJointIdScanResults();
  }
}

void Rv_Message_Print(int slave, uint8_t ack_status) {
  (void)slave;
  (void)ack_status;
}

void Tenkun_PostInitEnableJoints(void) {}

}
