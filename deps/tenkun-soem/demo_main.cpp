// Minimal executable: bring up SOEM on a NIC and run the cyclic master until Enter.
//
// Usage:
//   sudo ./tenkun_soem_demo --config configs/rti_example.json   # 网卡由 TenKunRti 从 JSON 的 interface 读取
//   sudo ./tenkun_soem_demo enp3s0 --config legacy.txt          # 非 JSON 配置：位置参数写入 TENKUN_EC_IF
//   TENKUN_EC_IF=enp3s0 sudo -E ./tenkun_soem_demo
//   ./tenkun_soem_demo --help

#include "tenkun_rti.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr float kTwoPiF = 6.283185307179586476925286766559f;
constexpr double kPi = 3.14159265358979323846;
constexpr double kPhase1DurationSec = 5.0;

/** 余弦缓动：t∈[0,1] 时从 1 光滑降到 0（端点导数为 0）。用于 p(t)=p0*blend(t) 在 2s 内回到 0。 */
float CosineBlend01(double t) {
  if (t <= 0.0) {
    return 1.0f;
  }
  if (t >= 1.0) {
    return 0.0f;
  }
  return static_cast<float>(0.5 * (1.0 + std::cos(kPi * t)));
}


void PrintUsage(const char* argv0) {
  std::fprintf(stderr,
               "Usage: %s [options] [network_interface]\n"
               "  --config <RTI.json>：网卡由 TenKunRti 从 JSON 的 interface 读取（勿再单独传网卡给 RTI）。\n"
               "  非 JSON 拓扑或仅用环境变量：将 [network_interface] 写入 TENKUN_EC_IF（本程序在需时自动 setenv）。\n"
               "  Example: sudo %s --config configs/rti_example.json\n"
               "  Example: sudo %s enp3s0 --config legacy_topology.txt\n"
               "  Options:\n"
               "    --config <file>         RTI JSON（interface / slaves / joints）或旧版拓扑 .txt。\n"
               "    --scan-can-fd [slave]   Broadcast CiH408 joint ID query on first CAN FD logical slot only,\n"
               "                            then print responses (default slave=0).\n",
               argv0, argv0);
}

/** 与 TenKunRti 一致：跳过空白后首字符是否为 '{'，用于判断是否把位置参数当作 TENKUN_EC_IF。 */
bool FileLooksLikeRtiJson(const std::string& path) {
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

}  // namespace

int main(int argc, char** argv) {
  std::vector<const char*> positional;
  int scan_slave = 0;
  std::string tenkun_topology_cfg;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
      PrintUsage(argv[0]);
      return 0;
    }
    if (std::strcmp(argv[i], "--config") == 0) {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "Error: --config needs a file path.\n");
        PrintUsage(argv[0]);
        return 1;
      }
      tenkun_topology_cfg = argv[++i];
      continue;
    }
    positional.push_back(argv[i]);
  }

  const bool json_rti_cfg = !tenkun_topology_cfg.empty() && FileLooksLikeRtiJson(tenkun_topology_cfg);
  if (!json_rti_cfg && !positional.empty() && positional[0][0] != '\0') {
    if (::setenv("TENKUN_EC_IF", positional[0], 1) != 0) {
      std::fprintf(stderr, "Error: setenv(TENKUN_EC_IF) failed.\n");
      return 1;
    }
  }

  if (!tenkun_topology_cfg.empty()) {
    std::printf("[tenkun_soem_demo] RTI config: %s\n", tenkun_topology_cfg.c_str());
  }

  {
    XJDLRobot::TenKunRti master(tenkun_topology_cfg);
    
    std::atomic<bool> stop_mapped_queries{false};
    std::thread mapped_query_thread([&]() {
      using clock = std::chrono::steady_clock;

      const size_t n_joints = master.JointsCmdOrder().size();
      if (n_joints == 0) {
        std::fprintf(stderr, "[tenkun_soem_demo] JointsCmdOrder 为空（检查 joints_names / 配置）\n");
        exit(1);
      }

      master.SetMotorEnabledAll(true);

      for (int w = 0; w < 50 && master.IsRunning() && !stop_mapped_queries.load(); ++w) {
        master.RunControlAllSlaves();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }

      std::vector<float> p0(n_joints, 0.0f);
      std::vector<float> lenght(n_joints, 0.0f);
      {
        const std::vector<TenkunCanFdRxInfo> snap = master.GetJointsStatusOrdered();
        for (size_t i = 0; i < n_joints && i < snap.size(); ++i) {
          p0[i] = snap[i].pos_rad;
          lenght[i] = master.DefaultPosRad()[i] - p0[i];
        }
        std::printf("[tenkun_soem_demo] 阶段1：已读各关节当前 pos_rad（作为回零起点），共 %zu 路\n", n_joints);
      }

      const clock::time_point t1_start = clock::now();
      while (master.IsRunning() && !stop_mapped_queries.load()) {
        const double elapsed =
            std::chrono::duration<double>(clock::now() - t1_start).count();
        if (elapsed >= kPhase1DurationSec) {
          break;
        }
        const double u = elapsed / kPhase1DurationSec;
        const float blend = CosineBlend01(u);

        std::vector<MotorCmdType> cmds(n_joints);
        for (size_t i = 0; i < n_joints; ++i) {
          cmds[i].kp = 20.0f;
          cmds[i].kd = 2.0f;
          cmds[i].pos = p0[i] +  lenght[i] * (1 - blend);
          cmds[i].spd = 0.0f;
          cmds[i].tor = 0.0f;
        }
        if (!master.SetJointsCmdOrdered(cmds)) {
          std::fprintf(stderr, "[tenkun_soem_demo] SetJointsCmdOrdered 失败（阶段1）\n");
        }
        master.RunControlAllSlaves();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      float phase2_s = 0.0f;
      while (master.IsRunning() && !stop_mapped_queries.load()) {
        phase2_s += 0.001f;
        const float sinp = std::sin(phase2_s * kTwoPiF) * 0.2;
        std::vector<MotorCmdType> cmds(n_joints);
        for (size_t i = 0; i < n_joints; ++i) {
          cmds[i].kp = 20.0f;
          cmds[i].kd = 2.0f;
          cmds[i].pos = sinp + master.DefaultPosRad()[i];
          cmds[i].spd = 0.0f;
          cmds[i].tor = 0.0f;
        }
        if (!master.SetJointsCmdOrdered(cmds)) {
          std::fprintf(stderr, "[tenkun_soem_demo] SetJointsCmdOrdered 失败（阶段2）\n");
        }
        master.RunControlAllSlaves();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    });
    std::printf("[tenkun_soem_demo] cyclic thread running; press Enter to exit.\n");
    
    while (true) {
      const std::vector<TenkunCanFdRxInfo> states = master.GetJointsStatusOrdered();
      const std::vector<std::string>& names = master.JointsCmdOrder();
      for (size_t i = 0; i < states.size(); ++i) {
        const char* tag = (i < names.size()) ? names[i].c_str() : "joint";
        PrintTenkunRxFeedback(tag, states[i]);
      }
      if (states.empty()) {
        std::printf("[tenkun_soem_demo] GetJointsStatusOrdered 为空（检查 joints_cmd_order / 配置）\n");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    
    stop_mapped_queries = true;
    mapped_query_thread.join();
  }

  std::printf("[tenkun_soem_demo] done.\n");
  return 0;
}
