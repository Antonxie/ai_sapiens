// 仅周期性读取并打印关节反馈（不使能电机、不下发运动/阻抗指令）。
// EtherCAT 主循环由库内线程负责；此处另起轻量线程调用 RunControlAllSlaves() 以与 demo 一致地刷新网关逻辑。
//
// Usage:
//   sudo ./tenkun_soem_read_states --config configs/rti_example.json
//   sudo ./tenkun_soem_read_states enp3s0 --config legacy_topology.txt
//   TENKUN_EC_IF=enp3s0 sudo -E ./tenkun_soem_read_states
//   ./tenkun_soem_read_states --help

#include "tenkun_rti.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

void PrintUsage(const char* argv0) {
  std::fprintf(stderr,
               "Usage: %s [options] [network_interface]\n"
               "  --config <RTI.json>：网卡由 TenKunRti 从 JSON 的 interface 读取。\n"
               "  非 JSON 拓扑：将 [network_interface] 写入 TENKUN_EC_IF（本程序在需时自动 setenv）。\n"
               "  Example: sudo %s --config configs/rti_example.json\n"
               "  Example: sudo %s enp3s0 --config legacy_topology.txt\n"
               "  Options:\n"
               "    --config <file>   RTI JSON 或旧版拓扑 .txt。\n"
               "  按 Enter 退出。\n",
               argv0, argv0, argv0);
}

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
    std::printf("[tenkun_soem_read_states] RTI config: %s\n", tenkun_topology_cfg.c_str());
  }

  std::atomic<bool> quit{false};

  XJDLRobot::TenKunRti master(tenkun_topology_cfg);

  if (!master.IsRunning()) {
    std::fprintf(stderr, "[tenkun_soem_read_states] EtherCAT 未运行，退出。\n");
    return 1;
  }

  if (master.JointsCmdOrder().empty()) {
    std::fprintf(stderr, "[tenkun_soem_read_states] JointsCmdOrder 为空（检查 joints_names / 配置）\n");
    return 1;
  }

  std::thread stdin_thread([&quit]() {
    std::string line;
    std::getline(std::cin, line);
    quit.store(true);
  });

  std::thread cycle_thread([&master, &quit]() {
    while (master.IsRunning() && !quit.load()) {
      master.RunControlAllSlaves();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  });

  std::printf("[tenkun_soem_read_states] 仅读状态；按 Enter 退出。\n");

  while (master.IsRunning() && !quit.load()) {
    const std::vector<TenkunCanFdRxInfo> states = master.GetJointsStatusOrdered();
    const std::vector<std::string>& names = master.JointsCmdOrder();
    std::cout <<"================================================" << std::endl;
    for (size_t i = 0; i < states.size(); ++i) {
      const char* tag = (i < names.size()) ? names[i].c_str() : "joint";
      PrintTenkunRxFeedback(tag, states[i]);
    }
    if (states.empty()) {
      std::printf("[tenkun_soem_read_states] GetJointsStatusOrdered 为空（检查配置）\n");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  quit.store(true);
  master.StopRunning();
  if (cycle_thread.joinable()) {
    cycle_thread.join();
  }
  if (stdin_thread.joinable()) {
    stdin_thread.join();
  }

  std::printf("[tenkun_soem_read_states] done.\n");
  return 0;
}
