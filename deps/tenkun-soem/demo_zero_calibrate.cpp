// 交互式 CiH408 关节零点标定：从标准输入读取 joints_names 顺序下标，调用 TenkunZeroCalibrateMotorOrdered。
// 后台线程周期性 RunControlAllSlaves()，与 read_states / demo 一致。
//
// Usage:
//   sudo ./tenkun_soem_zero_calibrate --config configs/rti_example.json
//   ./tenkun_soem_zero_calibrate --help

#include "tenkun_rti.hpp"

#include <atomic>
#include <cctype>
#include <cerrno>
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
               "  启动后打印关节列表，输入：zero set <下标>（与 joints_names 顺序一致，例：zero set 1）。\n"
               "  标定过程中会暂停周期性 RunControlAllSlaves，仅下发 0x03 零点指令。\n"
               "  输入 q 或 quit 退出。\n"
               "  --config <RTI.json>：网卡由 TenKunRti 从 JSON 的 interface 读取。\n"
               "  非 JSON 拓扑：将 [network_interface] 写入 TENKUN_EC_IF（本程序在需时自动 setenv）。\n"
               "  Example: sudo %s --config configs/rti_example.json\n"
               "  Options:\n"
               "    --config <file>   RTI JSON 或旧版拓扑 .txt。\n",
               argv0, argv0);
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

bool AsciiCaseEq(const std::string& a, const std::string& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (std::tolower(static_cast<unsigned char>(a[i])) != std::tolower(static_cast<unsigned char>(b[i]))) {
      return false;
    }
  }
  return true;
}

std::vector<std::string> SplitWhitespace(const std::string& s) {
  std::vector<std::string> out;
  size_t i = 0;
  const size_t n = s.size();
  while (i < n) {
    while (i < n && std::isspace(static_cast<unsigned char>(s[i]))) {
      ++i;
    }
    if (i >= n) {
      break;
    }
    size_t j = i;
    while (j < n && !std::isspace(static_cast<unsigned char>(s[j]))) {
      ++j;
    }
    out.emplace_back(s.substr(i, j - i));
    i = j;
  }
  return out;
}

/** 解析 "zero set <index>"（zero/set 不区分大小写），成功则写入 idx_out。 */
bool ParseZeroSetIndex(const std::string& line, size_t* idx_out) {
  if (idx_out == nullptr) {
    return false;
  }
  const std::vector<std::string> tok = SplitWhitespace(line);
  if (tok.size() != 3u) {
    return false;
  }
  if (!AsciiCaseEq(tok[0], "zero") || !AsciiCaseEq(tok[1], "set")) {
    return false;
  }
  errno = 0;
  char* endptr = nullptr;
  const unsigned long v = std::strtoul(tok[2].c_str(), &endptr, 10);
  if (errno == ERANGE || endptr == tok[2].c_str() || *endptr != '\0') {
    return false;
  }
  *idx_out = static_cast<size_t>(v);
  return true;
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
    std::printf("[tenkun_soem_zero_calibrate] RTI config: %s\n", tenkun_topology_cfg.c_str());
  }

  std::atomic<bool> quit{false};
  std::atomic<bool> hold_run_control{false};

  XJDLRobot::TenKunRti master(tenkun_topology_cfg);

  if (!master.IsRunning()) {
    std::fprintf(stderr, "[tenkun_soem_zero_calibrate] EtherCAT 未运行，退出。\n");
    return 1;
  }

  const std::vector<std::string>& names = master.JointsCmdOrder();
  if (names.empty()) {
    std::fprintf(stderr, "[tenkun_soem_zero_calibrate] JointsCmdOrder 为空（检查 joints_names / 配置）\n");
    return 1;
  }

  // std::thread cycle_thread([&master, &quit, &hold_run_control]() {
  //   while (master.IsRunning() && !quit.load()) {
  //     if (!hold_run_control.load()) {
  //       master.RunControlAllSlaves();
  //     }
  //     std::this_thread::sleep_for(std::chrono::milliseconds(2));
  //   }
  // });

  std::printf("[tenkun_soem_zero_calibrate] 关节列表（下标与 SetJointsCmdOrdered 顺序一致）：\n");
  for (size_t i = 0; i < names.size(); ++i) {
    std::printf("  [%zu] %s\n", i, names[i].c_str());
  }
  std::printf("输入：zero set <下标> 进行零点标定（例：zero set 1）；q / quit 退出。\n");

  while (master.IsRunning() && !quit.load()) {
    std::printf("> ");
    std::fflush(stdout);
    std::string line;
    if (!std::getline(std::cin, line)) {
      break;
    }
    TrimInPlace(&line);
    if (line.empty()) {
      continue;
    }
    if (line == "q" || line == "Q" || line == "quit" || line == "QUIT") {
      quit.store(true);
      break;
    }

    size_t idx = 0;
    if (!ParseZeroSetIndex(line, &idx)) {
      std::fprintf(stderr, "无效输入：请使用 \"zero set <下标>\"（例：zero set 1），或 q 退出。\n");
      continue;
    }

    std::printf("[tenkun_soem_zero_calibrate] 标定关节 [%zu] %s ...\n", idx,
                idx < names.size() ? names[idx].c_str() : "?");

    hold_run_control.store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    bool ok = master.TenkunZeroCalibrateMotorOrdered(idx);
    hold_run_control.store(false);
    if (ok) {
      std::printf("[tenkun_soem_zero_calibrate] 关节 [%zu] 零点标定成功。\n", idx);
    } else {
      std::printf("[tenkun_soem_zero_calibrate] 关节 [%zu] 零点标定失败（见 stderr 日志）。\n", idx);
    }
  }

  quit.store(true);
  master.StopRunning();
  // if (cycle_thread.joinable()) {
  //   cycle_thread.join();
  // }

  std::printf("[tenkun_soem_zero_calibrate] done.\n");
  return 0;
}
