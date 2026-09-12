// SOEM 周期与初始化（Tenkun 过程映像 + 每从站命令队列）。

#include "transmit_soem_core.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <unistd.h>

extern "C" {
#include "ethercat.h"
#include "time.h"
}

std::atomic<bool> running{false};
std::thread run_thread;
spsc_queue<EtherCAT_Msg_ptr, capacity<10>> tx_messages[SLAVE_NUMBER];
char IOmap[4096];
OSAL_THREAD_HANDLE checkThread;
int expectedWKC = 0;
boolean needlf = FALSE;
volatile int wkc = 0;
boolean inOP = FALSE;
uint8 currentgroup = 0;

bool is_config[SLAVE_NUMBER]{};
long cmd_send_num = 0;
long read_num = 0;

const char* SoemTransmitLogTag(void) { return "[Tenkun SOEM]"; }

void SoemSendToQueue(int slave_id, const std::shared_ptr<EtherCAT_Msg>& msg) {
  if (slave_id < 0 || slave_id >= SLAVE_NUMBER) {
    std::printf("%s SendToQueue: invalid slave_id %d\n", SoemTransmitLogTag(), slave_id);
    return;
  }
  if (tx_messages[slave_id].write_available()) {
    tx_messages[slave_id].push(msg);
  }
}

namespace {

constexpr int kEthercatErrPeriodIterations = 100;
constexpr int kEthercatErrMaxPerPeriod = 20;
constexpr int kEthercatMonitorTimeoutUs = 500;
constexpr int kEthercatCheckThreadSleepUs = 50000;

constexpr int kEthercatInitMaxAttempts = 10;
constexpr int kEthercatInitRetryDelayUs = 1000000;

int err_count = 0;
int err_iteration_count = 0;
int wkc_err_count = 0;
int wkc_err_iteration_count = 0;

void DegradedHandler() {
  std::printf("[EtherCAT Error] Logging error...\n");
  const time_t current_time = time(nullptr);
  char* time_str = ctime(&current_time);
  if (time_str != nullptr) {
    std::printf("ESTOP. EtherCAT became degraded at %s", time_str);
  }
  std::printf("[EtherCAT Error] Stopping RT process.\n");
}

}  // namespace



namespace {

int RunEthercat(const char* ifname) {
  int i = 0;
  int oloop = 0;
  int iloop = 0;
  int chk = 0;
  needlf = FALSE;
  inOP = FALSE;

  if (!ec_init(ifname)) {
    std::printf("[EtherCAT Error] No socket connection on %s (need root/cap_net_raw?)\n", ifname);
    return 0;
  }

  std::printf("[EtherCAT Init] Initialization on device %s succeeded.\n", ifname);

  if (ec_config_init(FALSE) <= 0) {
    std::printf("[EtherCAT Error] No slaves found!\n");
    return 0;
  }

  std::printf("[EtherCAT Init] %d slaves found and configured.\n", ec_slavecount);
  if (ec_slavecount < SLAVE_NUMBER) {
    std::printf("[EtherCAT Init] Warning: compile-time SLAVE_NUMBER=%d, found %d slaves.\n", SLAVE_NUMBER,
                ec_slavecount);
  }

  for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++) {
    ec_slave[slave_idx + 1].CoEdetails &= ~ECT_COEDET_SDOCA;
  }

  ec_config_map(&IOmap);
  ec_configdc();

  std::printf("[EtherCAT Init] Mapped slaves.\n");
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * SLAVE_NUMBER);

  for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++) {
    std::printf("[SLAVE %d]\n", slave_idx);
    std::printf("  IN  %d bytes, %d bits\n", ec_slave[slave_idx].Ibytes, ec_slave[slave_idx].Ibits);
    std::printf("  OUT %d bytes, %d bits\n", ec_slave[slave_idx].Obytes, ec_slave[slave_idx].Obits);
    std::printf("\n");
  }

  oloop = ec_slave[0].Obytes;
  if ((oloop == 0) && (ec_slave[0].Obits > 0)) {
    oloop = 1;
  }
  if (oloop > 8) {
    oloop = 8;
  }
  iloop = ec_slave[0].Ibytes;
  if ((iloop == 0) && (ec_slave[0].Ibits > 0)) {
    iloop = 1;
  }
  if (iloop > 8) {
    iloop = 8;
  }

  std::printf("[EtherCAT Init] segments : %d : %d %d %d %d\n", ec_group[0].nsegments,
              ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2],
              ec_group[0].IOsegment[3]);

  std::printf("[EtherCAT Init] Requesting operational state for all slaves...\n");
  expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  std::printf("[EtherCAT Init] Calculated workcounter %d\n", expectedWKC);
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  ec_writestate(0);
  chk = 40;
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
    std::printf("[EtherCAT Error] Not all slaves reached operational state.\n");
    ec_readstate();
    for (i = 1; i <= ec_slavecount; i++) {
      if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
        std::printf("[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state,
                    ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
      }
    }
    return 0;
  }

  std::printf("[EtherCAT Init] Operational state reached for all slaves.\n");
  inOP = TRUE;
  return 1;
}

static OSAL_THREAD_FUNC EcatCheck(void* ptr) {
  (void)ptr;
  int slave = 0;
  while (1) {
    if (err_iteration_count > kEthercatErrPeriodIterations) {
      err_iteration_count = 0;
      err_count = 0;
    }

    if (err_count > kEthercatErrMaxPerPeriod) {
      std::printf("[EtherCAT Error] EtherCAT connection degraded.\n");
      DegradedHandler();
      break;
    }
    err_iteration_count++;

    if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
      if (needlf) {
        needlf = FALSE;
        std::printf("\n");
      }
      ec_group[currentgroup].docheckstate = FALSE;
      ec_readstate();
      for (slave = 1; slave <= ec_slavecount; slave++) {
        if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
          ec_group[currentgroup].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            std::printf("[EtherCAT Error] Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
            err_count++;
          } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
            std::printf("[EtherCAT Error] Slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
            err_count++;
          } else if (ec_slave[slave].state > 0) {
            if (ec_reconfig_slave(slave, kEthercatMonitorTimeoutUs)) {
              ec_slave[slave].islost = FALSE;
              std::printf("[EtherCAT Status] Slave %d reconfigured\n", slave);
            }
          } else if (!ec_slave[slave].islost) {
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (!ec_slave[slave].state) {
              ec_slave[slave].islost = TRUE;
              std::printf("[EtherCAT Error] Slave %d lost\n", slave);
              err_count++;
            }
          }
        }
        if (ec_slave[slave].islost) {
          if (!ec_slave[slave].state) {
            if (ec_recover_slave(slave, kEthercatMonitorTimeoutUs)) {
              ec_slave[slave].islost = FALSE;
              std::printf("[EtherCAT Status] Slave %d recovered\n", slave);
            }
          } else {
            ec_slave[slave].islost = FALSE;
            std::printf("[EtherCAT Status] Slave %d found\n", slave);
          }
        }
      }
      if (!ec_group[currentgroup].docheckstate) {
        std::printf("[EtherCAT Status] All slaves resumed OPERATIONAL.\n");
      }
    }
    osal_usleep(kEthercatCheckThreadSleepUs);
  }
}

}  // namespace

int EtherCAT_Init(const char* ifname) {
  std::printf("[EtherCAT] Initializing EtherCAT\n");
  osal_thread_create((void*)&checkThread, 128000, (void*)&EcatCheck, nullptr);

  int rc = 0;
  int attempt = 0;
  for (attempt = 1; attempt <= kEthercatInitMaxAttempts; attempt++) {
    std::printf("[EtherCAT] Attempting to start EtherCAT, try %d of %d.\n", attempt, kEthercatInitMaxAttempts);
    rc = RunEthercat(ifname);
    if (rc != 0) {
      break;
    }
    osal_usleep(kEthercatInitRetryDelayUs);
  }

  if (rc != 0) {
    std::printf("[EtherCAT] EtherCAT successfully initialized on attempt %d\n", attempt);
    return 1;
  }
  std::printf("[EtherCAT Error] Failed to initialize EtherCAT after %d attempts.\n", kEthercatInitMaxAttempts);
  return 0;
}

void EtherCAT_Command_Set() {
  for (int slave = 0; slave < ec_slavecount; ++slave) {
    EtherCAT_Msg_ptr msg;
    if (tx_messages[slave].read_available()) {
      tx_messages[slave].pop(msg);
      is_config[slave] = true;
      auto* slave_dest = reinterpret_cast<EtherCAT_Msg*>(ec_slave[slave + 1].outputs);
      if (slave_dest != nullptr) {
        *slave_dest = *msg;
        cmd_send_num++;
        // printf("slave: %d, msg.id_can: %d, msg.channel: %d, msg.len: %d\n", slave, msg->pdo.fd[0].id_le, msg->pdo.fd[0].data[0], msg->pdo.fd[0].length);
      } else {
        std::printf("[EtherCAT Error] Slave %d output buffer is full\n", slave);
      }
    } else {
      EtherCAT_Msg_ptr msg(new EtherCAT_Msg);
      std::memset(msg->raw, 0, sizeof(msg->raw));
      auto* slave_dest = reinterpret_cast<EtherCAT_Msg*>(ec_slave[slave + 1].outputs);
      if (slave_dest != nullptr) {
        *slave_dest = *msg;
        // cmd_send_num++;
        // printf("slave: %d, msg.id_can: %d, msg.channel: %d, msg.len: %d\n", slave, msg->pdo.fd[0].id_le, msg->pdo.fd[0].data[0], msg->pdo.fd[0].length);
      }
    }
  }
}


void EtherCAT_Run() {
  static uint64_t cycle_count = 0;
  if (wkc_err_iteration_count > kEthercatErrPeriodIterations) {
    wkc_err_count = 0;
    wkc_err_iteration_count = 0;
  }
  if (wkc_err_count > kEthercatErrMaxPerPeriod) {
    std::printf("[EtherCAT Error] Error count too high!\n");
    DegradedHandler();
  }
  cycle_count++;
  EtherCAT_Command_Set();
  ec_send_processdata();
  wkc = ec_receive_processdata(EC_TIMEOUTRET);
  SoemProtocol_EtherCAT_Data_Get();
  if (wkc < expectedWKC) {
    std::printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
    wkc_err_count++;
  } else {
    needlf = TRUE;
  }
  wkc_err_iteration_count++;
}

namespace {

using SteadyTp = std::chrono::steady_clock::time_point;
inline SteadyTp SteadyNow() { return std::chrono::steady_clock::now(); }
inline double SecondsSince(SteadyTp t) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}
inline bool TimedOut(SteadyTp start, double timeout_sec) { return SecondsSince(start) > timeout_sec; }

}  // namespace

static void RunImpl() {
  std::printf("%s Cyclic thread started (%d Hz).\n", SoemTransmitLogTag(), soem_tx::kMainCycleHz);
  SteadyTp stats_start = SteadyNow();
  long run_time = 0;
  const double cycle_seconds = 1.0 / static_cast<double>(soem_tx::kMainCycleHz);

  while (running.load()) {
    SteadyTp cycle_start = SteadyNow();
    EtherCAT_Run();

    if (!TimedOut(cycle_start, cycle_seconds)) {
      const double sleep_time = cycle_seconds - SecondsSince(cycle_start);
      if (sleep_time > 0.0) {
        std::this_thread::sleep_for(
            std::chrono::microseconds(static_cast<long long>(sleep_time * 1e6)));
      }
    } else {
      // std::printf("%s cycle overrun: %.6f s > %.6f s\n", SoemTransmitLogTag(), SecondsSince(cycle_start),
                  // cycle_seconds);
    }

    run_time++;
    if (TimedOut(stats_start, soem_tx::kStatsLogIntervalSeconds)) {
      stats_start = SteadyNow();
      cmd_send_num = 0;
      run_time = 0;
    }
  }
}

void startRun() {
  running = true;
  run_thread = std::thread(RunImpl);
}

void stopRun() {
  running = false;
  if (run_thread.joinable()) {
    run_thread.join();
  }
}

bool getRunState(void) { return running.load(); }
