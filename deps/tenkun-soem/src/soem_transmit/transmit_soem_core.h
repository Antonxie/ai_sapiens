// SOEM 核心与协议层之间的内部衔接。本头不引用 queue_tenkun.h，避免与过程映像形成环依赖。
// 各 TU 须先包含 queue_tenkun.h（已完整定义 EtherCAT_Msg），再包含本头。
#pragma once

#include <memory>
#include <boost/lockfree/spsc_queue.hpp>
#include <thread>
#include <atomic>

#include "tenkun_gateway_pdo.h"

#define SLAVE_NUMBER 4

int EtherCAT_Init(const char* ifname);
void startRun();
void stopRun();
bool getRunState(void);

void SoemSendToQueue(int slave_id, const std::shared_ptr<EtherCAT_Msg>& msg);
void SoemProtocol_EtherCAT_Data_Get(void);

using boost::lockfree::capacity;
using boost::lockfree::spsc_queue;

using EtherCAT_Msg_ptr = std::shared_ptr<EtherCAT_Msg>;

extern spsc_queue<EtherCAT_Msg_ptr, capacity<10>> tx_messages[SLAVE_NUMBER];
extern std::atomic<bool> running;
extern std::thread run_thread;

extern bool is_config[SLAVE_NUMBER];
extern long cmd_send_num;
extern long read_num;

const char* SoemTransmitLogTag(void);


namespace soem_tx {
constexpr int kQueryResendEveryIterations = 200;
constexpr int kQueryPollSleepUs = 5000;
constexpr int kMainCycleHz = 1000;
constexpr double kStatsLogIntervalSeconds = 1.0;
}  // namespace soem_tx
