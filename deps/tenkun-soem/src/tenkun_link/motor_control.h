// EtherCAT 衔接层：查询类型（PDO 见 tenkun_gateway_pdo.h）。CiH408 组包/解析见 tenkun/。
// 每槽反馈缓存为 TenkunCanFdRxInfo（与 tenkun_canfd_parse.h 一致），不再使用臃肿的 OD 镜像结构。
#pragma once

#include "tenkun_canfd_parse.h"
#include "tenkun_gateway_pdo.h"
#include <stdint.h>
#include <string.h>

#define comm_ack 0u
#define comm_auto 1u

#define TENKUN_MOTORS_PER_SLAVE 12

#ifdef __cplusplus
extern "C" {
#endif

void Tenkun_FillBroadcastIdQueryAllCanFdSlots(EtherCAT_Msg* tx);
void Tenkun_FillBroadcastJointIdQueryAllCanFdChannels(EtherCAT_Msg* tx);

void Tenkun_ResetCanFdIdScanResults(int slave_idx);
void Tenkun_PrintCanFdIdScanResults(int slave_idx);

void Rv_Message_Print(int slave, uint8_t ack_status);

void Tenkun_PostInitEnableJoints(void);

#ifdef __cplusplus
}
#endif
