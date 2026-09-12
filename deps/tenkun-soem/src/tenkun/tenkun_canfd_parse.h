// Tenkun CiH408 CAN FD：从单槽「数据域 + DLC」解析出结构化信息（无 EtherCAT / 网关 / 从站状态依赖）。
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  TENKUN_RX_PARSE_NONE = 0,
  TENKUN_RX_PARSE_JOINT_ID_RSP,
  TENKUN_RX_PARSE_ENABLE_RSP,
  TENKUN_RX_PARSE_SET_JOINT_ID_RSP,
  TENKUN_RX_PARSE_SET_ZERO_RSP,
  TENKUN_RX_PARSE_STATUS_16,
  TENKUN_RX_PARSE_RAW,
} TenkunCanFdRxKind;

typedef struct {
  TenkunCanFdRxKind kind;
  uint32_t can_id_11; /* 标准帧：低 11 位有效，与网关 id_le 一致 */

  /* TENKUN_RX_PARSE_STATUS_16：与 TenkunCih408UnpackStatus80 一致 */
  float pos_rad;
  float vel_rad_s;
  float cur_a;
  float motor_temp_c;
  float mos_temp_c;
  float bus_v;
  uint8_t mode_nibble;
  uint16_t error12;

  /* 短应答 */
  uint8_t joint_id;
  uint8_t enable_on; /* ENABLE 应答：1=运行 0=停止 */
  uint8_t set_joint_old_id;
  uint8_t set_joint_new_id;
  uint8_t zero_set_ok;   /* 零点标定应答：1 成功 0 失败 */
  float zero_pos_rad;   /* 应答中的零点位置（弧度），与状态 0x80 位置单位一致 */
  uint64_t recv_cnt;
} TenkunCanFdRxInfo;

/* 解析网关子槽 payload：data 为槽内 data[]，dlc 为槽 length 字节数。
 * 返回 1 表示 kind 已识别并写入 out；0 表示空或无法归类（kind=RAW 时仍可能带 can_id_11）。 */
int TenkunCanFdParseSlotPayload(uint32_t can_id_le, const uint8_t* data, uint8_t dlc, TenkunCanFdRxInfo* out);

void PrintTenkunRxFeedback(const char* name, const TenkunCanFdRxInfo m);

#ifdef __cplusplus
}
#endif
