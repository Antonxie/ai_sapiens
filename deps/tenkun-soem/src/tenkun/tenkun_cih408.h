// tenkun/ 目录：仅 CiH408 CAN FD 关节协议（组包/解包），无 EtherCAT 与网关 PDO。
// 仲裁 1Mbps / 数据 5Mbps；多字节大端（总线侧）。
// 与《CAN协议说明 CiH408》一致；数据装入网关 PDO 的 data[] 后再由网关发到 CAN。
// 网关 PDO 数据域编号：16B 槽 data[0]=D15 … data[15]=D0；8B 槽 data[0]=D7 … data[7]=D0。
// 力位/速度/读状态 0x17（dlc=1）均从 D15 起排布，首字节在 data[0]。
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TENKUN_CIH408_CMD_ENABLE 0x10
#define TENKUN_CIH408_CMD_HYBRID 0x11
#define TENKUN_CIH408_CMD_POSITION 0x12
#define TENKUN_CIH408_CMD_VELOCITY 0x13
#define TENKUN_CIH408_CMD_CURRENT 0x14
#define TENKUN_CIH408_CMD_QUERY 0x17
/* 关节 ID 修改（CiH408）：dlc=3，Byte1=0x82，Byte2=旧关节 ID，Byte3=新关节 ID；勿改为与其他关节重复 ID */
#define TENKUN_CIH408_CMD_SET_JOINT_ID 0x02
#define TENKUN_CIH408_CMD_SET_ZERO 0x03 /* 关节零点标定：dlc=1，当前位置写为零点并存 Flash */
#define TENKUN_CIH408_RSP_SET_ZERO 0x83 /* 应答 dlc=6：Byte2 0x01 成功 / 0x00 失败，Byte3~6 零点位置 float 大端（度） */
#define TENKUN_CIH408_RSP_STATUS 0x80   // 0x17 / 运动控制 → 16B 默认状态帧首字节
#define TENKUN_CIH408_RSP_ENABLE 0x90   // 0x10 启动/停止应答首字节（dlc=2，Byte2=0x01启/0x00停）
// 关节 ID 查询（广播）：CAN ID=0x000，dlc=8，数据域全 0；应答 dlc=2，Byte1=0x80，Byte2=关节 ID
#define TENKUN_CIH408_JOINT_ID_QUERY_TX_DLC 8

void TenkunCih408PackEnable(uint8_t* d2, int start /*1 run 0 stop*/);
// 力位混合：16 字节；seq 为自增计数防丢包
void TenkunCih408PackHybrid(uint8_t* d16, uint16_t kp, uint16_t kd, float pos_rad, float vel_rad_s,
                            int16_t feedforward, uint8_t seq);
void TenkunCih408PackVelocity(uint8_t* d10, float vel_rad_s, float cur_limit_a, uint8_t seq);
void TenkunCih408PackCurrent(uint8_t* d6, float cur_a, uint8_t seq);
/* nbytes = sizeof(slot->data)；清零后在 D15（data[0]）写 0x17。 */
void TenkunCih408PackQuery(uint8_t* data, size_t nbytes);
// 关节 ID 查询发送帧（8 字节全 0，装入网关 PDO data[]）
void TenkunCih408PackJointIdQueryBroadcast(uint8_t* d8);
/* dlc=3：Byte1=0x82，Byte2=旧 ID，Byte3=新 ID（与《CAN协议说明 CiH408》一致） */
void TenkunCih408PackSetJointId(uint8_t* data, size_t nbytes, uint8_t old_joint_id, uint8_t new_joint_id);
/* nbytes = sizeof(slot->data)；清零后在 D15（data[0]）写 0x03，与查询/使能短帧一致（CiH408 §3.4）。 */
void TenkunCih408PackSetZero(uint8_t* data, size_t nbytes);
/* 解析零点标定应答 0x83，dlc>=6；成功时 zero_pos_rad 为标定后零点位置（与 0x80 位置同为弧度） */
void TenkunCih408UnpackSetZero83(const uint8_t* buf, size_t nbytes, int* success_out, float* zero_pos_rad_out);

// 解析默认状态帧 0x80，dlc>=16；电流 A = raw_i16/100，温度 = uint8-50，母线 V = uint8/2
void TenkunCih408UnpackStatus80(const uint8_t* d16, float* pos_out, float* vel_out, float* cur_a_out,
                                float* motor_temp_c_out, float* mos_temp_c_out, float* bus_v_out,
                                uint8_t* mode_nibble_out, uint16_t* error12_out);

#ifdef __cplusplus
}
#endif
