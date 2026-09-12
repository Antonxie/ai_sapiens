// Tenkun 自研 EtherCAT<->CAN FD 网关 PDO（Rx/Tx 同结构）。
// 软件固定 12 个 CAN FD 逻辑通道 1..12；实际使用几路由硬件与用户 map 决定。
// 简化模型：仅 CAN FD 子槽（22B），无 CAN2；原 CAN2 带宽并入 reserved_tail。
// 总长 320 字节；CAN ID 小端 4 字节（D0..D3）。
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TENKUN_GATEWAY_PDO_BYTES 320

/** 逻辑 CAN FD 槽数量（与 PDO 内 fd[] 一致）。 */
#define TENKUN_GATEWAY_FD_SLOTS 12

/** reserved_tail 字节数：320 - 8 - 12*22 = 48 */
#define TENKUN_GATEWAY_RESERVED_TAIL_BYTES 48

// type：bit0=1 远程帧；bit1=1 扩展帧。
#define TENKUN_TYPE_DATA_STD 0u
#define TENKUN_TYPE_REMOTE_STD 1u
#define TENKUN_TYPE_DATA_EXT 2u

#pragma pack(push, 1)

typedef struct {
  uint32_t id_le;
  uint8_t type;
  uint8_t length;  // 有效数据长度 0~16
  /* data[0]=D15 … data[15]=D0 */
  uint8_t data[16];
} TenkunGatewayCanFdSlot;

/* 8 + 12*22 + 48 = 320 */
typedef struct {
  uint8_t reserved_head[8];
  TenkunGatewayCanFdSlot fd[TENKUN_GATEWAY_FD_SLOTS];
  uint8_t reserved_tail[TENKUN_GATEWAY_RESERVED_TAIL_BYTES];
} TenkunGatewayPdo;

#pragma pack(pop)

/** Tenkun 网关过程映像：与 TenkunGatewayPdo 同 320B，也可用 raw[] 按字节访问。 */
typedef union EtherCAT_Msg {
  TenkunGatewayPdo pdo;
  uint8_t raw[TENKUN_GATEWAY_PDO_BYTES];
} EtherCAT_Msg;

// 逻辑通道 1..12；无槽时 bytes==NULL。
typedef struct {
  uint8_t* bytes;
  unsigned channel;
} TenkunGatewaySlotView;

void TenkunGatewayGetSlot(uint8_t* pdo320, unsigned channel, TenkunGatewaySlotView* out);

void TenkunGatewayClearSlot(TenkunGatewaySlotView* slot);

#ifdef __cplusplus
}
#endif
