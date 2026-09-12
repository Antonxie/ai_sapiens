
#pragma once

#include <cstdint>

typedef struct {
  int slave;
  uint16_t chanel_id;  // 网关逻辑通道 1..12（与 PDO CAN FD 槽一致）
  uint16_t can_tx_id;

  float kp;
  float kd;
  float pos;
  float spd;
  float tor;
 
} MotorCmdType;
