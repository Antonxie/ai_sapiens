#include "tenkun_cih408.h"

#include <arpa/inet.h>
#include <string.h>

const float RAD_TO_DEG = 57.29578f;
const float DEG_TO_RAD = 0.017453292f;
const float RPM_TO_RAD_S = 0.1047198f;
const float RAD_S_TO_RPM = 9.5492966f;

static void pack_f32_be(uint8_t* d, float f) {
  uint32_t u;
  memcpy(&u, &f, sizeof(u));
  u = htonl(u);
  memcpy(d, &u, 4);
}

static float unpack_f32_be(const uint8_t* d) {
  uint32_t u;
  memcpy(&u, d, 4);
  u = ntohl(u);
  float f;
  memcpy(&f, &u, sizeof(f));
  return f;
}

void TenkunCih408PackEnable(uint8_t* d2, int start) {
  d2[0] = TENKUN_CIH408_CMD_ENABLE;
  d2[1] = start ? 0x01 : 0x00;
}

/* 16B 网关域：d16[0]=D15 … d16[15]=D0；与 CiH408 Byte1..16 从先到后一致。 */
void TenkunCih408PackHybrid(uint8_t* d16, uint16_t kp, uint16_t kd, float pos_rad, float vel_rad_s,
                            int16_t feedforward, uint8_t seq) {
  d16[0] = TENKUN_CIH408_CMD_HYBRID;
  d16[1] = (uint8_t)((kp >> 8) & 0xff);
  d16[2] = (uint8_t)(kp & 0xff);
  d16[3] = (uint8_t)((kd >> 8) & 0xff);
  d16[4] = (uint8_t)(kd & 0xff);
  pack_f32_be(d16 + 5, pos_rad * RAD_TO_DEG);
  pack_f32_be(d16 + 9, vel_rad_s * RAD_S_TO_RPM);
  d16[13] = (uint8_t)((feedforward >> 8) & 0xff);
  d16[14] = (uint8_t)(feedforward & 0xff);
  d16[15] = seq;
}

void TenkunCih408PackVelocity(uint8_t* d10, float vel_rad_s, float cur_limit_a, uint8_t seq) {
  memset(d10, 0, 16);
  d10[0] = TENKUN_CIH408_CMD_VELOCITY;
  d10[5] = 0x3F;
  d10[15] = 1;
}

void TenkunCih408PackCurrent(uint8_t* d6, float cur_a, uint8_t seq) {
  d6[0] = TENKUN_CIH408_CMD_CURRENT;
  pack_f32_be(d6 + 1, cur_a);
  d6[5] = seq;
}

void TenkunCih408PackQuery(uint8_t* data, size_t nbytes) {
  if (data == NULL || nbytes == 0u) {
    return;
  }
  memset(data, 0, nbytes);
  /* dlc=1：0x17 写在数据域首字节 D15（data[0]），与使能等短帧一致。 */
  data[0] = TENKUN_CIH408_CMD_QUERY;
}

void TenkunCih408PackJointIdQueryBroadcast(uint8_t* d8) { memset(d8, 0, (size_t)TENKUN_CIH408_JOINT_ID_QUERY_TX_DLC); }

void TenkunCih408PackSetJointId(uint8_t* data, size_t nbytes, uint8_t old_joint_id, uint8_t new_joint_id) {
  if (data == NULL || nbytes < 3u) {
    return;
  }
  memset(data, 0, nbytes);
  /* CiH408：dlc=3，D15 起 Byte1=0x82，Byte2=旧关节 ID，Byte3=新关节 ID */
  data[0] = TENKUN_CIH408_CMD_SET_JOINT_ID;
  data[1] = old_joint_id;
  data[2] = new_joint_id;
}

void TenkunCih408PackSetZero(uint8_t* data, size_t nbytes) {
  if (data == NULL || nbytes == 0u) {
    return;
  }
  memset(data, 0, nbytes);
  data[0] = TENKUN_CIH408_CMD_SET_ZERO;
}

void TenkunCih408UnpackSetZero83(const uint8_t* buf, size_t nbytes, int* success_out, float* zero_pos_rad_out) {
  if (buf == NULL || nbytes < 6u) {
    return;
  }
  if (buf[0] != TENKUN_CIH408_RSP_SET_ZERO) {
    return;
  }
  if (success_out) {
    *success_out = (buf[1] == 0x01u) ? 1 : 0;
  }
  if (zero_pos_rad_out) {
    *zero_pos_rad_out = unpack_f32_be(buf + 2) * DEG_TO_RAD;
  }
  printf("== TENKUN_CIH408_RSP_SET_ZERO %d\n", buf[0]);
}

void TenkunCih408UnpackStatus80(const uint8_t* d16, float* pos_out, float* vel_out, float* cur_a_out,
                                float* motor_temp_c_out, float* mos_temp_c_out, float* bus_v_out,
                                uint8_t* mode_nibble_out, uint16_t* error12_out) {
  if (d16[0] != TENKUN_CIH408_RSP_STATUS) {
    return;
  }
  uint16_t mode_err = ((uint16_t)d16[1] << 8) | d16[2];
  if (mode_nibble_out) {
    *mode_nibble_out = (uint8_t)((mode_err >> 12) & 0x0fu);
  }
  if (error12_out) {
    *error12_out = mode_err & 0x0fffu;
  }
  if (pos_out) {
    *pos_out = unpack_f32_be(d16 + 3) * DEG_TO_RAD;
  }
  if (vel_out) {
    *vel_out = unpack_f32_be(d16 + 7) * RPM_TO_RAD_S;
  }
  if (cur_a_out) {
    int16_t cur = (int16_t)(((uint16_t)d16[11] << 8) | d16[12]);
    *cur_a_out = (float)cur / 100.0f;
  }
  if (motor_temp_c_out) {
    *motor_temp_c_out = (float)d16[13] - 50.0f;
  }
  if (mos_temp_c_out) {
    *mos_temp_c_out = (float)d16[14] - 50.0f;
  }
  if (bus_v_out) {
    *bus_v_out = (float)d16[15] / 2.0f;
  }
}
