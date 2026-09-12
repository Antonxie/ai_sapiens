#include "tenkun_canfd_parse.h"

#include "tenkun_cih408.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

int 
TenkunCanFdParseSlotPayload(uint32_t can_id_le, const uint8_t* data, uint8_t dlc, TenkunCanFdRxInfo* out) {
  if (out == NULL) {
    return 0;
  }
  memset(out, 0, sizeof(*out));
  out->can_id_11 = can_id_le & 0x7ffu;

  if (data == NULL || dlc == 0u) {
    out->kind = TENKUN_RX_PARSE_RAW;
    return 0;
  }

  /* dlc=2, 0x80 + 关节 ID */
  if (dlc == 2u && data[0] == TENKUN_CIH408_RSP_STATUS) {
    out->kind = TENKUN_RX_PARSE_JOINT_ID_RSP;
    out->joint_id = data[1];
    return 1;
  }

  if (dlc == 2u && data[0] == TENKUN_CIH408_RSP_ENABLE) {
    out->kind = TENKUN_RX_PARSE_ENABLE_RSP;
    out->enable_on = (data[1] == 0x01u) ? 1u : 0u;
    return 1;
  }

  if (dlc == 3u && data[0] == TENKUN_CIH408_CMD_SET_JOINT_ID) {
    out->kind = TENKUN_RX_PARSE_SET_JOINT_ID_RSP;
    out->set_joint_old_id = data[1];
    out->set_joint_new_id = data[2];
    out->joint_id = data[2];
    return 1;
  }

  if (dlc == 6u && data[0] == TENKUN_CIH408_RSP_SET_ZERO) {
    printf("TENKUN_CIH408_RSP_SET_ZERO %d\n", data[0]);
    out->kind = TENKUN_RX_PARSE_SET_ZERO_RSP;
    int ok = 0;
    float z = 0.f;
    TenkunCih408UnpackSetZero83(data, (size_t)dlc, &ok, &z);
    out->zero_set_ok = (uint8_t)ok;
    out->zero_pos_rad = z;
    printf("TENKUN_CIH408_RSP_SET_ZERO ok %d z %f\n", ok, z);
    return 1;
  }

  if (dlc >= 16u && data[0] == TENKUN_CIH408_RSP_STATUS) {
    out->kind = TENKUN_RX_PARSE_STATUS_16;
    TenkunCih408UnpackStatus80(data, &out->pos_rad, &out->vel_rad_s, &out->cur_a, &out->motor_temp_c,
                               &out->mos_temp_c, &out->bus_v, &out->mode_nibble, &out->error12);
    return 1;
  }

  out->kind = TENKUN_RX_PARSE_RAW;
  return 0;
}

void PrintTenkunRxFeedback(const char* name, const TenkunCanFdRxInfo m) {
  char spos[20];
  char svel[20];
  char sia[16];
  char stm[16];
  char sts[16];
  char sub[16];
  char srx[24];
  (void)snprintf(spos, sizeof(spos), "%10.4f", (double)m.pos_rad);
  (void)snprintf(svel, sizeof(svel), "%10.4f", (double)m.vel_rad_s);
  (void)snprintf(sia, sizeof(sia), "%8.3f", (double)m.cur_a);
  (void)snprintf(stm, sizeof(stm), "%4.0f", (double)m.motor_temp_c);
  (void)snprintf(sts, sizeof(sts), "%4.0f", (double)m.mos_temp_c);
  (void)snprintf(sub, sizeof(sub), "%5.1f", (double)m.bus_v);
  (void)snprintf(srx, sizeof(srx), "%12" PRIu64, (uint64_t)m.recv_cnt);

  printf(
      "%-26s | can=%3u zero_set_ok=%3u en=%u | pos=%s vel=%s I=%s | Tmot=%s Tmos=%s Ub=%s | md=%u err=0x%03x rx=%s\n",
      name != NULL ? name : "", (unsigned)m.can_id_11, (unsigned)m.zero_set_ok, (unsigned)m.enable_on, spos, svel, sia,
      stm, sts, sub, (unsigned)m.mode_nibble, (unsigned)(m.error12 & 0xfffu), srx);
}