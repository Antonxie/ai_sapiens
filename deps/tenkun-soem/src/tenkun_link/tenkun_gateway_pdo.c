#include "tenkun_gateway_pdo.h"

#include <string.h>

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(TenkunGatewayCanFdSlot) == 22, "fd slot");
_Static_assert(sizeof(TenkunGatewayPdo) == TENKUN_GATEWAY_PDO_BYTES, "gateway pdo");
#endif

void TenkunGatewayGetSlot(uint8_t* pdo320, unsigned channel, TenkunGatewaySlotView* out) {
  out->bytes = NULL;
  out->channel = channel;
  if (pdo320 == NULL || channel < 1u || channel > TENKUN_GATEWAY_FD_SLOTS) {
    return;
  }
  TenkunGatewayPdo* p = (TenkunGatewayPdo*)pdo320;
  out->bytes = (uint8_t*)&p->fd[channel - 1u];
}

void TenkunGatewayClearSlot(TenkunGatewaySlotView* slot) {
  if (slot == NULL || slot->bytes == NULL) {
    return;
  }
  memset(slot->bytes, 0, sizeof(TenkunGatewayCanFdSlot));
}
