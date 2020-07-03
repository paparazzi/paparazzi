/* TODO PLACEHOLDER CODE! */
#include "cc2500_compat.h"

#define RC_CHANNEL_COUNT_FRSKY_D 8

static rx_spi_received_e frSkyDHandlePacket(uint8_t * const packet, uint8_t * const protocolState) {
  (void) packet;
  (void) protocolState;
  return RX_SPI_RECEIVED_NONE;
}

static void frSkyDSetRcData(uint16_t *rcData, const uint8_t *payload) {
  (void) rcData;
  (void) payload;
  return;
}

static void frSkyDInit(void) {
  return;
}
