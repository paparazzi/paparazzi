/* TODO PLACEHOLDER CODE! */
#include "cc2500_compat.h"

#define RC_CHANNEL_COUNT_FRSKY_X 16

static rx_spi_received_e frSkyXHandlePacket(uint8_t * const packet, uint8_t * const protocolState) {
  (void) packet;
  (void) protocolState;
  return RX_SPI_RECEIVED_NONE;
}

static void frSkyXSetRcData(uint16_t *rcData, const uint8_t *payload) {
  (void) rcData;
  (void) payload;
  return;
}

static void frSkyXInit(const rx_spi_protocol_e spiProtocol) {
  (void) spiProtocol;
  return;
}
