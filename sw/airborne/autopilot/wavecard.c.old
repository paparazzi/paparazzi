#include <inttypes.h>
#include <stdlib.h>

#include "wavecard.h"

typedef uint8_t bool_t;

uint8_t  wc_payload[WC_MAX_PAYLOAD];

static uint8_t wc_status;

const uint16_t poly = 0x8408;
uint16_t crc;
uint8_t wc_length, payload_idx;

#define UNINIT 0
#define GOT_SYNC1 1
#define GOT_STX 2
#define GOT_LENGTH 3
#define GOT_PAYLOAD 4
#define GOT_CRC1 5
#define GOT_CRC2 6


bool_t waiting_ack, wc_msg_received;

uint8_t wc_protocol_error, wc_ovrn, wc_error;

void parse_payload() {
  switch (wc_payload[0]) {
  case WC_ACK:
    if (waiting_ack)
      waiting_ack = FALSE;
    else
      wc_protocol_error++;
    break;
  case WC_NAK:
  case WC_ERROR:
    wc_protocol_error++;
    break;
  default:
    WcSendAck();
  }
  wc_msg_received = FALSE;
}

inline void parse_wc( uint8_t c ) {
  //  printf("s=%d\n", wc_status);
  switch (wc_status) {
  case UNINIT:
    if (c == WC_SYNC)
      wc_status++;
    break;
  case GOT_SYNC1:
    if (c != WC_STX)
      goto error;
    crc = 0;
    wc_status++;
    break;
  case GOT_STX:
    if (wc_msg_received) {
      wc_ovrn++;
      goto error;
    }
    wc_length = c;
    update_crc(c);
    wc_status++;
    payload_idx = 0;
    break;
  case GOT_LENGTH:
    wc_payload[payload_idx] = c;
    update_crc(c);
    payload_idx++;
    if (payload_idx == wc_length-3)
      wc_status++;
    break;
  case GOT_PAYLOAD:
    if (c != (crc & 0xff))
      goto error;
    wc_status++;
    break;
  case GOT_CRC1:
    if (c != (crc >> 8))
      goto error;
    wc_status++;
    break;
  case GOT_CRC2:
    if (c != WC_ETX)
      goto error;
    wc_msg_received = TRUE;
    goto restart;
    break;
  }
  return;
 error:
  wc_error++;
 restart:
  wc_status = UNINIT;
  return;
}

 
/***     
#define WcPut1CtlByte(_byte) { \
  tx_buf[tx_head] = _byte; \
  tx_head++; \
  if (tx_head >= TX_BUF_SIZE) tx_head = 0; \
}
***/


  
