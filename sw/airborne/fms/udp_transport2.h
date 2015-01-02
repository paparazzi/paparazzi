#ifndef UDP_TRANSPORT2_H
#define UDP_TRANSPORT2_H

#include "fms_network.h"
#include "fms_debug.h"
#include "std.h"
#include "generated/airframe.h"

#define STX_UDP_TX  0x98
#define STX_UDP_RX  0x99
#define PPRZ_PROTOCOL_OVERHEAD 8

#ifndef MSG_TIMESTAMP
#define MSG_TIMESTAMP 0
#endif

struct DownlinkTransport *udp_transport_new(struct FmsNetwork *network);

#define UDPT_TX_BUF_LEN 1496
#define UDPT_TX_BUF_WATERMARK 1024
#define UDP_DL_PAYLOAD_LEN 256

struct udp_transport {
  /*
   * Downlink
   */
  char updt_tx_buf[UDPT_TX_BUF_LEN];
  uint16_t udpt_tx_buf_idx;
  uint8_t udpt_ck_a, udpt_ck_b;

  /*
   * Uplink
   */
  uint8_t udp_dl_payload[UDP_DL_PAYLOAD_LEN];
  volatile uint8_t udp_dl_payload_len;
  volatile bool_t udp_dl_msg_received;
  uint8_t udp_dl_ovrn, udp_dl_nb_err;
  uint8_t udp_dl_status;
  uint8_t _ck_a, _ck_b, payload_idx;

  struct FmsNetwork *network;
};


/*
 * Parsing of uplink msg
 */
#define UNINIT 0
#define GOT_STX 1
#define GOT_LENGTH 2
#define GOT_PAYLOAD 3
#define GOT_CRC1 4

static inline void parse_udp_dl(struct udp_transport *tp, uint8_t c)
{

  switch (tp->udp_dl_status) {
    case UNINIT:
      if (c == STX_UDP_RX) {
        tp->udp_dl_status++;
      }
      break;
    case GOT_STX:
      if (tp->udp_dl_msg_received) {
        tp->udp_dl_ovrn++;
        goto error;
      }
      tp->udp_dl_payload_len = c - 4; /* Counting STX, LENGTH and CRC1 and CRC2 */
      tp->_ck_a = tp->_ck_b = c;
      tp->udp_dl_status++;
      tp->payload_idx = 0;
      break;
    case GOT_LENGTH:
      tp->udp_dl_payload[tp->payload_idx] = c;
      tp->_ck_a += c; tp->_ck_b += tp->_ck_a;
      tp->payload_idx++;
      if (tp->payload_idx == tp->udp_dl_payload_len) {
        tp->udp_dl_status++;
      }
      break;
    case GOT_PAYLOAD:
      if (c != tp->_ck_a) {
        goto error;
      }
      tp->udp_dl_status++;
      break;
    case GOT_CRC1:
      if (c != tp->_ck_b) {
        goto error;
      }
      tp->udp_dl_msg_received = TRUE;
      goto restart;
  }
  return;
error:
  tp->udp_dl_nb_err++;
restart:
  tp->udp_dl_status = UNINIT;
  return;
}

#endif /* UDP_TRANSPORT2_H */

