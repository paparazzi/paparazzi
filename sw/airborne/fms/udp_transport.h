#ifndef UDP_TRANSPORT_H
#define UDP_TRANSPORT_H

#include "fms_network.h"
#include "fms_debug.h"
#include "std.h"

#ifdef UDP_TRANSPORT_TIMESTAMP
#define STX_UDP_TX  0x98
#define STX_UDP_RX  0x99
#define PPRZ_PROTOCOL_OVERHEAD 8
#define UdpTransportPutTimestamp(x) UdpTransportPutUint32ByAddr(x)
#else
#define STX_UDP_TX  0x99
#define STX_UDP_RX  0x99
#define PPRZ_PROTOCOL_OVERHEAD 4
#define UdpTransportPutTimestamp(x) {}
#endif

#ifndef MSG_TIMESTAMP
#define MSG_TIMESTAMP 0
#endif

#define UDPT_TX_BUF_LEN 1496
extern char updt_tx_buf[UDPT_TX_BUF_LEN];
extern uint16_t udpt_tx_buf_idx;
extern uint8_t udpt_ck_a, udpt_ck_b;
#define  UDPT_TX_BUF_WATERMARK 1024

#define UdpTransportSizeOf(_payload) (_payload + PPRZ_PROTOCOL_OVERHEAD)

#define UdpTransportPutSTX() UdpTransportPut1Byte(STX_UDP_TX)

#define UdpTransportPeriodic() {        \
    if (udpt_tx_buf_idx) {          \
      int len;                \
      len = network_write(network, updt_tx_buf, udpt_tx_buf_idx); \
      downlink.nb_bytes += udpt_tx_buf_idx;     \
      downlink.nb_msgs++;         \
      if (len != udpt_tx_buf_idx)       \
        downlink.nb_ovrn++;         \
      udpt_tx_buf_idx = 0;          \
    }               \
  }

#define UdpTransportCheckFreeSpace(_x) (TRUE)


#define UdpTransportHeader(payload_len) {   \
    uint32_t msg_timestamp = MSG_TIMESTAMP;   \
    /*udpt_tx_buf_idx = 0;*/        \
    UdpTransportPutSTX();   \
    uint8_t msg_len = UdpTransportSizeOf(payload_len);  \
    udpt_ck_a = udpt_ck_b = 0;        \
    UdpTransportPutUint8(msg_len);      \
    UdpTransportPutTimestamp(&msg_timestamp);   \
  }


#define UdpTransportTrailer() {           \
    UdpTransportPut1Byte(udpt_ck_a);          \
    UdpTransportPut1Byte(udpt_ck_b);          \
    if (udpt_tx_buf_idx > UDPT_TX_BUF_WATERMARK) {      \
      network_write(network, updt_tx_buf, udpt_tx_buf_idx);   \
      udpt_tx_buf_idx = 0;            \
    }                 \
  }

#define UdpTransportPut1Byte(_x) {      \
    updt_tx_buf[udpt_tx_buf_idx] = (_x);    \
    udpt_tx_buf_idx++;          \
  }

#define UdpTransportPutUint8(_byte) {   \
    udpt_ck_a += _byte;       \
    udpt_ck_b += udpt_ck_a;     \
    UdpTransportPut1Byte(_byte);    \
  }

#define UdpTransportPutNamedUint8(_name, _byte) UdpTransportPutUint8(_byte)

#define UdpTransportPut1ByteByAddr(_byte) { \
    uint8_t _x = *(_byte);      \
    UdpTransportPutUint8(_x);     \
  }

#define UdpTransportPut2ByteByAddr(_byte) {     \
    UdpTransportPut1ByteByAddr(_byte);        \
    UdpTransportPut1ByteByAddr((const uint8_t*)_byte+1);  \
  }

#define UdpTransportPut4ByteByAddr(_byte) {     \
    UdpTransportPut2ByteByAddr(_byte);        \
    UdpTransportPut2ByteByAddr((const uint8_t*)_byte+2);  \
  }

#define UdpTransportPut8ByteByAddr(_byte) {     \
    UdpTransportPut4ByteByAddr(_byte);        \
    UdpTransportPut4ByteByAddr((const uint8_t*)_byte+4);  \
  }


/* base types */
#define UdpTransportPutInt8ByAddr(_x)   UdpTransportPut1ByteByAddr(_x)
#define UdpTransportPutUint8ByAddr(_x)  UdpTransportPut1ByteByAddr((const uint8_t*)_x)
#define UdpTransportPutInt16ByAddr(_x)  UdpTransportPut2ByteByAddr((const uint8_t*)_x)
#define UdpTransportPutUint16ByAddr(_x) UdpTransportPut2ByteByAddr((const uint8_t*)_x)
#define UdpTransportPutInt32ByAddr(_x)  UdpTransportPut4ByteByAddr((const uint8_t*)_x)
#define UdpTransportPutUint32ByAddr(_x) UdpTransportPut4ByteByAddr((const uint8_t*)_x)
#define UdpTransportPutFloatByAddr(_x)  UdpTransportPut4ByteByAddr((const uint8_t*)_x)
#define UdpTransportPutDoubleByAddr(_x) UdpTransportPut8ByteByAddr((const uint8_t*)_x)

#define UdpTransportPutArray(_put, _n, _x) { \
    uint8_t _i;            \
    UdpTransportPutUint8(_n);        \
    for(_i = 0; _i < _n; _i++) {       \
      _put(&_x[_i]);           \
    }              \
  }

#define UdpTransportPutInt16Array(_n, _x) UdpTransportPutArray(UdpTransportPutInt16ByAddr, _n, _x)

#define UdpTransportPutUint8Array(_n, _x) UdpTransportPutArray(UdpTransportPutUint8ByAddr, _n, _x)

#define UdpTransportPutUint16Array(_n, _x) UdpTransportPutArray(UdpTransportPutUint16ByAddr, _n, _x)


/*
 * Parsing of uplink msg
 */
#define UNINIT 0
#define GOT_STX 1
#define GOT_LENGTH 2
#define GOT_PAYLOAD 3
#define GOT_CRC1 4

#define UDP_DL_PAYLOAD_LEN 256
extern uint8_t udp_dl_payload[UDP_DL_PAYLOAD_LEN];
extern volatile uint8_t udp_dl_payload_len;
extern volatile bool_t udp_dl_msg_received;
extern uint8_t udp_dl_ovrn, udp_dl_nb_err;

static inline void parse_udp_dl(uint8_t c)
{
  static uint8_t udp_dl_status = UNINIT;
  static uint8_t _ck_a, _ck_b, payload_idx;

  switch (udp_dl_status) {
    case UNINIT:
      if (c == STX_UDP_RX) {
        udp_dl_status++;
      }
      break;
    case GOT_STX:
      if (udp_dl_msg_received) {
        udp_dl_ovrn++;
        goto error;
      }
      udp_dl_payload_len = c - 4; /* Counting STX, LENGTH and CRC1 and CRC2 */
      _ck_a = _ck_b = c;
      udp_dl_status++;
      payload_idx = 0;
      break;
    case GOT_LENGTH:
      udp_dl_payload[payload_idx] = c;
      _ck_a += c; _ck_b += _ck_a;
      payload_idx++;
      if (payload_idx == udp_dl_payload_len) {
        udp_dl_status++;
      }
      break;
    case GOT_PAYLOAD:
      if (c != _ck_a) {
        goto error;
      }
      udp_dl_status++;
      break;
    case GOT_CRC1:
      if (c != _ck_b) {
        goto error;
      }
      udp_dl_msg_received = TRUE;
      goto restart;
  }
  return;
error:
  udp_dl_nb_err++;
restart:
  udp_dl_status = UNINIT;
  return;
}

#endif /* UDP_TRANSPORT_H */

