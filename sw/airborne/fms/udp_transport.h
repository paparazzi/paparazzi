#ifndef UDP_TRANSPORT_H
#define UDP_TRANSPORT_H

#include "fms_network.h"

#define UDP_TRANSPORT_BUF_SIZE 1024
extern char upd_transport_buf[UDP_TRANSPORT_BUF_SIZE];

extern uint8_t udpt_ck_a, udpt_ck_b;
extern uint16_t udpt_buf_idx;

#define UdpTransportCheckFreeSpace(_x) ((_x) < UDP_TRANSPORT_BUF_SIZE)

#define UdpTransportSizeOf(_payload) (_payload+4)

#define UdpTransportHeader(payload_len) {		\
    udpt_buf_idx = 0;					\
    UdpTransportPut1Byte(STX);				\
    uint8_t msg_len = UdpTransportSizeOf(payload_len);	\
    UdpTransportPut1Byte(msg_len);			\
    udpt_ck_a = msg_len; udpt_ck_b = msg_len;		\
}

#define UdpTransportTrailer() {						\
    UdpTransportPut1Byte(udpt_ck_a);					\
    UdpTransportPut1Byte(udpt_ck_b);					\
    network_write(network, upd_transport_buf, udpt_buf_idx);		\
  }

#define UdpTransportPut1Byte(_x) {upd_transport_buf[udpt_buf_idx] = (_x); udpt_buf_idx++;}

#define UdpTransportPutUint8(_byte) {	  \
    udpt_ck_a += _byte;			  \
    udpt_ck_b += udpt_ck_a;			  \
    UdpTransportPut1Byte(_byte);	  \
  }

#define UdpTransportPutNamedUint8(_name, _byte) UdpTransportPutUint8(_byte)

#define UdpTransportPut1ByteByAddr(_byte) {	\
    uint8_t _x = *(_byte);			\
    UdpTransportPutUint8(_x);			\
  }

/* base types */
#define UdpTransportPutInt8ByAddr(_x)  UdpTransportPut1ByteByAddr(_x)
#define UdpTransportPutUint8ByAddr(_x) UdpTransportPut1ByteByAddr((const uint8_t*)_x)

#define UdpTransportPutArray(_put, _n, _x) { \
    uint8_t _i;				     \
    UdpTransportPutUint8(_n);		     \
    for(_i = 0; _i < _n; _i++) {	     \
      _put(&_x[_i]);			     \
    }					     \
}

#define UdpTransportPutUint8Array(_n, _x) UdpTransportPutArray(UdpTransportPutUint8ByAddr, _n, _x)

#endif /* UDP_TRANSPORT_H */

