#include "udp_transport.h"

char upd_transport_buf[UDP_TRANSPORT_BUF_SIZE];
uint16_t udpt_buf_idx;
uint8_t udpt_ck_a, udpt_ck_b;
