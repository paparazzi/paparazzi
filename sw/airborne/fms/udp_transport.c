#include "udp_transport.h"

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
