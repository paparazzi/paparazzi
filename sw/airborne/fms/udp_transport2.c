#include <stdlib.h>
#include "udp_transport2.h"
#include "fms_network.h"
#include "downlink_transport.h"


static void put_1byte(struct udp_transport *udp, const uint8_t x)
{
  udp->updt_tx_buf[udp->udpt_tx_buf_idx] = x;
  udp->udpt_tx_buf_idx++;
}

static void put_uint8_t(struct udp_transport *udp, const uint8_t byte)
{
  udp->udpt_ck_a += byte;
  udp->udpt_ck_b += udp->udpt_ck_a;
  put_1byte(udp, byte);
}

static void put_named_uint8_t(struct udp_transport *udp, char *name __attribute__((unused)), const uint8_t byte)
{
  put_uint8_t(udp, byte);
}

static void put_bytes(void *impl, enum DownlinkDataType data_type __attribute__((unused)), uint8_t len, const void *buf)
{
  struct udp_transport *udp = (struct udp_transport *) impl;
  const uint8_t *bytes = (const uint8_t *) buf;
  for (int i = 0; i < len; i++) {
    put_uint8_t(udp, bytes[i]);
  }
}

static void header(struct udp_transport *udp, uint8_t payload_len)
{
  uint32_t msg_timestamp = MSG_TIMESTAMP;
  /*udpt_tx_buf_idx = 0;*/
  put_1byte(udp, STX_UDP_TX);
  uint8_t msg_len = payload_len + PPRZ_PROTOCOL_OVERHEAD;
  udp->udpt_ck_a = udp->udpt_ck_b = 0;
  put_uint8_t(udp, msg_len);
  put_bytes(udp, DL_TYPE_UINT32, 4, &msg_timestamp);
}

static void start_message(void *impl, char *name, uint8_t msg_id, uint8_t payload_len)
{
  struct udp_transport *udp = (struct udp_transport *) impl;
  header(udp, 2 + payload_len);
  put_uint8_t(udp, AC_ID);
  put_named_uint8_t(udp, name, msg_id);
}

static void end_message(void *impl)
{
  struct udp_transport *udp = (struct udp_transport *) impl;
  put_1byte(udp, udp->udpt_ck_a);
  put_1byte(udp, udp->udpt_ck_b);
  if (udp->udpt_tx_buf_idx > UDPT_TX_BUF_WATERMARK) {
    network_write(udp->network, udp->updt_tx_buf, udp->udpt_tx_buf_idx);
    udp->udpt_tx_buf_idx = 0;
  }
}

static void overrun(void *impl __attribute__((unused)))
{

}

static void count_bytes(void *udp __attribute__((unused)), uint8_t bytes __attribute__((unused)))
{

}

static int check_free_space(void *udp __attribute__((unused)), uint8_t bytes __attribute__((unused)))
{
  return TRUE;
}

static uint8_t size_of(void *udp __attribute__((unused)), uint8_t len)
{
  return len + 2;
}

static void periodic(void *impl)
{
  struct udp_transport *udp = (struct udp_transport *) impl;
  if (udp->udpt_tx_buf_idx > 0) {
    network_write(udp->network, udp->updt_tx_buf, udp->udpt_tx_buf_idx);
    udp->udpt_tx_buf_idx = 0;
  }
}

struct DownlinkTransport *udp_transport_new(struct FmsNetwork *network)
{
  struct DownlinkTransport *tp = calloc(1, sizeof(struct DownlinkTransport));
  struct udp_transport *udp = calloc(1, sizeof(struct udp_transport));

  udp->udp_dl_status = UNINIT;
  udp->network = network;
  tp->impl = udp;

  tp->StartMessage = start_message;
  tp->EndMessage = end_message;
  tp->PutBytes = put_bytes;

  tp->Overrun = overrun;
  tp->CountBytes = count_bytes;
  tp->SizeOf = size_of;
  tp->CheckFreeSpace = check_free_space;
  tp->Periodic = periodic;

  return tp;
}
