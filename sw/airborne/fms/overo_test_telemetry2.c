
#include <inttypes.h>

#include <stdio.h>

#include <event.h>
#include <evutil.h>

#include <string.h>
#include <unistd.h>

#include "downlink_transport.h"
#include "messages2.h"
#include "udp_transport2.h"
#include "dl_protocol.h"
#include "fms_network.h"

#define GCS_HOST "10.31.4.7"
#define GCS_PORT 4242
#define DATALINK_PORT 4243

#define TIMEOUT_DT_SEC  0
//#define TIMEOUT_DT_USEC 500000
#define TIMEOUT_DT_USEC 50000

#define DL_MSG_SIZE 128

#define ADD_TIMEOUT() {       \
    struct timeval tv;        \
    evutil_timerclear(&tv);     \
    tv.tv_sec  = TIMEOUT_DT_SEC;    \
    tv.tv_usec = TIMEOUT_DT_USEC;   \
    event_add(&timeout, &tv);     \
  }

static void timeout_cb(int fd, short event, void *arg);

static struct event timeout;
static struct event read_event;
static struct FmsNetwork *network;
static struct DownlinkTransport *udp_transport;

void timeout_cb(int fd, short event, void *arg)
{

  //  printf("in timeout_cb\n");

  DOWNLINK_SEND_ALIVE(udp_transport, 16, MD5SUM);

  float  foof = 3.14159265358979323846;
  double food = 3.14159265358979323846;
  DOWNLINK_SEND_TEST_FORMAT(udp_transport, &food, &foof);

  if (udp_transport->Periodic) {
    udp_transport->Periodic(udp_transport->impl);
  }

  ADD_TIMEOUT();

}

static inline int checked_read(int fd, char *buf, size_t len)
{
  int bytes = read(fd, buf, len);

  if (bytes == 0) {
    fprintf(stderr, "Connection closed\n");
  } else if (bytes == -1) {
    perror("read");
  }

  return bytes;
}

bool_t my_dl_msg_available;
uint8_t my_dl_buffer[DL_MSG_SIZE]  __attribute__((aligned));

#define IdOfMsg(x) (x[1])

static void dl_handle_msg(struct DownlinkTransport *tp)
{
  uint8_t msg_id = IdOfMsg(my_dl_buffer);
  switch (msg_id) {

    case  DL_PING: {
      DOWNLINK_SEND_PONG(tp);
    }
    break;

    case DL_SETTING : {
      uint8_t i = DL_SETTING_index(my_dl_buffer);
      float var = DL_SETTING_value(my_dl_buffer);
      // DlSetting(i, var);
      DOWNLINK_SEND_DL_VALUE(tp, &i, &var);
    }
    break;
  }

}

static void on_datalink_event(int fd, short event __attribute__((unused)), void *arg)
{
  char buf[255];
  int bytes_read;
  bytes_read = checked_read(fd, buf, sizeof(buf) - 1);
  struct DownlinkTransport *tp = (struct DownlinkTransport *) arg;
  struct udp_transport *udp_impl = tp->impl;

  int i = 0;
  while (i < bytes_read) {
    parse_udp_dl(udp_impl, buf[i]);
    i++;
    if (udp_impl->udp_dl_msg_received) {
      memcpy(my_dl_buffer, udp_impl->udp_dl_payload, udp_impl->udp_dl_payload_len);
      dl_handle_msg(tp);
      udp_impl->udp_dl_msg_received = FALSE;
    }
  }
}

int main(int argc, char **argv)
{

  network = network_new(GCS_HOST, GCS_PORT, DATALINK_PORT, FALSE);
  udp_transport = udp_transport_new(network);


  /* Initalize the event library */
  event_init();

  event_set(&read_event, network->socket_in, EV_READ | EV_PERSIST, on_datalink_event, udp_transport);
  event_add(&read_event, NULL);

  /* Add a timeout event */
  evtimer_set(&timeout, timeout_cb, &timeout);

  ADD_TIMEOUT();

  event_dispatch();

  return 0;
}
