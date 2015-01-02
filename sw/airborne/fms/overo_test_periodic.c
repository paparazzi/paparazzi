

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <event.h>

//#define LINK_HOST  "10.31.4.7"
//#define LINK_HOST  "stripe"
#define LINK_HOST     "192.168.1.0"
#define LINK_PORT             4242
#define DATALINK_PORT         4243
#define FMS_NETWORK_BROADCAST TRUE

#include "generated/airframe.h"

#include "fms_periodic.h"
#include "subsystems/datalink/downlink.h"
#include "udp_transport.h"
#include "fms_network.h"
#include "fms_spi_link.h"
#include "fms_autopilot_msg.h"

static void main_periodic(int);
static void main_send_to_stm(void);

static void on_datalink_event(int fd, short event, void *arg);
static struct FmsNetwork *network;

int main(int argc, char **argv)
{

  /* Initalize event library */
  event_init();

  if (fms_periodic_init(main_periodic)) {
    TRACE(TRACE_ERROR, "%s", "failed to start periodic generator\n");
    return -1;
  }

  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link\n");
    return -1;
  }

  /* Add our datalink event */
  network = network_new(LINK_HOST, LINK_PORT, DATALINK_PORT, FMS_NETWORK_BROADCAST);
  struct event datalink_event;
  event_set(&datalink_event, network->socket_in, EV_READ, on_datalink_event, &datalink_event);
  event_add(&datalink_event, NULL);

  event_dispatch();

  return 0;
}


static void main_periodic(int my_sig_num)
{

  DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);

  main_send_to_stm();

  UdpTransportPeriodic();

}


static void main_send_to_stm(void)
{

  struct OVERO_LINK_MSG_UP   msg_in;
  struct OVERO_LINK_MSG_DOWN msg_out;
  spi_link_send(&msg_out, sizeof(union AutopilotMessage), &msg_in);

  //  printf("spi telemetry got %d\n", msg_in.up.tw_len);
  //  for (int i=0; i<msg_in.up.tw_len; i++)
  //    printf("%02x ",  msg_in.up.data[i]);
  //  printf("\n");
}


static void on_datalink_event(int fd, short event, void *arg)
{
  char buf[255];
  int bytes_read;
  bytes_read = read(fd, buf, sizeof(buf) - 1);
  if (bytes_read == -1) {
    perror("read");
    return;
  } else if (bytes_read == 0) {
    fprintf(stderr, "Connection closed\n");
    return;
  }
  printf("on_datalink_event, read %d\n", bytes_read);

  struct event *ev = arg;
  event_add(ev, NULL);
}

