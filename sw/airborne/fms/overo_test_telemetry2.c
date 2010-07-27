
#include <inttypes.h>

#include <stdio.h>

#include <event.h>
#include <evutil.h>

#include "downlink_transport.h"
#include "messages2.h"
#include "udp_transport2.h"
#include "fms_network.h"

#define GCS_HOST "10.10.13.32"
#define GCS_PORT 4242
#define DATALINK_PORT 4243

#define TIMEOUT_DT_SEC  0
//#define TIMEOUT_DT_USEC 500000
#define TIMEOUT_DT_USEC 50000


#define ADD_TIMEOUT() {				\
    struct timeval tv;				\
    evutil_timerclear(&tv);			\
    tv.tv_sec  = TIMEOUT_DT_SEC;		\
    tv.tv_usec = TIMEOUT_DT_USEC;		\
    event_add(&timeout, &tv);			\
  }

static void timeout_cb(int fd, short event, void *arg);

static struct event timeout;
static struct FmsNetwork* network;
static struct DownlinkTrasport *udp_transport;

void timeout_cb(int fd, short event, void *arg) {

  //  printf("in timeout_cb\n");

  DOWNLINK_SEND_ALIVE(udp_transport, 16, MD5SUM);

  float  foof = 3.14159265358979323846;
  double food = 3.14159265358979323846;
  DOWNLINK_SEND_TEST_FORMAT(udp_transport, &food, &foof);

  ADD_TIMEOUT();

}


int main(int argc, char** argv) {

  network = network_new(GCS_HOST, GCS_PORT, DATALINK_PORT, FALSE);
  udp_transport = udp_transport_new(network);

  /* Initalize the event library */
  event_init();

  /* Add a timeout event */
  evtimer_set(&timeout, timeout_cb, &timeout);

  ADD_TIMEOUT();

  event_dispatch();

  return 0;
}
