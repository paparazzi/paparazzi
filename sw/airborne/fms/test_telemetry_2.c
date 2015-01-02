
#include <inttypes.h>

#include <stdio.h>

#include <event.h>
#include <evutil.h>


#include "subsystems/datalink/downlink.h"
#include "udp_transport.h"
#include "fms_network.h"

#define GCS_HOST "10.31.4.7"
#define GCS_PORT 4242

#define TIMEOUT_DT_SEC  0
#define TIMEOUT_DT_USEC 500000

static void timeout_cb(int fd, short event, void *arg);

static struct event timeout;
static struct FmsNetwork *network;

void timeout_cb(int fd, short event, void *arg)
{

  printf("in timeout_cb\n");

  DOWNLINK_SEND_ALIVE(16, MD5SUM);

  struct timeval tv;
  evutil_timerclear(&tv);
  tv.tv_sec  = TIMEOUT_DT_SEC;
  tv.tv_usec = TIMEOUT_DT_USEC;
  event_add(&timeout, &tv);

}


int main(int argc, char **argv)
{

  printf("hello world\n");

  network = network_new(GCS_HOST, GCS_PORT);

  /* Initalize the event library */
  event_init();

  /* Add a timeout event */
  evtimer_set(&timeout, timeout_cb, &timeout);

  struct timeval tv;
  evutil_timerclear(&tv);
  tv.tv_sec  = TIMEOUT_DT_SEC;
  tv.tv_usec = TIMEOUT_DT_USEC;
  event_add(&timeout, &tv);

  event_dispatch();

  return 0;
}
