#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <time.h>
#include <stdio.h>

#include <event.h>

#define LINK_HOST "127.0.0.1"
#define LINK_PORT 4242
#define DATALINK_PORT 4243
#include "udp_transport.h"
#include "fms_network.h"
#include "subsystems/datalink/downlink.h"

#define PERIODIC_SEC       0
#define PERIODIC_USEC   50000
#define PERIODIC_NSEC   (PERIODIC_USEC * 1000)
#define ONE_SEC_NS 1000000000
static struct event periodic_event;
static struct event datalink_event;
static struct timespec periodic_date;

static struct FmsNetwork *network;

#define PERIODIC_START() {        \
    clock_gettime(CLOCK_MONOTONIC, &periodic_date); \
    struct timeval tv;          \
    evutil_timerclear(&tv);       \
    tv.tv_sec  = PERIODIC_SEC;        \
    tv.tv_usec = PERIODIC_USEC;       \
    event_add(&periodic_event, &tv);      \
  }

#define PERIODIC_RESCHEDULE() {           \
    periodic_date.tv_nsec +=  PERIODIC_NSEC;        \
    if (periodic_date.tv_nsec>= ONE_SEC_NS) {       \
      periodic_date.tv_nsec -= ONE_SEC_NS;        \
      periodic_date.tv_sec++;           \
    }                 \
    struct timespec time_now;           \
    clock_gettime(CLOCK_MONOTONIC, &time_now);        \
    int32_t dt_ns = (int32_t)periodic_date.tv_nsec - (int32_t)time_now.tv_nsec; \
    if (time_now.tv_sec != periodic_date.tv_sec)      \
      dt_ns += ONE_SEC_NS;            \
    struct timeval tv;              \
    evutil_timerclear(&tv);           \
    tv.tv_sec  = PERIODIC_SEC;            \
    tv.tv_usec = dt_ns / 1000;            \
    event_add(&periodic_event, &tv);          \
  }


static void periodic_task(int fd, short event, void *arg)
{

  DOWNLINK_SEND_ALIVE(16, MD5SUM);

  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  DOWNLINK_SEND_FMS_TIME(&ts.tv_sec, &ts.tv_nsec);

  PERIODIC_RESCHEDULE();

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


int main(int argc , char **argv)
{

  /* Set real time priority */
  struct sched_param param;
  param.sched_priority = 90;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    //    TRACE(TRACE_ERROR, "sched_setscheduler failed");
    perror("sched_setscheduler failed");
    exit(EXIT_FAILURE);
  }

  /* Initalize event library */
  event_init();

  /* Add our periodic event */
  evtimer_set(&periodic_event, periodic_task, &periodic_event);
  PERIODIC_START();

  /* Add our datalink event */
  network = network_new(LINK_HOST, LINK_PORT, DATALINK_PORT, FMS_UNICAST);
  event_set(&datalink_event, network->socket_in, EV_READ, on_datalink_event, &datalink_event);
  event_add(&datalink_event, NULL);

  event_dispatch();

  return 0;
}


