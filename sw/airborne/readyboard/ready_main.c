/*
 * $Id$
 *  
 * Copyright (C) 2009  Antoine Drouin, Allen H. Ibara
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

#include <stdlib.h>
#include <unistd.h>
#include <sched.h>

#include <event.h>

#include "rdyb_xsens.h"
#include "rdyb_ahrs.h"
#include "rdyb_debug.h"
#include "rdyb_gpio.h"

#define GCS_HOST "127.0.0.1"
#define GCS_PORT 4242
#include "udp_transport.h"
#include "fms_network.h"
#include "downlink.h"

#define PERIODIC_SEC       0
#define PERIODIC_USEC   2000
static struct event periodic_event;
static struct event xsens1_event;
#define SCHEDULE_PERIODIC() {			\
    struct timeval tv;				\
    evutil_timerclear(&tv);			\
    tv.tv_sec  = PERIODIC_SEC;			\
    tv.tv_usec = PERIODIC_USEC;			\
    event_add(&periodic_event, &tv);		\
}

static struct RdybXsens* xsens1;
//static struct RdybXsens* xsens2;
static struct RdybAhrs*   ahrs;
static struct FmsNetwork* network;

static void periodic_task(int fd, short event, void *arg);
static void on_xsens_serial_data(int fd, short event, void *arg);
static void on_xens_message_received(void);

#define PERIODIC_PRESCALE 10
static void periodic_task(int fd, short event, void *arg) {
  static uint32_t cnt = 0;
  cnt++;
  if (cnt >= PERIODIC_PRESCALE) cnt = 0;
    
  switch (cnt) {
  case 0:
    DOWNLINK_SEND_ALIVE(16, MD5SUM);
    break;
  }
  ahrs_propagate(ahrs, &xsens1->gyro);
  SCHEDULE_PERIODIC();

}

static void on_xsens_serial_data(int fd, short event, void *arg) {

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
  xsens_parse(xsens1, bytes_read, buf, on_xens_message_received);
  struct event *ev = arg;
  event_add(ev, NULL);

}

static void on_xens_message_received(void) {

#if 0
  gpio_toggle();
  TELEMETRY_SEND_ACCEL_RAW(xs->accel_raw);
  TELEMETRY_SEND_GYRO_RAW(xs->gyro_raw);
  TELEMETRY_SEND_MAG_RAW(xs->mag_raw);

  TELEMETRY_SEND_ACCEL(xs->accel);
  TELEMETRY_SEND_GYRO(xs->gyro);
  TELEMETRY_SEND_MAG(xs->mag);

  ahrs_propagate(ahrs, &xs->gyro);
  ahrs_update(ahrs, &xs->accel, &xs->mag);
  
  TELEMETRY_SEND_AHRS_EULER(ahrs->ltp2imu_euler);
  TELEMETRY_SEND_AHRS_MEASUREMENT_EULER(ahrs->measurement);
#endif

}

int main(int argc , char** argv) {

  /* Set real time priority */
  struct sched_param param;
  param.sched_priority = 90;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    //    TRACE(TRACE_ERROR, "sched_setscheduler failed");
    perror("sched_setscheduler failed");
    return(EXIT_FAILURE);
  }

  /* Initalize event library */
  event_init();

  /* Add our periodic event */
  evtimer_set(&periodic_event, periodic_task, &periodic_event);
  SCHEDULE_PERIODIC();

  /* instanciate xsens */
  if (!(xsens1 = xsens_new())) {
    TRACE(TRACE_ERROR, "opening xsens1 failed %d\n", 1);
    return(EXIT_FAILURE);
  }

  /* Add xsens as event source */
  event_set(&xsens1_event, xsens1->sp->fd, EV_READ, on_xsens_serial_data, &xsens1_event);
  event_add(&xsens1_event, NULL);

  /* */
  network = network_new(GCS_HOST, GCS_PORT);
  
  /* */
  ahrs = ahrs_new();
  
  /* Initialises GPIO */
  gpio_init();

  event_dispatch();

  return 0;
}


