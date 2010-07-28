/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fms_network.h"
#include "fms_periodic.h"
#include "fms_debug.h"
#include "fms_spi_link.h"
#include "fms_autopilot_msg.h"

#include <event.h>

#include "downlink.h"
#include "udp_transport.h"

#define GCS_HOST "10.31.4.104"
#define GCS_PORT 4242
#define DATALINK_PORT 4243


static void main_periodic(int);
static void on_datalink_event(int fd, short event, void *arg);

static struct FmsNetwork* network;

static struct AutopilotMessageBethUp   msg_in;
static struct AutopilotMessageBethDown msg_out;
static void send_message(void);

uint16_t az,elev,tilt;

static uint32_t foo = 0;

static void main_periodic(int my_sig_num) {

  RunOnceEvery(10, {DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);});

  send_message();

  RunOnceEvery(10, {DOWNLINK_SEND_BETH(DefaultChannel,&msg_in.bench_sensor.x,&msg_in.bench_sensor.y,
    &msg_in.bench_sensor.z,&foo);});

  RunOnceEvery(10, {DOWNLINK_SEND_BOOZ2_GYRO(DefaultChannel,
			     &msg_in.gyro.p,
			     &msg_in.gyro.q,
			     &msg_in.gyro.r);});
    

  RunOnceEvery(10, {DOWNLINK_SEND_BOOZ2_ACCEL(DefaultChannel,
			      &msg_in.accel.x,
			      &msg_in.accel.y,
			      &msg_in.accel.z);});

  RunOnceEvery(10, {UdpTransportPeriodic();});

}


int main(int argc, char *argv[]) {
  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return -1;
  }

  network = network_new(GCS_HOST, GCS_PORT, DATALINK_PORT, FALSE);

  /* Initalize the event library */
  event_init();

  if (fms_periodic_init(main_periodic)) {
    TRACE(TRACE_ERROR, "%s", "failed to start periodic generator\n");
    return -1; 
  }

  network = network_new(GCS_HOST, GCS_PORT, DATALINK_PORT, FALSE);
  struct event datalink_event;
  event_set(&datalink_event, network->socket_in, EV_READ, on_datalink_event, &datalink_event);
  event_add(&datalink_event, NULL);

  event_dispatch();

  return 0;
}

static int8_t pitchval = 0;
static int8_t adder = 1;

static void send_message() {
  //uint8_t *fooptr;

  msg_out.thrust = 7;

  if (!(foo%100)) { 
    if (pitchval == 10 ) adder=-1;
    if (pitchval == -10 ) adder=1;
    pitchval = pitchval + adder;
    printf("pitchval now %d\n",pitchval);
  }
  msg_out.pitch = pitchval;
  //msg_out.cksum = msg_out.thrust + msg_out.pitch;

  spi_link_send(&msg_out, sizeof(union AutopilotMessage) , &msg_in);

  /*if (msg_in.bench_sensor.x == 0){
    fooptr = &msg_in;
    for (int i=0; i<sizeof(msg_in); i++)
      printf("%02x ",  fooptr[i]);
    printf("\n");
  }*/

  /*if (!(foo%10)) { 
    printf("%d %d %d %x\r\n",msg_in.bench_sensor.x,msg_in.bench_sensor.y,
    msg_in.bench_sensor.z,msg_in.bench_sensor.x);   
  }*/
  foo++;
}

static void on_datalink_event(int fd, short event, void *arg) {
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
