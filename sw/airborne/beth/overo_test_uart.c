/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
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

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <event.h>

#include "gps.h"

#include "messages2.h"
#include "airframe.h"

#include "fms_periodic.h"
#include "fms_debug.h"
#include "fms_serial_port.h"

#include "overo_gcs_com.h"
#include "uart_hw.h"

struct OveroController {
  int armed;
} controller;

static void main_periodic(int);
static void main_exit(int sig);
static void main_talk_with_tiny(void);
void check_gps(void);

//make gps.c happy without incluing navigation code
uint8_t nav_utm_zone0 = 5;

static uint16_t foo = 0;
//struct FmsSerialPort* fmssp;
//int spfd;
uint8_t portnum;

int main(int argc, char *argv[]) {

  if (argc > 1) {
    if (portnum > 10 ) {
      printf("Port number must be <11\n");
      return -1;
    }
    portnum = atoi(argv[1]);
  } else portnum = 0;

  printf("Using /dev/ttyUSB%d\n",portnum);
  
  (void) signal(SIGINT, main_exit);

  uart_init();
  gps_init();

/*  fmssp = serial_port_new();
  //speed_t speed;

  if (serial_port_open_raw(fmssp,"/dev/ttyUSB0",B9600)){
    printf("error opening USB serial port!");
    return -1;
  } 

  spfd = (int)fmssp->fd;  
*/
  /* Initalize the event library */
  event_init();

  gcs_com_init();

  if (fms_periodic_init(main_periodic)) {
    TRACE(TRACE_ERROR, "%s", "failed to start periodic generator\n");
    return -1; 
  }
  
  event_dispatch();
  //should never occur!
  printf("goodbye! (%d)\n",foo);

  return 0;
}

static uint16_t tempstatus = 0;

static void main_periodic(int my_sig_num) {


  RunOnceEvery(10, {DOWNLINK_SEND_MOTOR(gcs_com.udp_transport, &tempstatus, &foo );});
  
  RunOnceEvery(50, {DOWNLINK_SEND_ALIVE(gcs_com.udp_transport, 16, MD5SUM);});

#ifdef USE_UART0 
  uart0_handler();
#elif USE_UART1
  uart1_handler();
#endif
  main_talk_with_tiny();
  check_gps();

  RunOnceEvery(20, gcs_com_periodic());

}



void check_gps(void){

/*  if (GpsTimeoutError) {
    printf("gps timeout\n");
  }
*/
  if (GpsBuffer()) {
    ReadGpsBuffer();
  }

  if (gps_msg_received) {
    printf("gps msg rx\n");
    /* parse and use GPS messages */
    parse_gps_msg();
    gps_msg_received = FALSE;
    if (gps_pos_available) {
      printf("gps pos avail\n");
      gps_verbose_downlink = 0;
//      UseGpsPosNoSend(estimator_update_state_gps);
      gps_downlink();
      gps_pos_available = FALSE;
    }
  }
}

static void main_exit(int sig) {
  printf("Initiating shutdown...\n");

  printf("Application Exiting...\n");
  exit(EXIT_SUCCESS);
}

static void main_talk_with_tiny() {
  //unsigned char c='D';
  //write(spfd,&c,1);
  //if (read(spfd,&c,1)>0) write(STDOUT_FILENO,&c,1);
  //fprintf(spfd,"testing\n");
  //printf(".");
  foo++;
}

