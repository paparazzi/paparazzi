/*
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

#include "subsystems/gps.h"

#include "messages2.h"
//#include "dl_protocol2.h"
#include "generated/airframe.h"

#include "fms_periodic.h"
#include "fms_debug.h"
#include "serial_port.h"

#include "overo_gcs_com.h"
#include "uart_hw.h"
#include "subsystems/datalink/pprz_transport.h"


struct OveroController {
  int armed;
} controller;

static void main_periodic(int);
static void main_exit(int sig);
static void main_talk_with_tiny(void);
void check_gps(void);

//make gps.c happy without including navigation code
uint8_t nav_utm_zone0 = 31;

static uint16_t foo = 0;
//struct SerialPort* fmssp;
//int spfd;
uint8_t portnum;
#ifdef GPS_CONFIGURE
static uint8_t donegpsconf = 0;
#endif
static uint8_t configgps = 0;


extern float phi;
extern float psi;
extern float theta;
extern uint16_t throttle;
extern uint16_t voltage;
extern uint16_t amps;
extern uint16_t energy;
extern uint16_t adc1;
extern uint16_t adc2;

int main(int argc, char *argv[])
{
  portnum = 0;

  if (argc > 1) {
    portnum = atoi(argv[1]);
    if (portnum > 10) {
      printf("Port number must be <11\n");
      return -1;
    }
    if (argc > 2) { configgps = atoi(argv[2]); }
    if (configgps)
#ifdef GPS_CONFIGURE
      printf("Will configure GPS.\n");
#else
      printf("Rebuild with GPS configure support.\n");
#endif
  }

  printf("Using /dev/ttyUSB%d for GPS\n", portnum);


  (void) signal(SIGINT, main_exit);

  uart_init();
  gps_init();

  /* Initalize the event library */
  event_init();

  gcs_com_init();

  if (fms_periodic_init(main_periodic)) {
    TRACE(TRACE_ERROR, "%s", "failed to start periodic generator\n");
    return -1;
  }

#ifdef GPS_CONFIGURE
  //periodic task is launched so we are now ready to use uart to request gps baud change...
  if (configgps) { gps_configure_uart(); }
#endif
  event_dispatch();
  //should never occur!
  printf("goodbye! (%d)\n", foo);

  return 0;
}


static void main_periodic(int my_sig_num)
{


  RunOnceEvery(50, {DOWNLINK_SEND_ALIVE(gcs_com.udp_transport, 16, MD5SUM);});
  RunOnceEvery(5, {DOWNLINK_SEND_ADC_GENERIC(gcs_com.udp_transport, &adc1, &adc2);});

#if USE_UART0
  uart0_handler();
#endif
#if USE_UART1
  uart1_handler();
#endif
  DatalinkEvent()
  main_talk_with_tiny();
  check_gps();

  RunOnceEvery(20, gcs_com_periodic());

}

#include "subsystems/datalink/downlink.h"
#if 0
uint8_t downlink.nb_ovrn;
uint16_t downlink.nb_bytes;
uint16_t downlink.nb_msgs;

#define __Transport(dev, _x) dev##_x
#define _Transport(dev, _x) __Transport(dev, _x)
#define Transport(_chan, _fun) _Transport(_chan, _fun)
#define DownlinkIDsSize(_chan, _x) (_x+2)
#define DownlinkSizeOf(_chan, _x) Transport(_chan, SizeOf(DownlinkIDsSize(_chan, _x)))

#define DownlinkCheckFreeSpace(_chan, _x) Transport(_chan, CheckFreeSpace((uint8_t)(_x)))

#define DownlinkPutUint8ByAddr(_chan, _x) Transport(_chan, PutUint8ByAddr(_x))
#define DownlinkPutUint8Array(_chan, _n, _x) Transport(_chan, PutUint8Array(_n, _x))

#define DownlinkOverrun(_chan) downlink.nb_ovrn++;
#define DownlinkCountBytes(_chan, _n) downlink.nb_bytes += _n;

#define DownlinkStartMessage(_chan, _name, msg_id, payload_len) { \
    downlink.nb_msgs++; \
    Transport(_chan, Header(DownlinkIDsSize(_chan, payload_len))); \
    Transport(_chan, PutUint8(AC_ID)); \
    Transport(_chan, PutNamedUint8(_name, msg_id)); \
  }

#define DownlinkEndMessage(_chan) Transport(_chan, Trailer())


#define __DOWNLINK_SEND_HITL_UBX(_chan, class, id, ac_id, nb_ubx_payload, ubx_payload){ \
    if (DownlinkCheckFreeSpace(_chan, DownlinkSizeOf(_chan, 0+1+1+1+1+nb_ubx_payload*1))) {\
      DownlinkCountBytes(_chan, DownlinkSizeOf(_chan, 0+1+1+1+1+nb_ubx_payload*1)); \
      DownlinkStartMessage(_chan, "HITL_UBX", DL_HITL_UBX, 0+1+1+1+1+nb_ubx_payload*1) \
      DownlinkPutUint8ByAddr(_chan, (class)); \
      DownlinkPutUint8ByAddr(_chan, (id)); \
      DownlinkPutUint8ByAddr(_chan, (ac_id)); \
      DownlinkPutUint8Array(_chan, nb_ubx_payload, ubx_payload); \
      DownlinkEndMessage(_chan ) \
    } else \
      DownlinkOverrun(_chan ); \
  }
#endif

void check_gps(void)
{

  /*  if (GpsTimeoutError) {
      printf("gps timeout\n");
    }*/
  if (GpsBuffer()) {
    ReadGpsBuffer();
  }

  if (gps_msg_received) {
#ifdef GPS_CONFIGURE
    if (gps_configuring) {
      gps_configure();
    } else {
      if (!donegpsconf) {
        printf("Finished GPS configuration.\n");
        donegpsconf = 1;
      }
      parse_gps_msg();
    }
#else
    parse_gps_msg();
#endif
    printf("gps msg rx %x %x\n", ubx_class, ubx_id);
    const uint8_t ac_id = 3;
    //DOWNLINK_SEND_HITL_UBX(gcs_com.udp_transport, &ubx_class, &ubx_id, &ac_id,  &ubx_len ,ubx_msg_buf);
    DOWNLINK_SEND_HITL_UBX(PprzTransport, &ubx_class, &ubx_id, &ac_id, ubx_len , ubx_msg_buf);
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

static void main_exit(int sig)
{
  printf("Application Exiting...\n");
  exit(EXIT_SUCCESS);
}

static void main_talk_with_tiny()
{
  //unsigned char c='D';
  //write(spfd,&c,1);
  //if (read(spfd,&c,1)>0) write(STDOUT_FILENO,&c,1);
  //fprintf(spfd,"testing\n");
  //printf(".");
  foo++;
}

