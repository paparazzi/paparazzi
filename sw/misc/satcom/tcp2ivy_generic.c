/*
 * Paparazzi tcp to ivy handling for sat based telemetry
 *
 * Copyright (C) 2011 Martin Mueller <martinmm@pfump.org>
 *
 * This file is part of paparazzi.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <glib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <math.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include "generated/airframe.h"

//#define AC_ID 161
//unsigned char md5[] = {"\207\151\313\256\355\252\016\273\072\126\273\222\017\372\320\200"};
unsigned char* md5 = (unsigned char*)MD5SUM;

#define PORT_OUT 7024
#define HOSTADDR "localhost"
#define BUFSIZE  0x10000

#define TIMEOUT_PERIOD 5000

#define MAX_PPRZ 9600

#define RadOfDeg(x) ((x) * (M_PI/180.))
#define DegOfRad(x) ((x) * (180./M_PI))

static const char usage_str[] =
"tcp2ivy [options]\n"
"options:\n"
"  -s <server address>\n";

unsigned char gps_mode;
unsigned short gps_week;
unsigned int gps_itow;
unsigned int gps_alt;
unsigned short gps_gspeed;
short gps_climb;
short gps_course;
int gps_utm_east, gps_utm_north;
unsigned char gps_utm_zone;
int gps_lat, gps_lon; /* 1e7 deg */
int gps_hmsl;
short estimator_airspeed;
unsigned short electrical_vsupply;
unsigned char nav_block;
unsigned short energy;
unsigned char throttle;
unsigned short autopilot_flight_time;
unsigned char nav_utm_zone0;
float latlong_utm_x, latlong_utm_y;
unsigned char pprz_mode;


GMainLoop *ml;
int sock, length;
struct sockaddr_in addr;
char hostaddr[4096] = {HOSTADDR};

unsigned int buf2uint(char* dat)
{
  return  (unsigned char)*(dat+0) |
         ((unsigned char)*(dat+1) << 8) |
         ((unsigned char)*(dat+2) << 16)|
         ((unsigned char)*(dat+3) << 24);
}

unsigned short buf2ushort(char* dat)
{
  return  (unsigned char)*(dat+0) |
         ((unsigned char)*(dat+1) << 8);
}

static gboolean read_data(GIOChannel *chan, GIOCondition cond, gpointer data) {
  int count;
  char buf[BUFSIZE];

  /* receive data packet containing formatted data */
  count = recv(sock, buf, sizeof(buf), 0);
  if (count > 0) {
    if (count == 23) {
      //    FillBufWith32bit(com_trans.buf, 1, gps_lat);
      gps_lat = buf2uint(&buf[0]);
      //    FillBufWith32bit(com_trans.buf, 5, gps_lon);
      gps_lon = buf2uint(&buf[4]);
      //    FillBufWith16bit(com_trans.buf, 9, (int16_t)(gps_alt/100)); // meters
      gps_alt = buf2ushort(&buf[8]);
      //    FillBufWith16bit(com_trans.buf, 11, gps_gspeed); // ground speed
      gps_gspeed = buf2ushort(&buf[10]);
      //    FillBufWith16bit(com_trans.buf, 13, gps_course); // course
      gps_course = buf2ushort(&buf[12]);
      //    FillBufWith16bit(com_trans.buf, 15, (uint16_t)(estimator_airspeed*100)); // TAS (cm/s)
      estimator_airspeed = buf2ushort(&buf[14]);
      //    com_trans.buf[17] = electrical.vsupply; // decivolt
      //FIXME: electrical.vsupply is now a uint16
      electrical_vsupply = buf[16];
      //    com_trans.buf[18] = (uint8_t)(energy / 100); // deciAh
      energy = buf[17];
      //    com_trans.buf[19] = (uint8_t)(ap_state->commands[COMMAND_THROTTLE]*100/MAX_PPRZ);
      throttle = buf[18];
      //    com_trans.buf[20] = pprz_mode;
      pprz_mode = buf[19];
      //    com_trans.buf[21] = nav_block;
      nav_block = buf[20];
      //    FillBufWith16bit(com_trans.buf, 22, autopilot_flight_time);
      autopilot_flight_time = buf2ushort(&buf[21]);

#if 0
      gps_lat = 52.2648312 * 1e7;
      gps_lon =  9.9939456 * 1e7;
      gps_alt = 169 * 1000;
      gps_gspeed = 13 * 100;
      gps_course = 60 * 10;
      estimator_airspeed = 15 * 100;
      electrical_vsupply = 126;
      energy = 9;
      throttle = 51;
      pprz_mode = 2;
      nav_block = 1;
      autopilot_flight_time = 123;
#endif

      printf("**** message received from iridium module ****\n");
      printf("gps_lat %f\n", DegOfRad(gps_lat/1e7));
      printf("gps_lon %f\n", DegOfRad(gps_lon/1e7));
      printf("gps_alt %d\n", gps_alt);
      printf("gps_gspeed %d\n", gps_gspeed);
      printf("gps_course %d\n", gps_course);
      printf("estimator_airspeed %d\n", estimator_airspeed);
      printf("electrical_vsupply %d\n", electrical_vsupply);
      printf("energy %d\n", energy);
      printf("throttle %d\n", throttle);
      printf("pprz_mode %d\n", pprz_mode);
      printf("nav_block %d\n", nav_block);
      printf("autopilot_flight_time %d\n", autopilot_flight_time);
      fflush(stdout);

      IvySendMsg("%d GENERIC_COM %d %d %d %d %d %d %d %d %d %d %d %d",
          AC_ID,
          gps_lat,
          gps_lon,
          gps_alt,
          gps_gspeed,
          gps_course,
          estimator_airspeed,
          electrical_vsupply,
          energy,
          throttle,
          pprz_mode,
          nav_block,
          autopilot_flight_time);

    }
    else {
      int i = 0;
      // Raw print
      printf("**** Raw message ****\n");
      printf("Char: ");
      for (i = 0; i < count; i++) printf("%c",buf[i]);
      printf("\nHex: ");
      for (i = 0; i < count; i++) printf("%x ",buf[i]);
      printf("\n");
    }
  }
  else {
    printf("disconnect\n");
    close(sock);
    g_main_loop_quit(ml);
    return 0;
  }

  return 1;
}

static gboolean alive(gpointer data __attribute__ ((unused))) {
  /*
     <message name="ALIVE" id="2">
     <field name="md5sum" type="uint8[]"/>
     </message>
     */
  IvySendMsg("%d ALIVE %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,",
      AC_ID,
      md5[0], md5[1], md5[2], md5[3],
      md5[4], md5[5], md5[6], md5[7],
      md5[8], md5[9], md5[10], md5[11],
      md5[12], md5[13], md5[14], md5[15]);

  return 1;
}

int main ( int argc, char** argv) {
  GIOChannel *sk;
  struct hostent *hent;
  int c;

  printf("Starting TCP2IVY agent for AC %s (id:%d)\n", AIRFRAME_NAME, AC_ID);
  fflush(stdout);

  IvyInit ("IvySatCom", "IvySatCom READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  while ((c = getopt(argc, argv, "h:s:")) != EOF) {
    switch (c) {

    case 'h':
      printf(usage_str);
      break;

    case 's':
      strncpy(hostaddr, optarg, strlen(optarg)+1);
      break;
    }
  }

  sock = socket(PF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    perror("socket");
    exit(1);
  }

  hent = gethostbyname(hostaddr);
  if (hent == 0) {
    perror("unknown host");
    exit(1);
  }

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(PORT_OUT);
  memcpy((char *)&addr.sin_addr,
         (char *)hent->h_addr_list[0],
         4);

  if (connect(sock, (struct sockaddr *) &addr, sizeof(addr))) {
    perror("connect");
    exit(1);
  }

  sk = g_io_channel_unix_new(sock);
  g_io_add_watch(sk, G_IO_IN | G_IO_NVAL | G_IO_HUP,
                 read_data, NULL);
  g_timeout_add(TIMEOUT_PERIOD, alive, NULL);

  ml =  g_main_loop_new(NULL, FALSE);

  g_main_loop_run(ml);

  return 0;
}
