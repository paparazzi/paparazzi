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

#define AC_ID 161
unsigned char md5[] = {"\207\151\313\256\355\252\016\273\072\126\273\222\017\372\320\200"};

#define PORT_OUT 7024
#define HOSTADDR "localhost"
#define BUFSIZE  0x10000

#define TIMEOUT_PERIOD 5000

#define MAX_PPRZ 9600

#define RadOfDeg(x) ((x) * (M_PI/180.))

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
unsigned char energy;
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

static const float serie_coeff_proj_mercator[5] = {
  0.99832429842242842444,
  0.00083632803657738403,
  0.00000075957783563707,
  0.00000000119563131778,
  0.00000000000241079916
};

static const float serie_coeff_proj_mercator_inverse[5] = {
  0.998324298422428424,
  0.000837732168742475825,
  5.90586914811817062e-08,
  1.6734091890305064e-10,
  2.13883575853313883e-13
};

/* Computation for the WGS84 geoid only */
#define E 0.08181919106
#define K0 0.9996
#define XS 500000.
#define YS 0.
#define A 6378137.0
#define N (K0*A)

struct complex { float re; float im; };

#define LambdaOfUtmZone(utm_zone) RadOfDeg((utm_zone-1)*6-180+3)
#define CScal(k, z) { z.re *= k; z.im *= k; }
#define CAdd(z1, z2) { z2.re += z1.re; z2.im += z1.im; }
#define CSub(z1, z2) { z2.re -= z1.re; z2.im -= z1.im; }
#define CI(z) { float tmp = z.re; z.re = - z.im; z.im = tmp; }
#define CExp(z) { float e = exp(z.re); z.re = e*cos(z.im); z.im = e*sin(z.im); }
#define CSin(z) { CI(z); struct complex _z = {-z.re, -z.im}; float e = exp(z.re); float cos_z_im = cos(z.im); z.re = e*cos_z_im; float sin_z_im = sin(z.im); z.im = e*sin_z_im; _z.re = cos_z_im/e; _z.im = -sin_z_im/e; CSub(_z, z); CScal(-0.5, z); CI(z); }

static inline float isometric_latitude(float phi, float e) {
  return log (tan (M_PI_4 + phi / 2.0)) - e / 2.0 * log((1.0 + e * sin(phi)) / (1.0 - e * sin(phi)));
}

static inline float isometric_latitude0(float phi) {
  return log (tan (M_PI_4 + phi / 2.0));
}

void latlong_utm_of(float phi, float lambda, uint8_t utm_zone) {
  float lambda_c = LambdaOfUtmZone(utm_zone);
  float ll = isometric_latitude(phi , E);
  float dl = lambda - lambda_c;
  float phi_ = asin(sin(dl) / cosh(ll));
  float ll_ = isometric_latitude0(phi_);
  float lambda_ = atan(sinh(ll) / cos(dl));
  struct complex z_ = { lambda_, ll_ };
  CScal(serie_coeff_proj_mercator[0], z_);
  uint8_t k;
  for(k = 1; k < 3; k++) {
    struct complex z = { lambda_, ll_ };
    CScal(2*k, z);
    CSin(z);
    CScal(serie_coeff_proj_mercator[k], z);
    CAdd(z, z_);
  }
  CScal(N, z_);
  latlong_utm_x = XS + z_.im;
  latlong_utm_y = z_.re;
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
      gps_alt = buf2ushort(&buf[8]) * 100;
//    FillBufWith16bit(com_trans.buf, 11, gps_gspeed); // ground speed
      gps_gspeed = buf2ushort(&buf[10]);
//    FillBufWith16bit(com_trans.buf, 13, gps_course); // course
      gps_course = buf2ushort(&buf[12]);
//    FillBufWith16bit(com_trans.buf, 15, (uint16_t)(estimator_airspeed*100)); // TAS (cm/s)
      estimator_airspeed = buf2ushort(&buf[14]);
//    com_trans.buf[16] = electrical.vsupply;
// should be (estimator_airspeed is two bytes):
//    com_trans.buf[17] = electrical.vsupply;
      electrical_vsupply = buf[16];
//    com_trans.buf[17] = (uint8_t)(energy*10);
      energy = buf[17] / 10;
//    com_trans.buf[18] = (uint8_t)(ap_state->commands[COMMAND_THROTTLE]*100/MAX_PPRZ);
      throttle = buf[18];
//    com_trans.buf[19] = pprz_mode;
      pprz_mode = buf[19];
//    com_trans.buf[20] = nav_block;
      nav_block = buf[20];
//    FillBufWith16bit(com_trans.buf, 21, autopilot_flight_time);
      autopilot_flight_time = buf2ushort(&buf[21]);

//gps_lat = 52.2648312 * 1e7;
//gps_lon =  9.9939456 * 1e7;
//gps_alt = 169 * 1000;

//gps_gspeed = 13 * 100;
//gps_course = 60 * 10;
//estimator_airspeed = 15 * 100;
//electrical_vsupply = 126;
//energy = 9;
//throttle = 51;
//pprz_mode = 2;
//nav_block = 1;
//autopilot_flight_time = 123;

      nav_utm_zone0 = (gps_lon/10000000+180) / 6 + 1;
      latlong_utm_of(RadOfDeg(gps_lat/1e7), RadOfDeg(gps_lon/1e7), nav_utm_zone0);
      gps_utm_east = latlong_utm_x * 100;
      gps_utm_north = latlong_utm_y * 100;
      gps_utm_zone = nav_utm_zone0;

printf("gps_lat %f\n", gps_lat/1e7);
printf("gps_lon %f\n", gps_lon/1e7);
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

printf("gps_utm_east %d\n", gps_utm_east);
printf("gps_utm_north %d\n", gps_utm_north);
printf("gps_utm_zone %d\n", gps_utm_zone);

/*
   <message name="GPS" id="8">
     <field name="mode"       type="uint8"  unit="byte_mask"/>
     <field name="utm_east"   type="int32"  unit="cm" alt_unit="m"/>
     <field name="utm_north"  type="int32"  unit="cm" alt_unit="m"/>
     <field name="course"     type="int16"  unit="decideg" alt_unit="deg"/>
     <field name="alt"        type="int32"  unit="mm" alt_unit="m"/>
     <field name="speed"      type="uint16" unit="cm/s" alt_unit="m/s"/>
     <field name="climb"      type="int16"  unit="cm/s" alt_unit="m/s"/>
     <field name="week"       type="uint16" unit="weeks"/>
     <field name="itow"       type="uint32" unit="ms"/>
     <field name="utm_zone"   type="uint8"/>
     <field name="gps_nb_err" type="uint8"/>
   </message>
*/
      IvySendMsg("%d GPS %d %d %d %d %d %d %d %d %d %d %d",
                AC_ID,
                3, // mode = 3D
                gps_utm_east,
                gps_utm_north,
                gps_course,
                gps_alt,
                gps_gspeed,
                0, // climb
                0, // week
                0, //itow
                gps_utm_zone,
                0); // gps_nb_err

/*
   <message name="PPRZ_MODE" id="11">
     <field name="ap_mode" type="uint8" values="MANUAL|AUTO1|AUTO2|HOME|NOGPS|FAILSAFE"/>
     <field name="ap_gaz" type="uint8" values="MANUAL|AUTO_THROTTLE|AUTO_CLIMB|AUTO_ALT"/>
     <field name="ap_lateral" type="uint8" values="MANUAL|ROLL_RATE|ROLL|COURSE"/>
     <field name="ap_horizontal" type="uint8" values="WAYPOINT|ROUTE|CIRCLE"/>
     <field name="if_calib_mode" type="uint8" values="NONE|DOWN|UP"/>
     <field name="mcu1_status" type="uint8" values="LOST|OK|REALLY_LOST"/>
   </message>
*/
      IvySendMsg("%d PPRZ_MODE %d %d %d %d %d %d",
                AC_ID,
                pprz_mode,
                0, // ap_gaz
                0, // ap_lateral
                0, // ap_horizontal
                0, // if_calib_mode
                0); // mcu1_status

/*
  <message name="AIRSPEED" id="54">
    <field name="airspeed" type="float" unit="m/s"/>
    <field name="airspeed_sp" type="float" unit="m/s"/>
    <field name="airspeed_cnt" type="float" unit="m/s"/>
    <field name="groundspeed_sp" type="float" unit="m/s"/>
  </message>
*/
      IvySendMsg("%d AIRSPEED %f %d %d %d",
                AC_ID,
                (float)(estimator_airspeed / 100.),
                0, // airspeed_sp
                0, // airspeed_cnt
                0); // groundspeed_sp

/*
   <message name="BAT" id="12">
     <field name="throttle" type="int16" unit="pprz"/>
     <field name="voltage" type="uint8" unit="1e-1V" alt_unit="V" alt_unit_coef="0.1"/>
     <field name="amps" type="int16" unit="A" alt_unit="A" />
     <field name="flight_time" type="uint16" unit="s"/>
     <field name="kill_auto_throttle" type="uint8" unit="bool"/>
     <field name="block_time" type="uint16" unit="s"/>
     <field name="stage_time" type="uint16" unit="s"/>
     <field name="energy" type="int16" unit="mAh"/>
   </message>
*/
      IvySendMsg("%d BAT %d %d %d %d %d %d %d %d",
                AC_ID,
                throttle * MAX_PPRZ / 100,
                electrical_vsupply,
                0, // amps
                autopilot_flight_time,
                0, // kill_auto_throttle
                0, // block_time
                0, // stage_time
                energy);

/*
   <message name="NAVIGATION" id="10">
     <field name="cur_block" type="uint8"/>
     <field name="cur_stage" type="uint8"/>
     <field name="pos_x" type="float" unit="m" format="%.1f"/>
     <field name="pos_y" type="float" unit="m" format="%.1f"/>
     <field name="dist_wp" type="float" format="%.1f" unit="m"/>
     <field name="dist_home" type="float" format="%.1f" unit="m"/>
     <field name="circle_count" type="uint8"/>
     <field name="oval_count" type="uint8"/>
   </message>
*/
      IvySendMsg("%d NAVIGATION %d %d %d %d %d %d %d %d",
                AC_ID,
                nav_block,
                0, // cur_stage
                0, // pos_x
                0, // pos_y
                0, // dist_wp
                0, // dist_home
                0, // circle_count
                0); // oval_count

/*
  <message name="ESTIMATOR" id="42">
    <field name="z" type="float" unit="m"/>
    <field name="z_dot" type="float" unit="m/s"/>
  </message>
*/
      IvySendMsg("%d ESTIMATOR %f %d",
                AC_ID,
                gps_alt / 1000.,
                0); // z_dot

/*
   <message name="ATTITUDE" id="6">
     <field name="phi"   type="float" unit="rad" alt_unit="deg"/>
     <field name="psi"   type="float" unit="rad" alt_unit="deg"/>
     <field name="theta" type="float" unit="rad" alt_unit="deg"/>
   </message>
*/
      IvySendMsg("%d ATTITUDE %f %f %f",
                AC_ID,
                0., // phi
                RadOfDeg(gps_course / 10.),
                0.); // theta

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
