/*
 * Paparazzi mcu0 $Id$
 *
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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

/** \file gps_ubx.c
 * \brief Parser for the UBX protocol (u-blox.com devices)
 */

#include <inttypes.h>
#include <string.h>
#include <math.h>

#ifdef FMS_PERIODIC_FREQ
//for printf
#include <stdio.h>
//for baudrate
#include "fms_serial_port.h"
#endif /* FMS_PERIODIC_FREQ */

#include "generated/flight_plan.h"
#include "mcu_periph/uart.h"
#include "gps.h"
#include "gps_ubx.h"
#include "subsystems/nav.h"
#include "latlong.h"

#ifdef GPS_TIMESTAMP
#include "sys_time.h"
#define MSEC_PER_WEEK (1000*60*60*24*7)
#endif

#define UbxInitCheksum() { send_ck_a = send_ck_b = 0; }
#define UpdateChecksum(c) { send_ck_a += c; send_ck_b += send_ck_a; }
#define UbxTrailer() { GpsUartSend1(send_ck_a);  GpsUartSend1(send_ck_b); GpsUartSendMessage(); }

#define UbxSend1(c) { uint8_t i8=c; GpsUartSend1(i8); UpdateChecksum(i8); }
#define UbxSend2(c) { uint16_t i16=c; UbxSend1(i16&0xff); UbxSend1(i16 >> 8); }
#define UbxSend1ByAddr(x) { UbxSend1(*x); }
#define UbxSend2ByAddr(x) { UbxSend1(*x); UbxSend1(*(x+1)); }
#define UbxSend4ByAddr(x) { UbxSend1(*x); UbxSend1(*(x+1)); UbxSend1(*(x+2)); UbxSend1(*(x+3)); }

#define UbxHeader(nav_id, msg_id, len) { \
  GpsUartSend1(UBX_SYNC1); \
  GpsUartSend1(UBX_SYNC2); \
  UbxInitCheksum(); \
  UbxSend1(nav_id); \
  UbxSend1(msg_id); \
  UbxSend2(len); \
}


/** Includes macros generated from ubx.xml */
#include "ubx_protocol.h"


uint16_t gps_week;
uint32_t gps_itow;
#ifdef GPS_TIMESTAMP
uint32_t gps_t0;
uint32_t gps_t0_itow;
uint32_t gps_t0_frac;
#endif
int32_t gps_alt;
uint16_t gps_speed_3d;
uint16_t gps_gspeed;
int16_t gps_climb;
int16_t gps_course;
int32_t gps_utm_east, gps_utm_north;
uint8_t gps_utm_zone;
uint8_t gps_mode;
uint8_t gps_status_flags;
uint8_t gps_sol_flags;
volatile bool_t gps_msg_received;
bool_t gps_pos_available;
uint8_t ubx_id, ubx_class;
uint16_t ubx_len;
int32_t gps_lat, gps_lon;
int32_t gps_hmsl;
uint16_t gps_reset;

uint16_t gps_PDOP;
uint32_t gps_Pacc, gps_Sacc;
int32_t gps_ecefVZ;
uint8_t gps_numSV;

#define UTM_HEM_NORTH 0
#define UTM_HEM_SOUTH 1

#define UBX_MAX_PAYLOAD 255

uint8_t ubx_msg_buf[UBX_MAX_PAYLOAD] __attribute__ ((aligned));

#define UNINIT        0
#define GOT_SYNC1     1
#define GOT_SYNC2     2
#define GOT_CLASS     3
#define GOT_ID        4
#define GOT_LEN1      5
#define GOT_LEN2      6
#define GPS_UBX_GOT_PAYLOAD   7
#define GOT_CHECKSUM1 8

static uint8_t  ubx_status;
static uint8_t  ubx_msg_idx;
static uint8_t ck_a, ck_b;
uint8_t send_ck_a, send_ck_b;

bool_t gps_configuring;
#ifdef GPS_CONFIGURE
static uint8_t gps_status_config;
#endif

void gps_init( void ) {
  ubx_status = UNINIT;
#ifdef GPS_CONFIGURE
  gps_status_config = 0;
  gps_configuring = TRUE;
#endif
}

#define UBX_PROTO_MASK  0x0001
#define NMEA_PROTO_MASK 0x0002
#define RTCM_PROTO_MASK 0x0004

#define GPS_PORT_DDC   0x00
#define GPS_PORT_UART1 0x01
#define GPS_PORT_UART2 0x02
#define GPS_PORT_USB   0x03
#define GPS_PORT_SPI   0x04


#ifdef GPS_CONFIGURE
/* GPS dynamic configuration */


#ifndef GPS_PORT_ID
#define GPS_PORT_ID GPS_PORT_UART1
#endif


/* Configure the GPS baud rate using the current uart baud rate. Busy
   wait for the end of the transmit. Then, BEFORE waiting for the ACK,
   change the uart rate. */
#if GPS_PORT_ID == GPS_PORT_UART1 || GPS_PORT_ID == GPS_PORT_UART2
void gps_configure_uart ( void ) {
#ifdef FMS_PERIODIC_FREQ
  UbxSend_CFG_PRT(GPS_PORT_ID, 0x0, 0x0, 0x000008D0, 38400, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);
  uint8_t loop=0;
  while (GpsUartRunning) {
    //doesn't work unless some printfs are used, so :
    if (loop<9) {
      printf("."); loop++;
    } else {
      printf("\b"); loop--;
    }
  }
#else
  UbxSend_CFG_PRT(GPS_PORT_ID, 0x0, 0x0, 0x000008D0, GPS_BAUD, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);
  while (GpsUartRunning); /* FIXME */
#endif

  GpsUartSetBaudrate(GPS_BAUD);
}
#endif

#if GPS_PORT_ID == GPS_PORT_DDC
void gps_configure_uart ( void ) {
  UbxSend_CFG_PRT(GPS_PORT_ID, 0x0, 0x0, GPS_I2C_SLAVE_ADDR, 0x0, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);
}
#endif

#define IGNORED 0
#define RESERVED 0

#ifdef USER_GPS_CONFIGURE
#include USER_GPS_CONFIGURE
#else
static bool_t user_gps_configure(bool_t cpt) {
  switch (cpt) {
  case 0:
    //New ublox firmware v5 or higher uses CFG_NAV5 message, CFG_NAV is no longer available
    //UbxSend_CFG_NAV(NAV_DYN_AIRBORNE_2G, 3, 16, 24, 20, 5, 0, 0x3C, 0x3C, 0x14, 0x03E8 ,0x0000, 0x0, 0x17, 0x00FA, 0x00FA, 0x0064, 0x012C, 0x000F, 0x00, 0x00);
    UbxSend_CFG_NAV5(0x05, NAV5_DYN_AIRBORNE_2G, NAV5_3D_ONLY, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, RESERVED, RESERVED, RESERVED, RESERVED);
    break;
  case 1:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_POSUTM_ID, 0, 1, 0, 0);
    break;
  case 2:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_VELNED_ID, 0, 1, 0, 0);
    break;
  case 3:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_STATUS_ID, 0, 1, 0, 0);
    break;
  case 4:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_SVINFO_ID, 0, 4, 0, 0);
    break;
  case 5:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_SOL_ID, 0, 8, 0, 0);
    break;
  case 6:
    UbxSend_CFG_SBAS(0x00, 0x00, 0x00, 0x00, 0x00);
    break;
  case 7:
    UbxSend_CFG_RATE(0x00FA, 0x0001, 0x0000);
    return FALSE;
  }
  return TRUE; /* Continue, except for the last case */
}
#endif // ! USER_GPS_CONFIGURE

/* GPS configuration. Must be called on ack message reception while
   gps_status_config < GPS_CONFIG_DONE */
void gps_configure ( void ) {
  if (ubx_class == UBX_ACK_ID) {
    if (ubx_id == UBX_ACK_ACK_ID) {
      gps_status_config++;
    }
  }
  gps_configuring = user_gps_configure(gps_status_config);
}
#endif /* GPS_CONFIGURE */

void ubxsend_cfg_rst(uint16_t bbr , uint8_t reset_mode) {
#ifdef GPS_LINK
  UbxSend_CFG_RST(bbr, reset_mode, 0x00);
#endif /* else less harmful for HITL */
}


struct svinfo gps_svinfos[GPS_NB_CHANNELS];
uint8_t gps_nb_channels;

void parse_gps_msg( void ) {
  if (ubx_class == UBX_NAV_ID) {
    if (ubx_id == UBX_NAV_STATUS_ID) {
      gps_mode = UBX_NAV_STATUS_GPSfix(ubx_msg_buf);
      gps_status_flags = UBX_NAV_STATUS_Flags(ubx_msg_buf);
      gps_sol_flags = UBX_NAV_SOL_Flags(ubx_msg_buf);
#ifdef GPS_USE_LATLONG
      /* Computes from (lat, long) in the referenced UTM zone */
    } else if (ubx_id == UBX_NAV_POSLLH_ID) {
      gps_lat = UBX_NAV_POSLLH_LAT(ubx_msg_buf);
      gps_lon = UBX_NAV_POSLLH_LON(ubx_msg_buf);
      gps_hmsl = UBX_NAV_POSLLH_HMSL(ubx_msg_buf);

      latlong_utm_of(RadOfDeg(gps_lat/1e7), RadOfDeg(gps_lon/1e7), nav_utm_zone0);

      gps_utm_east = latlong_utm_x * 100;
      gps_utm_north = latlong_utm_y * 100;
      gps_alt = UBX_NAV_POSLLH_HMSL(ubx_msg_buf) / 10;
      gps_utm_zone = nav_utm_zone0;
#else
    } else if (ubx_id == UBX_NAV_POSUTM_ID) {
      gps_utm_east = UBX_NAV_POSUTM_EAST(ubx_msg_buf);
      gps_utm_north = UBX_NAV_POSUTM_NORTH(ubx_msg_buf);
      uint8_t hem = UBX_NAV_POSUTM_HEM(ubx_msg_buf);
      if (hem == UTM_HEM_SOUTH)
        gps_utm_north -= 1000000000; /* Subtract false northing: -10000km */
      gps_alt = UBX_NAV_POSUTM_ALT(ubx_msg_buf);
      gps_utm_zone = UBX_NAV_POSUTM_ZONE(ubx_msg_buf);
#endif
    } else if (ubx_id == UBX_NAV_VELNED_ID) {
      gps_speed_3d = UBX_NAV_VELNED_Speed(ubx_msg_buf);
      gps_gspeed = UBX_NAV_VELNED_GSpeed(ubx_msg_buf);
      gps_climb = - UBX_NAV_VELNED_VEL_D(ubx_msg_buf);
      gps_course = UBX_NAV_VELNED_Heading(ubx_msg_buf) / 10000;
      gps_itow = UBX_NAV_VELNED_ITOW(ubx_msg_buf);

      gps_pos_available = TRUE; /* The 3 UBX messages are sent in one rafale */
    } else if (ubx_id == UBX_NAV_SOL_ID) {
#ifdef GPS_TIMESTAMP
      /* get hardware clock ticks */
      gps_t0 = T0TC;
      /* set receive time */
      gps_t0_itow = UBX_NAV_SOL_ITOW(ubx_msg_buf);
      gps_t0_frac = UBX_NAV_SOL_Frac(ubx_msg_buf);
#endif
      gps_mode = UBX_NAV_SOL_GPSfix(ubx_msg_buf);
      gps_PDOP = UBX_NAV_SOL_PDOP(ubx_msg_buf);
      gps_Pacc = UBX_NAV_SOL_Pacc(ubx_msg_buf);
      gps_ecefVZ = UBX_NAV_SOL_ECEFVZ(ubx_msg_buf);
      gps_Sacc = UBX_NAV_SOL_Sacc(ubx_msg_buf);
      gps_numSV = UBX_NAV_SOL_numSV(ubx_msg_buf);
      gps_week = UBX_NAV_SOL_week(ubx_msg_buf);
    } else if (ubx_id == UBX_NAV_SVINFO_ID) {
      gps_nb_channels = Min(UBX_NAV_SVINFO_NCH(ubx_msg_buf), GPS_NB_CHANNELS);
      uint8_t i;
      for(i = 0; i < gps_nb_channels; i++) {
        gps_svinfos[i].svid = UBX_NAV_SVINFO_SVID(ubx_msg_buf, i);
        gps_svinfos[i].flags = UBX_NAV_SVINFO_Flags(ubx_msg_buf, i);
        gps_svinfos[i].qi = UBX_NAV_SVINFO_QI(ubx_msg_buf, i);
        gps_svinfos[i].cno = UBX_NAV_SVINFO_CNO(ubx_msg_buf, i);
        gps_svinfos[i].elev = UBX_NAV_SVINFO_Elev(ubx_msg_buf, i);
        gps_svinfos[i].azim = UBX_NAV_SVINFO_Azim(ubx_msg_buf, i);
      }
    }
  }
}


uint8_t gps_nb_ovrn;


void parse_ubx( uint8_t c ) {
  if (ubx_status < GPS_UBX_GOT_PAYLOAD) {
    ck_a += c;
    ck_b += ck_a;
  }
  switch (ubx_status) {
  case UNINIT:
    if (c == UBX_SYNC1)
      ubx_status++;
    break;
  case GOT_SYNC1:
    if (c != UBX_SYNC2)
      goto error;
    ck_a = 0;
    ck_b = 0;
    ubx_status++;
    break;
  case GOT_SYNC2:
    if (gps_msg_received) {
      /* Previous message has not yet been parsed: discard this one */
      gps_nb_ovrn++;
      goto error;
    }
    ubx_class = c;
    ubx_status++;
    break;
  case GOT_CLASS:
    ubx_id = c;
    ubx_status++;
    break;
  case GOT_ID:
    ubx_len = c;
    ubx_status++;
    break;
  case GOT_LEN1:
    ubx_len |= (c<<8);
    if (ubx_len > UBX_MAX_PAYLOAD)
      goto error;
    ubx_msg_idx = 0;
    ubx_status++;
    break;
  case GOT_LEN2:
    ubx_msg_buf[ubx_msg_idx] = c;
    ubx_msg_idx++;
    if (ubx_msg_idx >= ubx_len) {
      ubx_status++;
    }
    break;
  case GPS_UBX_GOT_PAYLOAD:
    if (c != ck_a)
      goto error;
    ubx_status++;
    break;
  case GOT_CHECKSUM1:
    if (c != ck_b)
      goto error;
    gps_msg_received = TRUE;
    goto restart;
    break;
  default:
    goto error;
  }
  return;
 error:
 restart:
  ubx_status = UNINIT;
  return;
}

#ifdef GPS_TIMESTAMP

#ifndef PCLK
#error unknown PCLK frequency
#endif

uint32_t itow_from_ticks(uint32_t clock_ticks)
{
  uint32_t clock_delta;
  uint32_t time_delta;
  uint32_t itow_now;

  if (clock_ticks < gps_t0) {
    clock_delta = (0xFFFFFFFF - clock_ticks) + gps_t0 + 1;
  } else {
    clock_delta = clock_ticks - gps_t0;
  }

  time_delta = MSEC_OF_SYS_TICS(clock_delta);

  itow_now = gps_t0_itow + time_delta;
  if (itow_now > MSEC_PER_WEEK) itow_now %= MSEC_PER_WEEK;

  return itow_now;
}
#endif
