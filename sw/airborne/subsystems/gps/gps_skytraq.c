/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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

#include "subsystems/gps.h"
#include "led.h"

#if GPS_USE_LATLONG
/* currently needed to get nav_utm_zone0 */
#include "subsystems/navigation/common_nav.h"
#include "math/pprz_geodetic_float.h"
#endif

struct GpsSkytraq gps_skytraq;

/* parser status */
#define UNINIT        0
#define GOT_SYNC1     1
#define GOT_SYNC2     2
#define GOT_LEN1      3
#define GOT_LEN2      4
#define GOT_ID        5
#define GOT_PAYLOAD   6
#define GOT_CHECKSUM  7
#define GOT_SYNC3     8

#define SKYTRAQ_FIX_NONE    0x00
#define SKYTRAQ_FIX_2D      0x01
#define SKYTRAQ_FIX_3D      0x02
#define SKYTRAQ_FIX_3D_DGPS 0x03

//#include "my_debug_servo.h"

void gps_impl_init(void) {

  gps_skytraq.status = UNINIT;


  //DEBUG_SERVO1_INIT();

}


void gps_skytraq_read_message(void) {

  //DEBUG_S1_ON();

  if (gps_skytraq.msg_id == SKYTRAQ_ID_NAVIGATION_DATA) {
    gps.ecef_pos.x  = SKYTRAQ_NAVIGATION_DATA_ECEFX(gps_skytraq.msg_buf);
    gps.ecef_pos.y  = SKYTRAQ_NAVIGATION_DATA_ECEFY(gps_skytraq.msg_buf);
    gps.ecef_pos.z  = SKYTRAQ_NAVIGATION_DATA_ECEFZ(gps_skytraq.msg_buf);
    gps.ecef_vel.x  = SKYTRAQ_NAVIGATION_DATA_ECEFVX(gps_skytraq.msg_buf);
    gps.ecef_vel.y  = SKYTRAQ_NAVIGATION_DATA_ECEFVY(gps_skytraq.msg_buf);
    gps.ecef_vel.z  = SKYTRAQ_NAVIGATION_DATA_ECEFVZ(gps_skytraq.msg_buf);
    gps.lla_pos.lat = RadOfDeg(SKYTRAQ_NAVIGATION_DATA_LAT(gps_skytraq.msg_buf));
    gps.lla_pos.lon = RadOfDeg(SKYTRAQ_NAVIGATION_DATA_LON(gps_skytraq.msg_buf));
    gps.lla_pos.alt = SKYTRAQ_NAVIGATION_DATA_AEL(gps_skytraq.msg_buf)/10;
    gps.hmsl        = SKYTRAQ_NAVIGATION_DATA_ASL(gps_skytraq.msg_buf)/10;
    //   pacc;
    //   sacc;
    //     gps.pdop       = SKYTRAQ_NAVIGATION_DATA_PDOP(gps_skytraq.msg_buf);
    gps.num_sv      = SKYTRAQ_NAVIGATION_DATA_NumSV(gps_skytraq.msg_buf);
    gps.tow         = SKYTRAQ_NAVIGATION_DATA_TOW(gps_skytraq.msg_buf)/10;

    switch (SKYTRAQ_NAVIGATION_DATA_FixMode(gps_skytraq.msg_buf)) {
    case SKYTRAQ_FIX_3D_DGPS:
    case SKYTRAQ_FIX_3D:
      gps.fix = GPS_FIX_3D;
      break;
    case SKYTRAQ_FIX_2D:
      gps.fix = GPS_FIX_2D;
      break;
    default:
      gps.fix = GPS_FIX_NONE;
    }

#if GPS_USE_LATLONG
    /* Computes from (lat, long) in the referenced UTM zone */
    struct LlaCoor_f lla_f;
    lla_f.lat = ((float) gps.lla_pos.lat) / 1e7;
    lla_f.lon = ((float) gps.lla_pos.lon) / 1e7;
    struct UtmCoor_f utm_f;
    utm_f.zone = nav_utm_zone0;
    /* convert to utm */
    utm_of_lla_f(&utm_f, &lla_f);
    /* copy results of utm conversion */
    gps.utm_pos.east = utm_f.east*100;
    gps.utm_pos.north = utm_f.north*100;
    gps.utm_pos.alt = gps.lla_pos.alt;
    gps.utm_pos.zone = nav_utm_zone0;
#endif

    //DEBUG_S2_TOGGLE();

#ifdef GPS_LED
    if (gps.fix == GPS_FIX_3D) {
      LED_ON(GPS_LED);
    }
    else {
      LED_TOGGLE(GPS_LED);
    }
#endif
  }

  //DEBUG_S1_OFF();
}

void gps_skytraq_parse(uint8_t c) {
  if (gps_skytraq.status < GOT_PAYLOAD)
    gps_skytraq.checksum ^= c;
  switch (gps_skytraq.status) {
  case UNINIT:
    if (c == SKYTRAQ_SYNC1)
      gps_skytraq.status = GOT_SYNC1;
    break;
  case GOT_SYNC1:
    if (c != SKYTRAQ_SYNC2) {
      gps_skytraq.error_last = GPS_SKYTRAQ_ERR_OUT_OF_SYNC;
      goto error;
    }
    gps_skytraq.status = GOT_SYNC2;
    break;
  case GOT_SYNC2:
    gps_skytraq.len = c<<8;
    gps_skytraq.status = GOT_LEN1;
    break;
  case GOT_LEN1:
    gps_skytraq.len += c;
    gps_skytraq.status = GOT_LEN2;
    if (gps_skytraq.len > GPS_SKYTRAQ_MAX_PAYLOAD) {
      gps_skytraq.error_last = GPS_SKYTRAQ_ERR_MSG_TOO_LONG;
      goto error;
    }
    break;
  case GOT_LEN2:
    gps_skytraq.msg_id = c;
    gps_skytraq.msg_idx = 0;
    gps_skytraq.checksum = c;
    gps_skytraq.status = GOT_ID;
    break;
  case GOT_ID:
    gps_skytraq.msg_buf[gps_skytraq.msg_idx] = c;
    gps_skytraq.msg_idx++;
    if (gps_skytraq.msg_idx >= gps_skytraq.len-1) {
      gps_skytraq.status = GOT_PAYLOAD;
    }
    break;
  case GOT_PAYLOAD:
    if (c != gps_skytraq.checksum) {
      gps_skytraq.error_last = GPS_SKYTRAQ_ERR_CHECKSUM;
      goto error;
    }
    gps_skytraq.status = GOT_CHECKSUM;
    break;
  case GOT_CHECKSUM:
    if (c != SKYTRAQ_SYNC3) {
      gps_skytraq.error_last = GPS_SKYTRAQ_ERR_OUT_OF_SYNC;
      goto error;
    }
    gps_skytraq.status = GOT_SYNC3;
    break;
  case GOT_SYNC3:
    gps_skytraq.msg_available = TRUE;
    goto restart;
  default:
    gps_skytraq.error_last = GPS_SKYTRAQ_ERR_UNEXPECTED;
    goto error;
  }
  return;
 error:
  gps_skytraq.error_cnt++;
 restart:
  gps_skytraq.status = UNINIT;
  return;
}
