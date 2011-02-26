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

/* parser status */
#define UNINIT        0
#define GOT_SYNC1     1
#define GOT_SYNC2     2
#define GOT_CLASS     3
#define GOT_ID        4
#define GOT_LEN1      5
#define GOT_LEN2      6
#define GOT_PAYLOAD   7
#define GOT_CHECKSUM1 8

/* last error type */
#define GPS_UBX_ERR_NONE         0
#define GPS_UBX_ERR_OVERRUN      1
#define GPS_UBX_ERR_MSG_TOO_LONG 2
#define GPS_UBX_ERR_CHECKSUM     3
#define GPS_UBX_ERR_UNEXPECTED   4
#define GPS_UBX_ERR_OUT_OF_SYNC  5

#define UTM_HEM_NORTH 0
#define UTM_HEM_SOUTH 1

struct GpsUbx gps_ubx;

void gps_impl_init(void) {
   gps_ubx.status = UNINIT;
   gps_ubx.msg_available = FALSE;
   gps_ubx.error_cnt = 0;
   gps_ubx.error_last = GPS_UBX_ERR_NONE;
}


void gps_ubx_read_message(void) {

  if (gps_ubx.msg_class == UBX_NAV_ID) {
    if (gps_ubx.msg_id == UBX_NAV_SOL_ID) {
#ifdef GPS_TIMESTAMP
      /* get hardware clock ticks */
      gps.t0 = T0TC;
      gps.t0_tow        = UBX_NAV_SOL_ITOW(gps_ubx.msg_buf);
      gps.t0_tow_frac   = UBX_NAV_SOL_Frac(gps_ubx.msg_buf);
#endif
      gps.tow        = UBX_NAV_SOL_ITOW(gps_ubx.msg_buf);
      gps.week       = UBX_NAV_SOL_week(gps_ubx.msg_buf);
      gps.fix        = UBX_NAV_SOL_GPSfix(gps_ubx.msg_buf);
      gps.ecef_pos.x = UBX_NAV_SOL_ECEF_X(gps_ubx.msg_buf);
      gps.ecef_pos.y = UBX_NAV_SOL_ECEF_Y(gps_ubx.msg_buf);
      gps.ecef_pos.z = UBX_NAV_SOL_ECEF_Z(gps_ubx.msg_buf);
      gps.pacc       = UBX_NAV_SOL_Pacc(gps_ubx.msg_buf);
      gps.ecef_vel.x = UBX_NAV_SOL_ECEFVX(gps_ubx.msg_buf);
      gps.ecef_vel.y = UBX_NAV_SOL_ECEFVY(gps_ubx.msg_buf);
      gps.ecef_vel.z = UBX_NAV_SOL_ECEFVZ(gps_ubx.msg_buf);
      gps.sacc       = UBX_NAV_SOL_Sacc(gps_ubx.msg_buf);
      gps.pdop       = UBX_NAV_SOL_PDOP(gps_ubx.msg_buf);
      gps.num_sv     = UBX_NAV_SOL_numSV(gps_ubx.msg_buf);
#ifdef GPS_LED
      if (gps.fix == GPS_FIX_3D) {
        LED_ON(GPS_LED);
      }
      else {
        LED_TOGGLE(GPS_LED);
      }
#endif
    } else if (gps_ubx.msg_id == UBX_NAV_POSLLH_ID) {
      gps.lla_pos.lat = RadOfDeg(UBX_NAV_POSLLH_LAT(gps_ubx.msg_buf));
      gps.lla_pos.lon = RadOfDeg(UBX_NAV_POSLLH_LON(gps_ubx.msg_buf));
      gps.lla_pos.alt = UBX_NAV_POSLLH_HEIGHT(gps_ubx.msg_buf);
      gps.hmsl        = UBX_NAV_POSLLH_HMSL(gps_ubx.msg_buf);
    }
    else if (gps_ubx.msg_id == UBX_NAV_POSUTM_ID) {
      gps.utm_pos.east = UBX_NAV_POSUTM_EAST(gps_ubx.msg_buf);
      gps.utm_pos.north = UBX_NAV_POSUTM_NORTH(gps_ubx.msg_buf);
      uint8_t hem = UBX_NAV_POSUTM_HEM(gps_ubx.msg_buf);
      if (hem == UTM_HEM_SOUTH)
        gps.utm_pos.north -= 1000000000; /* Subtract false northing: -10000km */
      gps.utm_pos.alt = UBX_NAV_POSUTM_ALT(gps_ubx.msg_buf)*10;
      gps.utm_pos.zone = UBX_NAV_POSUTM_ZONE(gps_ubx.msg_buf);
    }
    else if (gps_ubx.msg_id == UBX_NAV_VELNED_ID) {
      gps.speed_3d = UBX_NAV_VELNED_Speed(gps_ubx.msg_buf);
      gps.gspeed = UBX_NAV_VELNED_GSpeed(gps_ubx.msg_buf);
      gps.ned_vel.x = UBX_NAV_VELNED_VEL_N(gps_ubx.msg_buf);
      gps.ned_vel.y = UBX_NAV_VELNED_VEL_E(gps_ubx.msg_buf);
      gps.ned_vel.z = UBX_NAV_VELNED_VEL_D(gps_ubx.msg_buf);
      gps.course = RadOfDeg(UBX_NAV_VELNED_Heading(gps_ubx.msg_buf)*100);
      gps.tow = UBX_NAV_VELNED_ITOW(gps_ubx.msg_buf);
    }
    else if (gps_ubx.msg_id == UBX_NAV_SVINFO_ID) {
      gps_ubx.nb_channels = Min(UBX_NAV_SVINFO_NCH(gps_ubx.msg_buf), GPS_NB_CHANNELS);
      uint8_t i;
      for(i = 0; i < gps_ubx.nb_channels; i++) {
        gps.svinfos[i].svid = UBX_NAV_SVINFO_SVID(gps_ubx.msg_buf, i);
        gps.svinfos[i].flags = UBX_NAV_SVINFO_Flags(gps_ubx.msg_buf, i);
        gps.svinfos[i].qi = UBX_NAV_SVINFO_QI(gps_ubx.msg_buf, i);
        gps.svinfos[i].cno = UBX_NAV_SVINFO_CNO(gps_ubx.msg_buf, i);
        gps.svinfos[i].elev = UBX_NAV_SVINFO_Elev(gps_ubx.msg_buf, i);
        gps.svinfos[i].azim = UBX_NAV_SVINFO_Azim(gps_ubx.msg_buf, i);
      }
    }
  }
}


/* UBX parsing */
void gps_ubx_parse( uint8_t c ) {
  if (gps_ubx.status < GOT_PAYLOAD) {
    gps_ubx.ck_a += c;
    gps_ubx.ck_b += gps_ubx.ck_a;
  }
  switch (gps_ubx.status) {
  case UNINIT:
    if (c == UBX_SYNC1)
      gps_ubx.status++;
    break;
  case GOT_SYNC1:
    if (c != UBX_SYNC2) {
      gps_ubx.error_last = GPS_UBX_ERR_OUT_OF_SYNC;
      goto error;
    }
    gps_ubx.ck_a = 0;
    gps_ubx.ck_b = 0;
    gps_ubx.status++;
    break;
  case GOT_SYNC2:
    if (gps_ubx.msg_available) {
      /* Previous message has not yet been parsed: discard this one */
      gps_ubx.error_last = GPS_UBX_ERR_OVERRUN;
      goto error;
    }
    gps_ubx.msg_class = c;
    gps_ubx.status++;
    break;
  case GOT_CLASS:
    gps_ubx.msg_id = c;
    gps_ubx.status++;
    break;
  case GOT_ID:
    gps_ubx.len = c;
    gps_ubx.status++;
    break;
  case GOT_LEN1:
    gps_ubx.len |= (c<<8);
    if (gps_ubx.len > GPS_UBX_MAX_PAYLOAD) {
      gps_ubx.error_last = GPS_UBX_ERR_MSG_TOO_LONG;
      goto error;
    }
    gps_ubx.msg_idx = 0;
    gps_ubx.status++;
    break;
  case GOT_LEN2:
    gps_ubx.msg_buf[gps_ubx.msg_idx] = c;
    gps_ubx.msg_idx++;
    if (gps_ubx.msg_idx >= gps_ubx.len) {
      gps_ubx.status++;
    }
    break;
  case GOT_PAYLOAD:
    if (c != gps_ubx.ck_a) {
      gps_ubx.error_last = GPS_UBX_ERR_CHECKSUM;
      goto error;
    }
    gps_ubx.status++;
    break;
  case GOT_CHECKSUM1:
    if (c != gps_ubx.ck_b) {
      gps_ubx.error_last = GPS_UBX_ERR_CHECKSUM;
      goto error;
    }
    gps_ubx.msg_available = TRUE;
    goto restart;
    break;
  default:
    gps_ubx.error_last = GPS_UBX_ERR_UNEXPECTED;
    goto error;
  }
  return;
 error:
  gps_ubx.error_cnt++;
 restart:
  gps_ubx.status = UNINIT;
  return;
}

