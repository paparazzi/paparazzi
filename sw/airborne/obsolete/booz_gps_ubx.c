/*
 * $Id$
 *
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


#include "booz_gps.h"

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

struct BoosGpsUbx booz_gps_ubx;

void booz_gps_impl_init(void) {
   booz_gps_ubx.status = UNINIT;
   booz_gps_ubx.msg_available = FALSE;
   booz_gps_ubx.error_cnt = 0;
   booz_gps_ubx.error_last = GPS_UBX_ERR_NONE;
}


void booz_gps_ubx_read_message(void) {

  if (booz_gps_ubx.msg_class == UBX_NAV_ID) {
    if (booz_gps_ubx.msg_id == UBX_NAV_SOL_ID) {
      booz_gps_state.fix        = UBX_NAV_SOL_GPSfix(booz_gps_ubx.msg_buf);
      booz_gps_state.ecef_pos.x = UBX_NAV_SOL_ECEF_X(booz_gps_ubx.msg_buf);
      booz_gps_state.ecef_pos.y = UBX_NAV_SOL_ECEF_Y(booz_gps_ubx.msg_buf);
      booz_gps_state.ecef_pos.z = UBX_NAV_SOL_ECEF_Z(booz_gps_ubx.msg_buf);
      booz_gps_state.pacc       = UBX_NAV_SOL_Pacc(booz_gps_ubx.msg_buf);
      booz_gps_state.ecef_vel.x = UBX_NAV_SOL_ECEFVX(booz_gps_ubx.msg_buf);
      booz_gps_state.ecef_vel.y = UBX_NAV_SOL_ECEFVY(booz_gps_ubx.msg_buf);
      booz_gps_state.ecef_vel.z = UBX_NAV_SOL_ECEFVZ(booz_gps_ubx.msg_buf);
      booz_gps_state.sacc       = UBX_NAV_SOL_Sacc(booz_gps_ubx.msg_buf);
      booz_gps_state.pdop       = UBX_NAV_SOL_PDOP(booz_gps_ubx.msg_buf);
      booz_gps_state.num_sv     = UBX_NAV_SOL_numSV(booz_gps_ubx.msg_buf);
#ifdef GPS_LED
      if (booz_gps_state.fix == BOOZ2_GPS_FIX_3D) {
        LED_ON(GPS_LED);
      }
      else {
        LED_TOGGLE(GPS_LED);
      }
#endif
    } else if (booz_gps_ubx.msg_id == UBX_NAV_POSLLH_ID) {
      booz_gps_state.lla_pos.lat = UBX_NAV_POSLLH_LAT(booz_gps_ubx.msg_buf);
      booz_gps_state.lla_pos.lon = UBX_NAV_POSLLH_LON(booz_gps_ubx.msg_buf);
      booz_gps_state.lla_pos.alt = UBX_NAV_POSLLH_HEIGHT(booz_gps_ubx.msg_buf) / 10;
      booz_gps_state.hmsl        = UBX_NAV_POSLLH_HMSL(booz_gps_ubx.msg_buf) / 10;
    }
  }
}



/* UBX parsing */

void booz_gps_ubx_parse( uint8_t c ) {
  if (booz_gps_ubx.status < GOT_PAYLOAD) {
    booz_gps_ubx.ck_a += c;
    booz_gps_ubx.ck_b += booz_gps_ubx.ck_a;
  }
  switch (booz_gps_ubx.status) {
  case UNINIT:
    if (c == UBX_SYNC1)
      booz_gps_ubx.status++;
    break;
  case GOT_SYNC1:
    if (c != UBX_SYNC2) {
      booz_gps_ubx.error_last = GPS_UBX_ERR_OUT_OF_SYNC;
      goto error;
    }
    booz_gps_ubx.ck_a = 0;
    booz_gps_ubx.ck_b = 0;
    booz_gps_ubx.status++;
    break;
  case GOT_SYNC2:
    if (booz_gps_ubx.msg_available) {
      /* Previous message has not yet been parsed: discard this one */
      booz_gps_ubx.error_last = GPS_UBX_ERR_OVERRUN;
      goto error;
    }
    booz_gps_ubx.msg_class = c;
    booz_gps_ubx.status++;
    break;
  case GOT_CLASS:
    booz_gps_ubx.msg_id = c;
    booz_gps_ubx.status++;
    break;
  case GOT_ID:
    booz_gps_ubx.len = c;
    booz_gps_ubx.status++;
    break;
  case GOT_LEN1:
    booz_gps_ubx.len |= (c<<8);
    if (booz_gps_ubx.len > GPS_UBX_MAX_PAYLOAD) {
      booz_gps_ubx.error_last = GPS_UBX_ERR_MSG_TOO_LONG;
      goto error;
    }
    booz_gps_ubx.msg_idx = 0;
    booz_gps_ubx.status++;
    break;
  case GOT_LEN2:
    booz_gps_ubx.msg_buf[booz_gps_ubx.msg_idx] = c;
    booz_gps_ubx.msg_idx++;
    if (booz_gps_ubx.msg_idx >= booz_gps_ubx.len) {
      booz_gps_ubx.status++;
    }
    break;
  case GOT_PAYLOAD:
    if (c != booz_gps_ubx.ck_a) {
      booz_gps_ubx.error_last = GPS_UBX_ERR_CHECKSUM;
      goto error;
    }
    booz_gps_ubx.status++;
    break;
  case GOT_CHECKSUM1:
    if (c != booz_gps_ubx.ck_b) {
      booz_gps_ubx.error_last = GPS_UBX_ERR_CHECKSUM;
      goto error;
    }
    booz_gps_ubx.msg_available = TRUE;
    goto restart;
    break;
  default:
    booz_gps_ubx.error_last = GPS_UBX_ERR_UNEXPECTED;
    goto error;
  }
  return;
 error:
  booz_gps_ubx.error_cnt++;
 restart:
  booz_gps_ubx.status = UNINIT;
  return;
}
