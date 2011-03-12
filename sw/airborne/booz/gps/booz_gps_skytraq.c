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

struct BoozGpsSkytraq booz_gps_skytraq;

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

//#include "my_debug_servo.h"
#include "led.h"

void booz_gps_impl_init(void) {

  booz_gps_skytraq.status = UNINIT;


  //DEBUG_SERVO1_INIT();

}


void booz_gps_skytraq_read_message(void) {

  //DEBUG_S1_ON();

  if (booz_gps_skytraq.msg_id == SKYTRAQ_ID_NAVIGATION_DATA) {
    booz_gps_state.ecef_pos.x  = SKYTRAQ_NAVIGATION_DATA_ECEFX(booz_gps_skytraq.msg_buf);
    booz_gps_state.ecef_pos.y  = SKYTRAQ_NAVIGATION_DATA_ECEFY(booz_gps_skytraq.msg_buf);
    booz_gps_state.ecef_pos.z  = SKYTRAQ_NAVIGATION_DATA_ECEFZ(booz_gps_skytraq.msg_buf);
    booz_gps_state.ecef_vel.x  = SKYTRAQ_NAVIGATION_DATA_ECEFVX(booz_gps_skytraq.msg_buf);
    booz_gps_state.ecef_vel.y  = SKYTRAQ_NAVIGATION_DATA_ECEFVY(booz_gps_skytraq.msg_buf);
    booz_gps_state.ecef_vel.z  = SKYTRAQ_NAVIGATION_DATA_ECEFVZ(booz_gps_skytraq.msg_buf);
    booz_gps_state.lla_pos.lat = SKYTRAQ_NAVIGATION_DATA_LAT(booz_gps_skytraq.msg_buf);
    booz_gps_state.lla_pos.lon = SKYTRAQ_NAVIGATION_DATA_LON(booz_gps_skytraq.msg_buf);
    booz_gps_state.lla_pos.alt = SKYTRAQ_NAVIGATION_DATA_AEL(booz_gps_skytraq.msg_buf);
    booz_gps_state.hmsl        = SKYTRAQ_NAVIGATION_DATA_ASL(booz_gps_skytraq.msg_buf);
    //   pacc;
    //   sacc;
    //     booz_gps_state.pdop       = SKYTRAQ_NAVIGATION_DATA_PDOP(booz_gps_skytraq.msg_buf);
    booz_gps_state.num_sv      = SKYTRAQ_NAVIGATION_DATA_NumSV(booz_gps_skytraq.msg_buf);
    booz_gps_state.fix         = SKYTRAQ_NAVIGATION_DATA_FixMode(booz_gps_skytraq.msg_buf);
    booz_gps_state.tow         = SKYTRAQ_NAVIGATION_DATA_TOW(booz_gps_skytraq.msg_buf);
    //DEBUG_S2_TOGGLE();

#ifdef GPS_LED
    if (booz_gps_state.fix == BOOZ2_GPS_FIX_3D) {
      LED_ON(GPS_LED);
    }
    else {
      LED_TOGGLE(GPS_LED);
    }
#endif
  }

  //DEBUG_S1_OFF();
}

void booz_gps_skytraq_parse(uint8_t c) {
  if (booz_gps_skytraq.status < GOT_PAYLOAD)
    booz_gps_skytraq.checksum ^= c;
  switch (booz_gps_skytraq.status) {
  case UNINIT:
    if (c == SKYTRAQ_SYNC1)
      booz_gps_skytraq.status = GOT_SYNC1;
    break;
  case GOT_SYNC1:
    if (c != SKYTRAQ_SYNC2) {
      booz_gps_skytraq.error_last = GPS_SKYTRAQ_ERR_OUT_OF_SYNC;
      goto error;
    }
    booz_gps_skytraq.status = GOT_SYNC2;
    break;
  case GOT_SYNC2:
    booz_gps_skytraq.len = c<<8;
    booz_gps_skytraq.status = GOT_LEN1;
    break;
  case GOT_LEN1:
    booz_gps_skytraq.len += c;
    booz_gps_skytraq.status = GOT_LEN2;
    if (booz_gps_skytraq.len > GPS_SKYTRAQ_MAX_PAYLOAD) {
      booz_gps_skytraq.error_last = GPS_SKYTRAQ_ERR_MSG_TOO_LONG;
      goto error;
    }
    break;
  case GOT_LEN2:
    booz_gps_skytraq.msg_id = c;
    booz_gps_skytraq.msg_idx = 0;
    booz_gps_skytraq.checksum = c;
    booz_gps_skytraq.status = GOT_ID;
    break;
  case GOT_ID:
    booz_gps_skytraq.msg_buf[booz_gps_skytraq.msg_idx] = c;
    booz_gps_skytraq.msg_idx++;
    if (booz_gps_skytraq.msg_idx >= booz_gps_skytraq.len-1) {
      booz_gps_skytraq.status = GOT_PAYLOAD;
    }
    break;
  case GOT_PAYLOAD:
    if (c != booz_gps_skytraq.checksum) {
      booz_gps_skytraq.error_last = GPS_SKYTRAQ_ERR_CHECKSUM;
      goto error;
    }
    booz_gps_skytraq.status = GOT_CHECKSUM;
    break;
  case GOT_CHECKSUM:
    if (c != SKYTRAQ_SYNC3) {
      booz_gps_skytraq.error_last = GPS_SKYTRAQ_ERR_OUT_OF_SYNC;
      goto error;
    }
    booz_gps_skytraq.status = GOT_SYNC3;
    break;
  case GOT_SYNC3:
    booz_gps_skytraq.msg_available = TRUE;
    goto restart;
  default:
    booz_gps_skytraq.error_last = GPS_SKYTRAQ_ERR_UNEXPECTED;
    goto error;
  }
  return;
 error:
  booz_gps_skytraq.error_cnt++;
 restart:
  booz_gps_skytraq.status = UNINIT;
  return;
}
