/*
 * $Id$
 *  
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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
/** \file datalink.c
 *  \brief Handling of messages coming from ground and other A/Cs
 *
 */
#define DATALINK_C

#include <inttypes.h>
#include <string.h>
#include "traffic_info.h"
#include "nav.h"
#include "datalink.h"
#include "flight_plan.h"
#include "autopilot.h"
#include "ap_downlink.h"
#include "messages.h"

#include "estimator.h"
#include "fw_v_ctl.h"
#include "fw_h_ctl.h"
#include "cam.h"
#include "infrared.h"
#include "gps.h"
#include "uart.h"
#include "gpio.h"

#include "control_grz.h"
#include "settings.h"
#include "latlong.h"

#define MOfCm(_x) (((float)_x)/100.)

#define SenderIdOfMsg(x) (x[0])
#define IdOfMsg(x) (x[1])

void dl_parse_msg(void) {
  datalink_time = 0;
  uint8_t msg_id = IdOfMsg(dl_buffer);
#ifdef NAV
  if (msg_id == DL_ACINFO) {
    uint8_t id = DL_ACINFO_ac_id(dl_buffer);
    float ux = MOfCm(DL_ACINFO_utm_east(dl_buffer));
    float uy = MOfCm(DL_ACINFO_utm_north(dl_buffer));
    float a = MOfCm(DL_ACINFO_alt(dl_buffer));
    float c = RadOfDeg(((float)DL_ACINFO_course(dl_buffer))/ 10.);
    float s = MOfCm(DL_ACINFO_speed(dl_buffer));
    SetAcInfo(id, ux, uy, c, a, s);
  } else if (msg_id == DL_MOVE_WP) {
    uint8_t wp_id = DL_MOVE_WP_wp_id(dl_buffer);
    float a = MOfCm(DL_MOVE_WP_alt(dl_buffer));

    /* Computes from (lat, long) in the referenced UTM zone */
    float lat = MOfCm(DL_MOVE_WP_lat(dl_buffer));
    float lon = MOfCm(DL_MOVE_WP_lon(dl_buffer));
    latlong_utm_of(RadOfDeg(lat), RadOfDeg(lon), nav_utm_zone0);

    MoveWaypoint(wp_id, latlong_utm_x, latlong_utm_y, a);
    DOWNLINK_SEND_WP_MOVED(&wp_id, &latlong_utm_x, &latlong_utm_y, &a, &nav_utm_zone0);
  } else if (msg_id == DL_BLOCK) {
    nav_goto_block(DL_BLOCK_block_id(dl_buffer));
  } else if (msg_id == DL_WIND_INFO) {
    wind_east = DL_WIND_INFO_east(dl_buffer);
    wind_north = DL_WIND_INFO_north(dl_buffer);
  } else
#endif /** NAV */
#ifdef AP
    if (msg_id == DL_TELEMETRY_MODE) {
    telemetry_mode_Ap = DL_TELEMETRY_MODE_mode(dl_buffer);
#endif /** AP */
  } 
#ifdef HITL
  /** Infrared and GPS sensors are replaced by messages on the datalink */
  else if (msg_id == DL_HITL_INFRARED) {
    /** This code simulates infrared.c:ir_update() */
    ir_roll = DL_HITL_INFRARED_roll(dl_buffer);
    ir_pitch = DL_HITL_INFRARED_pitch(dl_buffer);
  } else if (msg_id == DL_HITL_UBX) {
    /** This code simulates gps_ubx.c:parse_ubx() */
    if (gps_msg_received) {
      gps_nb_ovrn++;
    } else {
      ubx_class = DL_HITL_UBX_class(dl_buffer);
      ubx_id = DL_HITL_UBX_id(dl_buffer);
      uint8_t l = DL_HITL_UBX_ubx_payload_length(dl_buffer);
      uint8_t *ubx_payload = DL_HITL_UBX_ubx_payload(dl_buffer);
      uint8_t i;
      for(i=0; i<l; i++) {
	ubx_msg_buf[i] = ubx_payload[i];
      }
      gps_msg_received = TRUE;
    }
  }
#endif
#ifdef DlSetting
  else if (msg_id == DL_SETTING) {
    uint8_t i = DL_SETTING_index(dl_buffer);
    float val = DL_SETTING_value(dl_buffer);
    DlSetting(i, val);
    DOWNLINK_SEND_DL_VALUE(&i, &val);
  }
#endif /** Else there is no dl_settings section in the flight plan */
}
