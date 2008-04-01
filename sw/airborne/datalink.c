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
#include "datalink.h"

#ifdef TRAFFIC_INFO
#include "traffic_info.h"
#endif // TRAFFIC_INFO

#include "common_nav.h"
#include "settings.h"
#include "latlong.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "uart.h"
#include "downlink.h"

#define MOfCm(_x) (((float)(_x))/100.)

#define SenderIdOfMsg(x) (x[0])
#define IdOfMsg(x) (x[1])

void dl_parse_msg(void) {
  datalink_time = 0;
  uint8_t msg_id = IdOfMsg(dl_buffer);

  if (msg_id == DL_PING) {
    DOWNLINK_SEND_PONG();
  } else
#ifdef TRAFFIC_INFO
  if (msg_id == DL_ACINFO) {
    uint8_t id = DL_ACINFO_ac_id(dl_buffer);
    float ux = MOfCm(DL_ACINFO_utm_east(dl_buffer));
    float uy = MOfCm(DL_ACINFO_utm_north(dl_buffer));
    float a = MOfCm(DL_ACINFO_alt(dl_buffer));
    float c = RadOfDeg(((float)DL_ACINFO_course(dl_buffer))/ 10.);
    float s = MOfCm(DL_ACINFO_speed(dl_buffer));
    uint32_t t = DL_ACINFO_itow(dl_buffer);
    SetAcInfo(id, ux, uy, c, a, s, t);
  } else
#endif
#ifdef NAV
  if (msg_id == DL_MOVE_WP && DL_MOVE_WP_ac_id(dl_buffer) == AC_ID) {
    uint8_t wp_id = DL_MOVE_WP_wp_id(dl_buffer);
    float a = MOfCm(DL_MOVE_WP_alt(dl_buffer));

    /* Computes from (lat, long) in the referenced UTM zone */
    float lat = RadOfDeg(DL_MOVE_WP_lat(dl_buffer));
    float lon = RadOfDeg(DL_MOVE_WP_lon(dl_buffer));
    latlong_utm_of(lat, lon, nav_utm_zone0);
    nav_move_waypoint(wp_id, latlong_utm_x, latlong_utm_y, a);

    /* Waypoint range is limited. Computes the UTM pos back from the relative
       coordinates */
    latlong_utm_x = waypoints[wp_id].x + nav_utm_east0;
    latlong_utm_y = waypoints[wp_id].y + nav_utm_north0;
    DOWNLINK_SEND_WP_MOVED(&wp_id, &latlong_utm_x, &latlong_utm_y, &a, &nav_utm_zone0);
  } else if (msg_id == DL_BLOCK && DL_BLOCK_ac_id(dl_buffer) == AC_ID) {
    nav_goto_block(DL_BLOCK_block_id(dl_buffer));
  } else
#endif /** NAV */
#ifdef WIND_INFO
    if (msg_id == DL_WIND_INFO && DL_WIND_INFO_ac_id(dl_buffer) == AC_ID) {
    wind_east = DL_WIND_INFO_east(dl_buffer);
    wind_north = DL_WIND_INFO_north(dl_buffer);
    estimator_airspeed = DL_WIND_INFO_airspeed(dl_buffer);
  } else
#endif /** WIND_INFO */
#ifdef HITL
  /** Infrared and GPS sensors are replaced by messages on the datalink */
  if (msg_id == DL_HITL_INFRARED) {
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
  } else
#endif
#ifdef DlSetting
  if (msg_id == DL_SETTING && DL_SETTING_ac_id(dl_buffer) == AC_ID) {
    uint8_t i = DL_SETTING_index(dl_buffer);
    float val = DL_SETTING_value(dl_buffer);
    DlSetting(i, val);
    DOWNLINK_SEND_DL_VALUE(&i, &val);
  } else
#endif /** Else there is no dl_settings section in the flight plan */
    { /* Last else */ }
}
