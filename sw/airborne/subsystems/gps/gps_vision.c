/*
 * Copyright (C) 2014 Freek van Tienen
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

/**
 * @file gps_datalink.c
 * @brief GPS system based on datalink
 *
 * This GPS parses the datalink REMOTE_GPS packet and sets the
 * GPS structure to the values received.
 */

#include "subsystems/gps.h"
#include "subsystems/abi.h"

// #include <stdio.h> 

#ifndef GPS_LOCAL_ORIGIN_X
#define GPS_LOCAL_ORIGIN_X 51.0
#endif

#ifndef GPS_LOCAL_ORIGIN_Y
#define GPS_LOCAL_ORIGIN_Y 4.2
#endif

#ifndef GPS_LOCAL_ORIGIN_Z
#define GPS_LOCAL_ORIGIN_Z
#endif

struct LtpDef_i tracking_ltp;

struct EnuCoor_i enu_pos, enu_speed;

struct EcefCoor_i ecef_pos, ecef_vel;

struct LlaCoor_i lla_pos;


bool_t gps_available;   ///< Is set to TRUE when a new REMOTE_GPS packet is received and parsed

/** GPS initialization */
void gps_impl_init(void)
{
  gps.fix = GPS_FIX_3D;
  gps_available = TRUE;
  gps.gspeed = 700; // To enable course setting
  gps.cacc = 0; // To enable course setting

  /*
  tracking_ltp.x = GPS_LOCAL_ORIGIN_X;
  tracking_ltp.y = GPS_LOCAL_ORIGIN_Y;
  tracking_ltp.z = GPS_LOCAL_ORIGIN_Z;

  ltp_def_from_ecef_i(&tracking_ltp, &tracking_ecef);
*/
}




/** Parse the REMOTE_GPS datalink packet */
void parse_gps_datalink(uint8_t numsv, int32_t ecef_x, int32_t ecef_y, int32_t ecef_z, int32_t lat, int32_t lon,
                        int32_t alt,
                        int32_t hmsl, int32_t ecef_xd, int32_t ecef_yd, int32_t ecef_zd, uint32_t tow, int32_t course)
{
  gps.lla_pos.lat = lat;
  gps.lla_pos.lon = lon;
  gps.lla_pos.alt = alt;
  gps.hmsl        = hmsl;

  gps.ecef_pos.x = ecef_x;
  gps.ecef_pos.y = ecef_y;
  gps.ecef_pos.z = ecef_z;

  gps.ecef_vel.x = ecef_xd;
  gps.ecef_vel.y = ecef_yd;
  gps.ecef_vel.z = ecef_zd;

  gps.course = course;
  gps.num_sv = numsv;
  gps.tow = tow;
  gps.fix = GPS_FIX_3D;
  gps_available = TRUE;

  // publish new GPS data
  uint32_t now_ts = get_sys_time_usec();
  gps.last_msg_ticks = sys_time.nb_sec_rem;
  gps.last_msg_time = sys_time.nb_sec;
  if (gps.fix == GPS_FIX_3D) {
    gps.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps.last_3dfix_time = sys_time.nb_sec;
  }
  AbiSendMsgGPS(GPS_DATALINK_ID, now_ts, &gps);
}

