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

#if GPS_USE_DATALINK_SMALL
#ifndef GPS_LOCAL_ECEF_ORIGIN_X
#error Local x coordinate in ECEF of the remote GPS required
#endif

#ifndef GPS_LOCAL_ECEF_ORIGIN_Y
#error Local y coordinate in ECEF of the remote GPS required
#endif

#ifndef GPS_LOCAL_ECEF_ORIGIN_Z
#error Local z coordinate in ECEF of the remote GPS required
#endif

struct EcefCoor_i tracking_ecef;

struct LtpDef_i tracking_ltp;

struct EnuCoor_i enu_pos, enu_speed;

struct EcefCoor_i ecef_pos, ecef_vel;

struct LlaCoor_i lla_pos;
#endif

bool_t gps_available;   ///< Is set to TRUE when a new REMOTE_GPS packet is received and parsed

/** GPS initialization */
void gps_impl_init(void)
{
  gps.fix = GPS_FIX_NONE;
  gps_available = FALSE;
  gps.gspeed = 700; // To enable course setting
  gps.cacc = 0; // To enable course setting

#if GPS_USE_DATALINK_SMALL
  tracking_ecef.x = GPS_LOCAL_ECEF_ORIGIN_X;
  tracking_ecef.y = GPS_LOCAL_ECEF_ORIGIN_Y;
  tracking_ecef.z = GPS_LOCAL_ECEF_ORIGIN_Z;

  ltp_def_from_ecef_i(&tracking_ltp, &tracking_ecef);
#endif
}

#ifdef GPS_USE_DATALINK_SMALL
// Parse the REMOTE_GPS_SMALL datalink packet
void parse_gps_datalink_small(uint8_t num_sv, uint32_t pos_xyz, uint32_t speed_xy)
{

  // Position in ENU coordinates
  enu_pos.x = (int32_t)((pos_xyz >> 22) & 0x3FF); // bits 31-22 x position in cm
  if (enu_pos.x & 0x200) {
    enu_pos.x |= 0xFFFFFC00;  // fix for twos complements
  }
  enu_pos.y = (int32_t)((pos_xyz >> 12) & 0x3FF); // bits 21-12 y position in cm
  if (enu_pos.y & 0x200) {
    enu_pos.y |= 0xFFFFFC00;  // fix for twos complements
  }
  enu_pos.z = (int32_t)(pos_xyz >> 2 & 0x3FF); // bits 11-2 z position in cm
  // bits 1 and 0 are free

  // printf("ENU Pos: %u (%d, %d, %d)\n", pos_xyz, enu_pos.x, enu_pos.y, enu_pos.z);

  // Convert the ENU coordinates to ECEF
  ecef_of_enu_point_i(&ecef_pos, &tracking_ltp, &enu_pos);
  gps.ecef_pos = ecef_pos;

  lla_of_ecef_i(&lla_pos, &ecef_pos);
  gps.lla_pos = lla_pos;

  enu_speed.x = (int32_t)((speed_xy >> 22) & 0x3FF); // bits 31-22 speed x in cm/s
  if (enu_speed.x & 0x200) {
    enu_speed.x |= 0xFFFFFC00;  // fix for twos complements
  }
  enu_speed.y = (int32_t)((speed_xy >> 12) & 0x3FF); // bits 21-12 speed y in cm/s
  if (enu_speed.y & 0x200) {
    enu_speed.y |= 0xFFFFFC00;  // fix for twos complements
  }
  enu_speed.z = 0;

  // printf("ENU Speed: %u (%d, %d, %d)\n", speed_xy, enu_speed.x, enu_speed.y, enu_speed.z);

  ecef_of_enu_vect_i(&gps.ecef_vel , &tracking_ltp , &enu_speed);

  gps.hmsl = tracking_ltp.hmsl + enu_pos.z * 10; // TODO: try to compensate for the loss in accuracy

  gps.course = (int32_t)((speed_xy >> 2) & 0x3FF); // bits 11-2 heading in rad*1e2
  if (gps.course & 0x200) {
    gps.course |= 0xFFFFFC00;  // fix for twos complements
  }

  // printf("Heading: %d\n", gps.course);

  gps.course *= 1e5;
  gps.num_sv = num_sv;
  gps.tow = 0; // set time-of-weak to 0
  gps.fix = GPS_FIX_3D; // set 3D fix to true
  gps_available = TRUE; // set GPS available to true

#if GPS_USE_LATLONG
  // Computes from (lat, long) in the referenced UTM zone
  struct LlaCoor_f lla_f;
  LLA_FLOAT_OF_BFP(lla_f, gps.lla_pos);
  struct UtmCoor_f utm_f;
  utm_f.zone = nav_utm_zone0;
  // convert to utm
  utm_of_lla_f(&utm_f, &lla_f);
  // copy results of utm conversion
  gps.utm_pos.east = utm_f.east * 100;
  gps.utm_pos.north = utm_f.north * 100;
  gps.utm_pos.alt = gps.lla_pos.alt;
  gps.utm_pos.zone = nav_utm_zone0;
#endif

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
#endif

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

#if GPS_USE_LATLONG
  // Computes from (lat, long) in the referenced UTM zone
  struct LlaCoor_f lla_f;
  LLA_FLOAT_OF_BFP(lla_f, gps.lla_pos);
  struct UtmCoor_f utm_f;
  utm_f.zone = nav_utm_zone0;
  // convert to utm
  utm_of_lla_f(&utm_f, &lla_f);
  // copy results of utm conversion
  gps.utm_pos.east = utm_f.east * 100;
  gps.utm_pos.north = utm_f.north * 100;
  gps.utm_pos.alt = gps.lla_pos.alt;
  gps.utm_pos.zone = nav_utm_zone0;
#endif

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

