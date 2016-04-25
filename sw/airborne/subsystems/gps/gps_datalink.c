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

#include "generated/flight_plan.h"        // reference lla NAV_XXX0

#include "subsystems/gps.h"
#include "subsystems/abi.h"

struct LtpDef_i ltp_def;

struct GpsState gps_datalink;

/** GPS initialization */
void gps_datalink_init(void)
{
  gps_datalink.fix = GPS_FIX_NONE;
  gps_datalink.pdop = 0;
  gps_datalink.sacc = 0;
  gps_datalink.pacc = 0;
  gps_datalink.cacc = 0;

  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  ltp_def_from_lla_i(&ltp_def, &llh_nav0);
}

// Parse the REMOTE_GPS_SMALL datalink packet
void parse_gps_datalink_small(int16_t heading, uint32_t pos_xyz, uint32_t speed_xyz, uint32_t tow)
{
  struct EnuCoor_i enu_pos, enu_speed;

  // Position in ENU coordinates
  enu_pos.x = (int32_t)((pos_xyz >> 21) & 0x7FF); // bits 31-21 x position in cm
  if (enu_pos.x & 0x400) {
    enu_pos.x |= 0xFFFFF800;  // sign extend for twos complements
  }
  enu_pos.y = (int32_t)((pos_xyz >> 10) & 0x7FF); // bits 20-10 y position in cm
  if (enu_pos.y & 0x400) {
    enu_pos.y |= 0xFFFFF800;  // sign extend for twos complements
  }
  enu_pos.z = (int32_t)(pos_xyz & 0x3FF); // bits 9-0 z position in cm

  // Convert the ENU coordinates to ECEF
  ecef_of_enu_point_i(&gps_datalink.ecef_pos, &ltp_def, &enu_pos);
  SetBit(gps_datalink.valid_fields, GPS_VALID_POS_ECEF_BIT);

  lla_of_ecef_i(&gps_datalink.lla_pos, &gps_datalink.ecef_pos);
  SetBit(gps_datalink.valid_fields, GPS_VALID_POS_LLA_BIT);

  enu_speed.x = (int32_t)((speed_xyz >> 21) & 0x7FF); // bits 31-21 speed x in cm/s
  if (enu_speed.x & 0x400) {
    enu_speed.x |= 0xFFFFF800;  // sign extend for twos complements
  }
  enu_speed.y = (int32_t)((speed_xyz >> 10) & 0x7FF); // bits 20-10 speed y in cm/s
  if (enu_speed.y & 0x400) {
    enu_speed.y |= 0xFFFFF800;  // sign extend for twos complements
  }
  enu_speed.z = (int32_t)((speed_xyz) & 0x3FF); // bits 9-0 speed z in cm/s
  if (enu_speed.z & 0x200) {
    enu_speed.z |= 0xFFFFFC00;  // sign extend for twos complements
  }

  VECT3_NED_OF_ENU(gps_datalink.ned_vel, enu_speed);
  SetBit(gps_datalink.valid_fields, GPS_VALID_VEL_NED_BIT);

  ecef_of_enu_vect_i(&gps_datalink.ecef_vel , &ltp_def , &enu_speed);
  SetBit(gps_datalink.valid_fields, GPS_VALID_VEL_ECEF_BIT);

  gps_datalink.gspeed = (int16_t)FLOAT_VECT2_NORM(enu_speed);
  gps_datalink.speed_3d = (int16_t)FLOAT_VECT3_NORM(enu_speed);

  gps_datalink.hmsl = ltp_def.hmsl + enu_pos.z * 10;
  SetBit(gps_datalink.valid_fields, GPS_VALID_HMSL_BIT);

  gps_datalink.course = ((int32_t)heading) * 1e3;
  SetBit(gps_datalink.valid_fields, GPS_VALID_COURSE_BIT);

  gps_datalink.num_sv = 7;
  gps_datalink.tow = tow;
  gps_datalink.fix = GPS_FIX_3D; // set 3D fix to true

  // publish new GPS data
  uint32_t now_ts = get_sys_time_usec();
  gps_datalink.last_msg_ticks = sys_time.nb_sec_rem;
  gps_datalink.last_msg_time = sys_time.nb_sec;

  gps_datalink.last_3dfix_ticks = sys_time.nb_sec_rem;
  gps_datalink.last_3dfix_time = sys_time.nb_sec;

  AbiSendMsgGPS(GPS_DATALINK_ID, now_ts, &gps_datalink);
}

/** Parse the REMOTE_GPS datalink packet */
void parse_gps_datalink(uint8_t numsv, int32_t ecef_x, int32_t ecef_y, int32_t ecef_z, int32_t lat, int32_t lon,
                        int32_t alt,
                        int32_t hmsl, int32_t ecef_xd, int32_t ecef_yd, int32_t ecef_zd, uint32_t tow, int32_t course)
{
  gps_datalink.lla_pos.lat = lat;
  gps_datalink.lla_pos.lon = lon;
  gps_datalink.lla_pos.alt = alt;
  SetBit(gps_datalink.valid_fields, GPS_VALID_POS_LLA_BIT);

  gps_datalink.hmsl        = hmsl;
  SetBit(gps_datalink.valid_fields, GPS_VALID_HMSL_BIT);

  gps_datalink.ecef_pos.x = ecef_x;
  gps_datalink.ecef_pos.y = ecef_y;
  gps_datalink.ecef_pos.z = ecef_z;
  SetBit(gps_datalink.valid_fields, GPS_VALID_POS_ECEF_BIT);

  gps_datalink.ecef_vel.x = ecef_xd;
  gps_datalink.ecef_vel.y = ecef_yd;
  gps_datalink.ecef_vel.z = ecef_zd;
  SetBit(gps_datalink.valid_fields, GPS_VALID_VEL_ECEF_BIT);

  ned_of_ecef_vect_i(&gps_datalink.ned_vel, &ltp_def , &gps_datalink.ecef_vel);
  SetBit(gps_datalink.valid_fields, GPS_VALID_VEL_NED_BIT);

  gps_datalink.gspeed = (int16_t)FLOAT_VECT2_NORM(gps_datalink.ned_vel);
  gps_datalink.speed_3d = (int16_t)FLOAT_VECT3_NORM(gps_datalink.ned_vel);

  gps_datalink.course = course;
  SetBit(gps_datalink.valid_fields, GPS_VALID_COURSE_BIT);

  gps_datalink.num_sv = numsv;
  gps_datalink.tow = tow;
  gps_datalink.fix = GPS_FIX_3D;

  // publish new GPS data
  uint32_t now_ts = get_sys_time_usec();
  gps_datalink.last_msg_ticks = sys_time.nb_sec_rem;
  gps_datalink.last_msg_time = sys_time.nb_sec;

  gps_datalink.last_3dfix_ticks = sys_time.nb_sec_rem;
  gps_datalink.last_3dfix_time = sys_time.nb_sec;

  AbiSendMsgGPS(GPS_DATALINK_ID, now_ts, &gps_datalink);
}


/*
 * register callbacks & structs
 */
void gps_datalink_register(void)
{
  gps_register_impl(gps_datalink_init, NULL, GPS_DATALINK_ID);
}
