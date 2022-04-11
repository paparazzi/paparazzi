
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

#include "modules/gps/gps.h"
#include "modules/core/abi.h"
#include "modules/imu/imu.h"
#include "modules/datalink/datalink.h"
#include "modules/datalink/downlink.h"

#ifndef EXTERNAL_POSE_SMALL_POS_RES
#define EXTERNAL_POSE_SMALL_POS_RES 1.0
#endif

#ifndef EXTERNAL_POSE_SMALL_SPEED_RES
#define EXTERNAL_POSE_SMALL_SPEED_RES 1.0
#endif

struct GpsState gps_datalink;
static struct LtpDef_i ltp_def;

/** GPS initialization */
void gps_datalink_init(void)
{
  gps_datalink.fix = GPS_FIX_NONE;
  gps_datalink.pdop = 10;
  gps_datalink.sacc = 5;
  gps_datalink.pacc = 1;
  gps_datalink.cacc = 1;

  gps_datalink.comp_id = GPS_DATALINK_ID;

  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  ltp_def_from_lla_i(&ltp_def, &llh_nav0);
}

/** Publish the GPS data */
static void gps_datalink_publish(uint32_t tow, struct EnuCoor_f *enu_pos, struct EnuCoor_f *enu_speed, float course)
{
  struct EnuCoor_i enu_pos_i, enu_speed_i;

  enu_pos_i.x = (int32_t)CM_OF_M(enu_pos->x);
  enu_pos_i.y = (int32_t)CM_OF_M(enu_pos->y);
  enu_pos_i.z = (int32_t)CM_OF_M(enu_pos->z);

  // Convert the ENU coordinates to ECEF
  ecef_of_enu_point_i(&gps_datalink.ecef_pos, &ltp_def, &enu_pos_i);
  SetBit(gps_datalink.valid_fields, GPS_VALID_POS_ECEF_BIT);

  lla_of_ecef_i(&gps_datalink.lla_pos, &gps_datalink.ecef_pos);
  SetBit(gps_datalink.valid_fields, GPS_VALID_POS_LLA_BIT);

  enu_speed_i.x = (int32_t)CM_OF_M(enu_speed->x);
  enu_speed_i.y = (int32_t)CM_OF_M(enu_speed->y);
  enu_speed_i.z = (int32_t)CM_OF_M(enu_speed->z);

  VECT3_NED_OF_ENU(gps_datalink.ned_vel, enu_speed_i);
  SetBit(gps_datalink.valid_fields, GPS_VALID_VEL_NED_BIT);

  ecef_of_enu_vect_i(&gps_datalink.ecef_vel , &ltp_def , &enu_speed_i);
  SetBit(gps_datalink.valid_fields, GPS_VALID_VEL_ECEF_BIT);

  gps_datalink.gspeed = (int16_t)FLOAT_VECT2_NORM(enu_speed_i);
  gps_datalink.speed_3d = (int16_t)FLOAT_VECT3_NORM(enu_speed_i);

  gps_datalink.hmsl = ltp_def.hmsl + enu_pos_i.z * 10;
  SetBit(gps_datalink.valid_fields, GPS_VALID_HMSL_BIT);

  gps_datalink.course = (int32_t)(course*1e7);
  SetBit(gps_datalink.valid_fields, GPS_VALID_COURSE_BIT);

  gps_datalink.num_sv = 7;
  gps_datalink.tow = tow;
  gps_datalink.fix = GPS_FIX_3D; // set 3D fix to true

  // set gps msg time
  gps_datalink.last_msg_ticks = sys_time.nb_sec_rem;
  gps_datalink.last_msg_time = sys_time.nb_sec;

  gps_datalink.last_3dfix_ticks = sys_time.nb_sec_rem;
  gps_datalink.last_3dfix_time = sys_time.nb_sec;

  uint32_t now_ts = get_sys_time_usec();

  // Publish GPS data
  AbiSendMsgGPS(GPS_DATALINK_ID, now_ts, &gps_datalink);
}

/** Parse the full EXTERNAL_POSE message and publish as GPS through ABI */
void gps_datalink_parse_EXTERNAL_POSE(uint8_t *buf)
{
  if (DL_EXTERNAL_POSE_ac_id(buf) != AC_ID) { return; } // not for this aircraft

  uint32_t tow = DL_EXTERNAL_POSE_timestamp(buf);

  struct EnuCoor_f enu_pos, enu_speed;
  enu_pos.x = DL_EXTERNAL_POSE_enu_x(buf);
  enu_pos.y = DL_EXTERNAL_POSE_enu_y(buf);
  enu_pos.z = DL_EXTERNAL_POSE_enu_z(buf);
  enu_speed.x = DL_EXTERNAL_POSE_enu_xd(buf);
  enu_speed.y = DL_EXTERNAL_POSE_enu_yd(buf);
  enu_speed.z = DL_EXTERNAL_POSE_enu_zd(buf);

  struct FloatQuat body_q; // Converted to NED for heading calculation
  body_q.qi = DL_EXTERNAL_POSE_body_qi(buf);
  body_q.qx = DL_EXTERNAL_POSE_body_qy(buf);
  body_q.qy = DL_EXTERNAL_POSE_body_qx(buf);
  body_q.qz = -DL_EXTERNAL_POSE_body_qz(buf);

  struct FloatEulers body_e;
  float_eulers_of_quat(&body_e, &body_q);
  float heading = body_e.psi;

  gps_datalink_publish(tow, &enu_pos, &enu_speed, heading);
}

/** Parse the EXTERNAL_POSE_SMALL message and publish as GPS through ABI */
void gps_datalink_parse_EXTERNAL_POSE_SMALL(uint8_t *buf)
{
  if (DL_EXTERNAL_POSE_SMALL_ac_id(buf) != AC_ID) { return; } // not for this aircraft

  uint32_t tow = DL_EXTERNAL_POSE_SMALL_timestamp(buf);

  struct EnuCoor_i enu_pos_cm, enu_speed_cm;
  struct EnuCoor_f enu_pos, enu_speed;

  /* Convert the 32 bit xyz position to cm seperated */
  uint32_t enu_pos_u = DL_EXTERNAL_POSE_SMALL_enu_pos(buf);
  enu_pos_cm.x = (int32_t)((enu_pos_u >> 21) & 0x7FF); // bits 31-21 x position in cm
  if (enu_pos_cm.x & 0x400) {
    enu_pos_cm.x |= 0xFFFFF800;  // sign extend for twos complements
  }
  enu_pos_cm.y = (int32_t)((enu_pos_u >> 10) & 0x7FF); // bits 20-10 y position in cm
  if (enu_pos_cm.y & 0x400) {
    enu_pos_cm.y |= 0xFFFFF800;  // sign extend for twos complements
  }
  enu_pos_cm.z = (int32_t)(enu_pos_u & 0x3FF); // bits 9-0 z position in cm

  /* Convert the cm position with a resolution to meters */
  enu_pos.x = enu_pos_cm.x / 100.0f * EXTERNAL_POSE_SMALL_POS_RES;
  enu_pos.y = enu_pos_cm.y / 100.0f * EXTERNAL_POSE_SMALL_POS_RES;
  enu_pos.z = enu_pos_cm.z / 100.0f * EXTERNAL_POSE_SMALL_POS_RES;

  /* Convert the 32 bit xyz speed to cm seperated */
  uint32_t enu_speed_u = DL_EXTERNAL_POSE_SMALL_enu_speed(buf);
  enu_speed_cm.x = (int32_t)((enu_speed_u >> 21) & 0x7FF); // bits 31-21 speed x in cm/s
  if (enu_speed_cm.x & 0x400) {
    enu_speed_cm.x |= 0xFFFFF800;  // sign extend for twos complements
  }
  enu_speed_cm.y = (int32_t)((enu_speed_u >> 10) & 0x7FF); // bits 20-10 speed y in cm/s
  if (enu_speed_cm.y & 0x400) {
    enu_speed_cm.y |= 0xFFFFF800;  // sign extend for twos complements
  }
  enu_speed_cm.z = (int32_t)((enu_speed_u) & 0x3FF); // bits 9-0 speed z in cm/s
  if (enu_speed_cm.z & 0x200) {
    enu_speed_cm.z |= 0xFFFFFC00;  // sign extend for twos complements
  }

  /* Convert the cm/s speed with a resolution to meters/s */
  enu_speed.x = enu_speed_cm.x / 100.0f * EXTERNAL_POSE_SMALL_SPEED_RES;
  enu_speed.y = enu_speed_cm.y / 100.0f * EXTERNAL_POSE_SMALL_SPEED_RES;
  enu_speed.z = enu_speed_cm.z / 100.0f * EXTERNAL_POSE_SMALL_SPEED_RES;

  /* Convert the heading with the 1e4 fraction to radians */
  float heading = DL_EXTERNAL_POSE_SMALL_heading(buf) / 1e4;

  gps_datalink_publish(tow, &enu_pos, &enu_speed, heading);
}

void gps_datalink_parse_REMOTE_GPS_LOCAL(uint8_t *buf)
{
  if (DL_REMOTE_GPS_LOCAL_ac_id(buf) != AC_ID) { return; } // not for this aircraft

  uint32_t tow = DL_REMOTE_GPS_LOCAL_tow(buf);

  struct EnuCoor_f enu_pos, enu_speed;
  enu_pos.x = DL_REMOTE_GPS_LOCAL_enu_x(buf);
  enu_pos.y = DL_REMOTE_GPS_LOCAL_enu_y(buf);
  enu_pos.z = DL_REMOTE_GPS_LOCAL_enu_z(buf);

  enu_speed.x = DL_REMOTE_GPS_LOCAL_enu_xd(buf);
  enu_speed.y = DL_REMOTE_GPS_LOCAL_enu_yd(buf);
  enu_speed.z = DL_REMOTE_GPS_LOCAL_enu_zd(buf);

  float course = RadOfDeg(DL_REMOTE_GPS_LOCAL_course(buf));

  gps_datalink_publish(tow, &enu_pos, &enu_speed, course);
}
