/*
 * Copyright (C) 2008-2011 The Paparazzi Team
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

#include "subsystems/gps/gps_sim_nps.h"
#include "subsystems/gps.h"
#include "subsystems/abi.h"
#include "nps_sensors.h"
#include "nps_fdm.h"

struct GpsState gps_nps;
bool gps_has_fix;

void gps_feed_value(void)
{
  // FIXME, set proper time instead of hardcoded to May 2014
  gps_nps.week = 1794;
  gps_nps.tow = fdm.time * 1000;

  gps_nps.ecef_pos.x = sensors.gps.ecef_pos.x * 100.;
  gps_nps.ecef_pos.y = sensors.gps.ecef_pos.y * 100.;
  gps_nps.ecef_pos.z = sensors.gps.ecef_pos.z * 100.;
  SetBit(gps_nps.valid_fields, GPS_VALID_POS_ECEF_BIT);
  gps_nps.ecef_vel.x = sensors.gps.ecef_vel.x * 100.;
  gps_nps.ecef_vel.y = sensors.gps.ecef_vel.y * 100.;
  gps_nps.ecef_vel.z = sensors.gps.ecef_vel.z * 100.;
  SetBit(gps_nps.valid_fields, GPS_VALID_VEL_ECEF_BIT);

  //ecef pos seems to be based on geocentric model, hence we get a very high alt when converted to lla
  gps_nps.lla_pos.lat = DegOfRad(sensors.gps.lla_pos.lat) * 1e7;
  gps_nps.lla_pos.lon = DegOfRad(sensors.gps.lla_pos.lon) * 1e7;
  gps_nps.lla_pos.alt = sensors.gps.lla_pos.alt * 1000.;
  SetBit(gps_nps.valid_fields, GPS_VALID_POS_LLA_BIT);

  gps_nps.hmsl        = sensors.gps.hmsl * 1000.;
  SetBit(gps_nps.valid_fields, GPS_VALID_HMSL_BIT);

  /* calc NED speed from ECEF */
  struct LtpDef_d ref_ltp;
  ltp_def_from_ecef_d(&ref_ltp, &sensors.gps.ecef_pos);
  struct NedCoor_d ned_vel_d;
  ned_of_ecef_vect_d(&ned_vel_d, &ref_ltp, &sensors.gps.ecef_vel);
  gps_nps.ned_vel.x = ned_vel_d.x * 100;
  gps_nps.ned_vel.y = ned_vel_d.y * 100;
  gps_nps.ned_vel.z = ned_vel_d.z * 100;
  SetBit(gps_nps.valid_fields, GPS_VALID_VEL_NED_BIT);

  /* horizontal and 3d ground speed in cm/s */
  gps_nps.gspeed = sqrt(ned_vel_d.x * ned_vel_d.x + ned_vel_d.y * ned_vel_d.y) * 100;
  gps_nps.speed_3d = sqrt(ned_vel_d.x * ned_vel_d.x + ned_vel_d.y * ned_vel_d.y + ned_vel_d.z * ned_vel_d.z) * 100;

#if PRIMARY_GPS == GPS_DATALINK
  /* vehicle heading in radians * 1e7 */
  gps_nps.course = fdm.ltp_to_body_eulers.psi * 1e7;
#else
  /* ground course in radians * 1e7 */
  gps_nps.course = atan2(ned_vel_d.y, ned_vel_d.x) * 1e7;
#endif
  SetBit(gps_nps.valid_fields, GPS_VALID_COURSE_BIT);

  gps_nps.pacc = 650;
  gps_nps.hacc = 450;
  gps_nps.vacc = 200;
  gps_nps.sacc = 100;
  gps_nps.pdop = 650; 

  if (gps_has_fix) {
    gps_nps.num_sv = 11;
    gps_nps.fix = GPS_FIX_3D;
  } else {
    gps_nps.num_sv = 1;
    gps_nps.fix = GPS_FIX_NONE;
  }

  // publish gps data
  uint32_t now_ts = get_sys_time_usec();
  gps_nps.last_msg_ticks = sys_time.nb_sec_rem;
  gps_nps.last_msg_time = sys_time.nb_sec;
  if (gps_nps.fix == GPS_FIX_3D) {
    gps_nps.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps_nps.last_3dfix_time = sys_time.nb_sec;
  }
  AbiSendMsgGPS(GPS_SIM_ID, now_ts, &gps_nps);
}

void gps_nps_init(void)
{
  gps_has_fix = true;
}
