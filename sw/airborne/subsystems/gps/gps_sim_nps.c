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

#include "nps_sensors.h"
#include "nps_fdm.h"

#if GPS_USE_LATLONG
/* currently needed to get nav_utm_zone0 */
#include "subsystems/navigation/common_nav.h"
#include "math/pprz_geodetic_float.h"
#endif

bool_t gps_available;
bool_t gps_has_fix;

void  gps_feed_value()
{
  // FIXME, set proper time instead of hardcoded to May 2014
  gps.week = 1794;
  gps.tow = fdm.time * 1000;

  gps.ecef_pos.x = sensors.gps.ecef_pos.x * 100.;
  gps.ecef_pos.y = sensors.gps.ecef_pos.y * 100.;
  gps.ecef_pos.z = sensors.gps.ecef_pos.z * 100.;
  gps.ecef_vel.x = sensors.gps.ecef_vel.x * 100.;
  gps.ecef_vel.y = sensors.gps.ecef_vel.y * 100.;
  gps.ecef_vel.z = sensors.gps.ecef_vel.z * 100.;
  //ecef pos seems to be based on geocentric model, hence we get a very high alt when converted to lla
  gps.lla_pos.lat = DegOfRad(sensors.gps.lla_pos.lat) * 1e7;
  gps.lla_pos.lon = DegOfRad(sensors.gps.lla_pos.lon) * 1e7;
  gps.lla_pos.alt = sensors.gps.lla_pos.alt * 1000.;
  gps.hmsl        = sensors.gps.hmsl * 1000.;

  /* calc NED speed from ECEF */
  struct LtpDef_d ref_ltp;
  ltp_def_from_ecef_d(&ref_ltp, &sensors.gps.ecef_pos);
  struct NedCoor_d ned_vel_d;
  ned_of_ecef_vect_d(&ned_vel_d, &ref_ltp, &sensors.gps.ecef_vel);
  gps.ned_vel.x = ned_vel_d.x * 100;
  gps.ned_vel.y = ned_vel_d.y * 100;
  gps.ned_vel.z = ned_vel_d.z * 100;

  /* horizontal and 3d ground speed in cm/s */
  gps.gspeed = sqrt(ned_vel_d.x * ned_vel_d.x + ned_vel_d.y * ned_vel_d.y) * 100;
  gps.speed_3d = sqrt(ned_vel_d.x * ned_vel_d.x + ned_vel_d.y * ned_vel_d.y + ned_vel_d.z * ned_vel_d.z) * 100;

  /* ground course in radians * 1e7 */
  gps.course = atan2(ned_vel_d.y, ned_vel_d.x) * 1e7;

#if GPS_USE_LATLONG
  /* Computes from (lat, long) in the referenced UTM zone */
  struct LlaCoor_f lla_f;
  LLA_FLOAT_OF_BFP(lla_f, gps.lla_pos);
  struct UtmCoor_f utm_f;
  utm_f.zone = nav_utm_zone0;
  /* convert to utm */
  utm_of_lla_f(&utm_f, &lla_f);
  /* copy results of utm conversion */
  gps.utm_pos.east = utm_f.east * 100;
  gps.utm_pos.north = utm_f.north * 100;
  gps.utm_pos.alt = gps.lla_pos.alt;
  gps.utm_pos.zone = nav_utm_zone0;
#endif

  if (gps_has_fix) {
    gps.fix = GPS_FIX_3D;
  } else {
    gps.fix = GPS_FIX_NONE;
  }
  gps_available = TRUE;
}

void gps_impl_init()
{
  gps_available = FALSE;
  gps_has_fix = TRUE;
}
