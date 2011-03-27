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

bool_t gps_available;

void  gps_feed_value() {
  gps.ecef_pos.x = sensors.gps.ecef_pos.x * 100.;
  gps.ecef_pos.y = sensors.gps.ecef_pos.y * 100.;
  gps.ecef_pos.z = sensors.gps.ecef_pos.z * 100.;
  gps.ecef_vel.x = sensors.gps.ecef_vel.x * 100.;
  gps.ecef_vel.y = sensors.gps.ecef_vel.y * 100.;
  gps.ecef_vel.z = sensors.gps.ecef_vel.z * 100.;
  //ecef pos seems to be based on geocentric model, hence we get a very high alt when converted to lla
  gps.lla_pos.lat = sensors.gps.lla_pos.lat * 1e7;
  gps.lla_pos.lon = sensors.gps.lla_pos.lon * 1e7;
  gps.lla_pos.alt = sensors.gps.lla_pos.alt * 1000.;
  gps.hmsl        = sensors.gps.hmsl * 1000.;
  gps.fix = GPS_FIX_3D;
  gps_available = TRUE;
}

void gps_impl_init() {
  gps_available = FALSE;
}
