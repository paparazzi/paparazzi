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

#include "subsystems/gps.h"

bool_t gps_available;


#if 0
void  gps_feed_values(double utm_north, double utm_east, double utm_alt, double gspeed, double course, double climb)
{
  gps.utm_pos.north = CM_OF_M(utm_north);
  gps.utm_pos.east = CM_OF_M(utm_east);
  //TODO set height above ellipsoid properly
  gps.hmsl = utm_alt * 1000.;
  gps.gspeed = CM_OF_M(gspeed);
  gps.course = EM7RAD_OF_RAD(RadOfDeg(course / 10.));
  gps.ned_vel.z = -climb * 100.;
  gps.fix = GPS_FIX_3D;
  gps_available = TRUE;
}
#endif

void gps_impl_init(void)
{
  gps.fix = GPS_FIX_NONE;
  gps_available = FALSE;
}
