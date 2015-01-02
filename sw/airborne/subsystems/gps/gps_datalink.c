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

bool_t gps_available;   ///< Is set to TRUE when a new REMOTE_GPS packet is received and parsed

/** GPS initialization */
void gps_impl_init(void)
{
  gps.fix = GPS_FIX_NONE;
  gps_available = FALSE;
  gps.gspeed = 700; // To enable course setting
  gps.cacc = 0; // To enable course setting
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
}

