/*
 * Copyright (C) 2013 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file subsystems/gps/gps_ardrone2.h
 * ARdrone 2 gps trough navdata for the SDK version and only works in combination with the ahrs ardrone2.
 *
*/

#ifndef GPS_ARDRONE_H
#define GPS_ARDRONE_H

#include "boards/ardrone/at_com.h"

//#define GPS_NB_CHANNELS 12 // TODO: Get channels out of packet
extern bool_t gps_ardrone2_available;

/*
 * The GPS event
 */
#define GpsEvent(_sol_available_callback) {        \
    if (gps_ardrone2_available) {                   \
    	if (gps.fix == GPS_FIX_3D) {               \
        gps.last_fix_ticks = sys_time.nb_sec_rem;  \
        gps.last_fix_time = sys_time.nb_sec;       \
      }                                            \
      _sol_available_callback();                   \
      gps_ardrone2_available = FALSE;               \
    }                                              \
  }

void gps_ardrone2_parse(navdata_gps_t *navdata_gps);

/* Maybe needed?
#define gps_nmea_Reset(_val) { }
*/

#endif /* GPS_ARDRONE_H */
