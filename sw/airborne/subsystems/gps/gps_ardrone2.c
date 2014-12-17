/*
 *
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

/**
 * @file subsystems/gps/gps_ardrone2.c
 * ARdrone 2 gps trough navdata for the SDK version and only works in combination with the ahrs ardrone2.
 */

#ifdef ARDRONE2_DEBUG
# include <stdio.h>
#endif

#include "subsystems/gps.h"
#include "math/pprz_geodetic_double.h"

bool_t gps_ardrone2_available;

void gps_impl_init(void)
{
  gps_ardrone2_available = FALSE;
}

void gps_ardrone2_parse(navdata_gps_t *navdata_gps)
{
  int i;

#ifdef ARDRONE2_DEBUG
  printf("state = %d\n", navdata_gps->gps_state);
#endif
  // Set the lla double struct from the navdata
  struct LlaCoor_d gps_lla_d;
  gps_lla_d.lat = RadOfDeg(navdata_gps->lat);
  gps_lla_d.lon = RadOfDeg(navdata_gps->lon);
  gps_lla_d.alt = navdata_gps->elevation;

  // Convert it to ecef
  struct EcefCoor_d gps_ecef_d;
  ecef_of_lla_d(&gps_ecef_d, &gps_lla_d);

  // Convert the lla and ecef to int and set them in gps
  ECEF_BFP_OF_REAL(gps.ecef_pos, gps_ecef_d);
  LLA_BFP_OF_REAL(gps.lla_pos, gps_lla_d);

  // TODO: parse other stuff
  gps.nb_channels = GPS_NB_CHANNELS;

  for (i = 0; i < GPS_NB_CHANNELS; i++) {
    gps.svinfos[i].svid = navdata_gps->channels[i].sat;
    gps.svinfos[i].cno = navdata_gps->channels[i].cn0;
  }

  // Check if we have a fix TODO: check if 2D or 3D fix?
  if (navdata_gps->gps_state == 1) {
    gps.fix = GPS_FIX_3D;
  } else {
    gps.fix = GPS_FIX_NONE;
  }

  // Set that there is a packet
  gps_ardrone2_available = TRUE;
}
