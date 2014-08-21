/*
 * Copyright (C) 2012  Sergey Krukowski <softsr@yahoo.de>
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
 * @file modules/geo_mag/geo_mag.c
 * @brief Calculation of the Geomagnetic field vector from current GPS fix.
 * This module is based on the WMM2010 model (http://www.ngdc.noaa.gov/geomag/models.shtml).
 */

#include "modules/geo_mag/geo_mag.h"
#include "math/pprz_geodetic_wmm2010.h"
#include "math/pprz_algebra_double.h"
#include "subsystems/gps.h"
#include "autopilot.h"

#include "subsystems/ahrs.h"

bool_t geo_mag_calc_flag;
struct GeoMagVect geo_mag_vect;

void geo_mag_init(void) {
  geo_mag_calc_flag = FALSE;
  geo_mag_vect.ready = FALSE;
}

void geo_mag_periodic(void) {
  if (!geo_mag_vect.ready && gps.fix == GPS_FIX_3D && kill_throttle)
    geo_mag_calc_flag = TRUE;
}

void geo_mag_event(void) {

  if (geo_mag_calc_flag) {
    double gha[MAXCOEFF]; // Geomag global variables
    int32_t nmax;

    /* Current date in decimal year, for example 2012.68 */
    double sdate = GPS_EPOCH_BEGIN +
      (double)gps.week/WEEKS_IN_YEAR +
      (double)gps.tow/1000/SECS_IN_YEAR;

    /* LLA Position in decimal degrees and altitude in km */
    double latitude = (double)gps.lla_pos.lat / 1e7;
    double longitude = (double)gps.lla_pos.lon / 1e7;
    double alt = (double)gps.lla_pos.alt / 1e6;

    // Calculates additional coeffs
    nmax = extrapsh(sdate, GEO_EPOCH, NMAX_1, NMAX_2, gha);
    // Calculates absolute magnet fields
    mag_calc(1, latitude, longitude, alt, nmax, gha,
             &geo_mag_vect.x, &geo_mag_vect.y, &geo_mag_vect.z,
             IEXT, EXT_COEFF1, EXT_COEFF2, EXT_COEFF3);
    FLOAT_VECT3_NORMALIZE(geo_mag_vect);

    // copy to ahrs
#ifdef AHRS_FLOAT
    VECT3_COPY(ahrs_impl.mag_h, geo_mag_vect);
#else
    // convert to MAG_BFP and copy to ahrs
    VECT3_ASSIGN(ahrs_impl.mag_h, MAG_BFP_OF_REAL(geo_mag_vect.x), MAG_BFP_OF_REAL(geo_mag_vect.y), MAG_BFP_OF_REAL(geo_mag_vect.z));
#endif

    geo_mag_vect.ready = TRUE;
  }
  geo_mag_calc_flag = FALSE;
}
