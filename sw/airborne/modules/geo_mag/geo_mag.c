/*
 * Copyright (C) 2012  Sergey Krukowski <softsr@yahoo.de>
 * Copyright (C) 2015  OpenUAS <info@openuas.org>
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
 * @brief Calculation of the Geomagnetic field vector from current location.
 * This module is based on the WMM2015 model (http://www.ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml).
 */

#include "modules/geo_mag/geo_mag.h"
#include "math/pprz_geodetic_wmm2015.h"
#include "math/pprz_algebra_double.h"
#include "subsystems/gps.h"
#include "subsystems/abi.h"

//FIXME: should not be in this spot
//for kill_throttle check
#include "autopilot.h"

#ifndef GEO_MAG_SENDER_ID
#define GEO_MAG_SENDER_ID 1
#endif

struct GeoMag geo_mag;

void geo_mag_init(void)
{
  geo_mag.calc_once = FALSE;
  geo_mag.ready = FALSE;
}

void geo_mag_periodic(void)
{
  //FIXME: kill_throttle has no place  in a geomag module
  if (!geo_mag.ready && gps.fix == GPS_FIX_3D && kill_throttle) {
    geo_mag.calc_once = TRUE;
  }
}

void geo_mag_event(void)
{
  if (geo_mag.calc_once) {
    double gha[MAXCOEFF]; // Geomag global variables
    int32_t nmax;

    /* Current date in decimal year, for example 2015.68 */
    double sdate = GPS_EPOCH_BEGIN +
                   (double)gps.week / WEEKS_IN_YEAR +
                   (double)gps.tow / 1000 / SECS_IN_YEAR;

    /* LLA Position in decimal degrees and altitude in km */
    double latitude = (double)gps.lla_pos.lat / 1e7;
    double longitude = (double)gps.lla_pos.lon / 1e7;
    double alt = (double)gps.lla_pos.alt / 1e6;

    // Calculates additional coeffs
    nmax = extrapsh(sdate, GEO_EPOCH, NMAX_1, NMAX_2, gha);
    // Calculates absolute magnet fields
    mag_calc(1, latitude, longitude, alt, nmax, gha,
             &geo_mag.vect.x, &geo_mag.vect.y, &geo_mag.vect.z,
             IEXT, EXT_COEFF1, EXT_COEFF2, EXT_COEFF3);

    // send as normalized float vector via ABI
    struct FloatVect3 h = { .x = geo_mag.vect.x,
                            .y = geo_mag.vect.y,
                            .z = geo_mag.vect.z };
    float_vect3_normalize(&h);
    AbiSendMsgGEO_MAG(GEO_MAG_SENDER_ID, &h);

    geo_mag.ready = TRUE;
  }
  geo_mag.calc_once = FALSE;
}
