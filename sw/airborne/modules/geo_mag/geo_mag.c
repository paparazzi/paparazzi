/*
 *
 * Copyright (C) 2012  Sergey Krukowski <softsr@yahoo.de>.
 *
 * This module based on the WMM2010 modell (http://www.ngdc.noaa.gov/geomag/models.shtml).
 *
 */

#include "modules/geo_mag/geo_mag.h"
#include "math/pprz_geodetic_wmm2010.h"
#include "math/pprz_algebra_double.h"
#include "subsystems/gps.h"
#include "autopilot.h"

// in int_cmpl_quat implementation, mag_h is an Int32Vect3 with INT32_MAG_FRAC
#include "subsystems/ahrs/ahrs_int_cmpl_quat.h"

bool_t geo_mag_calc_flag;
struct GeoMagVect geo_mag_vect;

void geo_mag_init(void) {
  geo_mag_calc_flag = FALSE;
  geo_mag_vect.ready = FALSE;
}

void geo_mag_periodic(void) {
  if(gps.fix == GPS_FIX_3D && !geo_mag_vect.ready && !autopilot_motors_on)
    geo_mag_calc_flag = TRUE;
}

void geo_mag_event(void) {

  if(geo_mag_calc_flag) {
    double gha[MAXCOEFF]; // Geomag global variables
    int32_t nmax;

    /* Current date in decimal year, for example 2012.68 */
    double sdate = GPS_EPOCH_BEGIN +
      (double)gps.week/WEEKS_IN_YEAR +
      (double)gps.tow/1000/SECS_IN_YEAR;

    /* LLA Position in decimal degrees and altitude in km */
    double latitude = DegOfRad((double)gps.lla_pos.lat / 1e7);
    double longitude = DegOfRad((double)gps.lla_pos.lon / 1e7);
    double alt = (double)gps.lla_pos.alt / 1e6;

    // Calculates additional coeffs
    nmax = extrapsh(sdate, GEO_EPOCH, NMAX_1, NMAX_2, gha);
    // Calculates absolute magnet fields
    mag_calc(1, latitude, longitude, alt, nmax, gha,
             &geo_mag_vect.x, &geo_mag_vect.y, &geo_mag_vect.z,
             IEXT, EXT_COEFF1, EXT_COEFF2, EXT_COEFF3);
    FLOAT_VECT3_NORMALIZE(geo_mag_vect);

    // convert to MAG_BFP and copy to ahrs
    VECT3_ASSIGN(ahrs_impl.mag_h, MAG_BFP_OF_REAL(geo_mag_vect.x), MAG_BFP_OF_REAL(geo_mag_vect.y), MAG_BFP_OF_REAL(geo_mag_vect.z));

    geo_mag_vect.ready = TRUE;
  }
  geo_mag_calc_flag = FALSE;
}
