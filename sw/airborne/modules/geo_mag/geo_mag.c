/*
 *
 * Copyright (C) 2012  Sergey Krukowski <softsr@yahoo.de>.
 * 
 * This module based on the WMM2010 modell (http://www.ngdc.noaa.gov/geomag/models.shtml).
 * 
 */

#include "modules/geo_mag/geo_mag.h"
#include "modules/geo_mag/wmm2010.h"
#include "math/pprz_algebra_float.h"
#include "subsystems/gps.h"
#include "autopilot.h"
#include "subsystems/ahrs/ahrs_int_cmpl_quat.h" // in AHRS subsystem AHRS_H must be float variables

bool_t calc_flag;
struct GeoMagVect geo_mag_vect;

void geo_mag_init(void) {
  calc_flag = FALSE;
	geo_mag_vect.ready = FALSE;
}

void geo_mag_periodic(void) {
	if(gps.fix == GPS_FIX_3D && !geo_mag_vect.ready && !autopilot_motors_on)
		calc_flag = TRUE;
}

void geo_mag_event(void) {
	
	if(calc_flag) {
	
		double gha[MAXCOEFF];              /* Geomag global variables */
		int32_t nmax;
		
		double sdate = julday(GPS_EPOCH_MONTH, GPS_EPOCH_DAY, GPS_EPOCH_YEAR) + 
			(double)gps.week/WEEKS_IN_YEAR + (double)gps.tow/1000/SECS_IN_YEAR; 	// in decimal year
		double latitude = DegOfRad((double)gps.lla_pos.lat / 1e7); 						// in decimal degree
		double longitude = DegOfRad((double)gps.lla_pos.lon / 1e7); 						// in decimal degree
		double alt = (double)gps.lla_pos.alt / 1e6; 														// in km
		
		nmax = extrapsh(sdate, GEO_EPOCH, NMAX_1, NMAX_2, gha);
		shval3(1, latitude, longitude, alt, nmax, gha, &geo_mag_vect.x, &geo_mag_vect.y, &geo_mag_vect.z, IEXT, EXT_COEFF1, EXT_COEFF2, EXT_COEFF3);
		
		FLOAT_VECT3_NORMALIZE(geo_mag_vect);
		
		VECT3_COPY(ahrs_h, geo_mag_vect);
		
		geo_mag_vect.ready = TRUE;
		
	}
	calc_flag = FALSE;
}
