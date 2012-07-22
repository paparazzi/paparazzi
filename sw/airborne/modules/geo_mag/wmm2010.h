/*
 *
 * Copyright (C) 2012  Sergey Krukowski <softsr@yahoo.de>.
 *
 * This module based on the WMM2010 modell (http://www.ngdc.noaa.gov/geomag/models.shtml).
 *
 */

#ifndef WMM2010_H
#define WMM2010_H

#define WMM2010_FRAC 	12
#define N_MAX_OF_GH 	12

#define GEO_EPOCH 		2010.
#define YR_MIN 				2010.
#define YR_MAX 				2015.
#define NMAX_1 				12
#define NMAX_2 				12

#define IEXT 0
#define EXT_COEFF1 (double)0
#define EXT_COEFF2 (double)0
#define EXT_COEFF3 (double)0

#define GPS_EPOCH_YEAR 			1980
#define GPS_EPOCH_MONTH 		1
#define GPS_EPOCH_DAY		 		6

#define WEEKS_IN_YEAR 			52.143
#define SECS_IN_YEAR 				31536000

#define MAXDEG 13
#define MAXCOEFF (MAXDEG*(MAXDEG+2)+1)

//extern const int32_t wmm2010_1[];
//extern const int32_t wmm2010_2[];
extern const double gh1[];
extern const double gh2[];

//void getshc(int16_t, int16_t, int16_t, int32_t *);
double julday(uint16_t month, uint16_t day, uint16_t year);
int16_t extrapsh(double date, double dte1, int16_t nmax1, int16_t nmax2, double *gh);
int16_t shval3(int16_t igdgc, double flat, double flon, double elev, int16_t nmax, double *gh, double *geo_mag_x, double *geo_mag_y, double *geo_mag_z, int16_t iext, double ext1, double ext2, double ext3);

#endif
