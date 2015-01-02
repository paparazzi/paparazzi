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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file pprz_geodetic_wmm2010.h
 * @brief WMM2010 Geomagnetic field model.
 *
 * Based on the WMM2010 model (http://www.ngdc.noaa.gov/geomag/models.shtml)
 */

#ifndef WMM2010_H
#define WMM2010_H

#ifdef __cplusplus
extern "C" {
#endif

#define WMM2010_FRAC 2
#define N_MAX_OF_GH  12

// Geo mag current observation epoch begin
#define GEO_EPOCH 2010.
#define YR_MIN    2010.
#define YR_MAX    2015.
#define NMAX_1 12
#define NMAX_2 12

#define IEXT 0
#define EXT_COEFF1 (double)0
#define EXT_COEFF2 (double)0
#define EXT_COEFF3 (double)0

/// Begin of the GPS epoch
#define GPS_EPOCH_BEGIN (double)1980.016393442623
#define GPS_EPOCH_YEAR  1980
#define GPS_EPOCH_MONTH 1
#define GPS_EPOCH_DAY   6

#define WEEKS_IN_YEAR 52.143
#define SECS_IN_YEAR 31536000

#define MAXDEG 13
#define MAXCOEFF (MAXDEG*(MAXDEG+2)+1)

extern const double gh1[];
extern const double gh2[];

int16_t extrapsh(double date, double dte1, int16_t nmax1, int16_t nmax2, double *gh);
int16_t mag_calc(int16_t igdgc, double flat, double flon, double elev, int16_t nmax,
                 double *gh, double *geo_mag_x, double *geo_mag_y, double *geo_mag_z,
                 int16_t iext, double ext1, double ext2, double ext3);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* WMM2010_H */
