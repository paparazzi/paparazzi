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
 * @file pprz_geodetic_wmm2010.c
 * @brief WMM2010 Geomagnetic field model.
 *
 * Based on the WMM2010 model (http://www.ngdc.noaa.gov/geomag/models.shtml)
 */

#include "std.h"
#include "math/pprz_geodetic_wmm2010.h"

const double gh1[MAXCOEFF] = {
  0.0, -29496.6, -1586.3, 4944.4,
  -2396.6, 3026.1, -2707.7, 1668.6, -576.1,
  1340.1, -2326.2, -160.2, 1231.9, 251.9, 634, -536.6,
  912.6, 808.9, 286.4, 166.7, -211.2, -357.1, 164.3, 89.4, -309.1,
  -230.9, 357.2, 44.6, 200.3, 188.9, -141.1, -118.2, -163.0, 0.0, -7.8, 100.9,
  72.8, 68.6, -20.8, 76.0, 44.1, -141.4, 61.5, -22.8, -66.3, 13.2, 3.1, -77.9, 55.0,
  80.5, -75.1, -57.9, -4.7, -21.1, 45.3, 6.5, 13.9, 24.9, 10.4, 7.0, 1.7, -27.7, 4.9, -3.3,
  24.4, 8.1, 11.0, -14.5, -20.0, -5.6, 11.9, -19.3, -17.4, 11.5, 16.7, 10.9, 7.0, -14.1, -10.8, -3.7, 1.7,
  5.4, 9.4, -20.5, 3.4, 11.5, -5.2, 12.8, 3.1, -7.2, -12.4, -7.4, -0.7, 8.0, 8.4, 2.1, -8.5, -6.1, -10.1, 7.0,
  -2.0, -6.3, 2.8, 0.9, -0.1, -1.1, 4.7, -0.2, 4.4, 2.5, -7.2, -0.3, -1.0, 2.2, -3.9, 3.1, -2.0, -1.0, -2.0, -2.8, -8.3,
  3.0, -1.5, 0.2, -2.1, 1.7, 1.7, -0.6, -0.5, -1.8, 0.5, 0.9, -0.8, -0.4, 0.4, -2.5, 1.8, -1.3, 0.1, -2.1, 0.7, -1.9, 3.8, -1.8,
  -2.2, -0.2, -0.9, 0.3, 0.3, 1.0, 2.1, -0.6, -2.5, 0.9, 0.5, -0.1, 0.6, 0.5, 0.0, -0.4, 0.1, -0.4, 0.3, 0.2, -0.9, -0.8, -0.2, 0.0, 0.9
};

const double gh2[MAXCOEFF] =  {
  0.0, 11.6, 16.5, -25.9,
  -12.1, -4.4, -22.5, 1.9, -11.8,
  0.4, -4.1, 7.3, -2.9, -3.9, -7.7, -2.6,
  -1.8, 2.3, 1.1, -8.7, 2.7, 4.6, 3.9, -2.1, -0.8,
  -1.0, 0.6, 0.4, -1.8, 1.8, -1.0, 1.2, 0.9, 4.0, 1.0, -0.6,
  -0.2, -0.2, -0.2, -0.1, -2.1, 2.0, -0.4, -1.7, -0.6, -0.3, 0.5, 1.7, 0.9,
  0.1, -0.1, 0.7, -0.6, 0.3, 1.3, -0.1, 0.4, -0.1, 0.3, -0.8, -0.7, -0.3, 0.6, 0.3,
  -0.1, 0.1, -0.1, -0.6, 0.2, 0.2, 0.4, -0.2, 0.4, 0.3, 0.1, 0.3, -0.1, -0.6, 0.4, 0.2, 0.3,
  0.0, -0.1, 0.0, 0.0, -0.2, 0.3, 0.0, -0.4, -0.1, -0.3, 0.1, 0.1, 0.0, -0.1, -0.2, -0.4, 0.3, -0.2, 0.2,
  0.0, 0.0, 0.1, -0.1, -0.1, 0.2, 0.0, 0.0, -0.1, -0.1, -0.1, -0.2, 0.0, 0.0, -0.1, -0.1, -0.2, -0.2, 0.0, -0.2, -0.1,
  0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, -0.1, 0.0, -0.1, -0.1, 0.0, 0.0, -0.1,
  0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1, 0.0, 0.1, 0.0
};

int16_t extrapsh(double date, double dte1, int16_t nmax1, int16_t nmax2, double *gh)
{

  int16_t   nmax;
  int16_t   k, l;
  int16_t   ii;
  double factor;

  factor = date - dte1;
  if (nmax1 == nmax2) {
    k =  nmax1 * (nmax1 + 2);
    nmax = nmax1;
  } else {
    if (nmax1 > nmax2) {
      k = nmax2 * (nmax2 + 2);
      l = nmax1 * (nmax1 + 2);

      for (ii = k + 1; ii <= l; ++ii) {
        gh[ii] = gh1[ii];
      }

      nmax = nmax1;
    } else {
      k = nmax1 * (nmax1 + 2);
      l = nmax2 * (nmax2 + 2);

      for (ii = k + 1; ii <= l; ++ii) {
        gh[ii] = factor * gh2[ii];
      }

      nmax = nmax2;
    }
  }

  for (ii = 1; ii <= k; ++ii) {
    gh[ii] = gh1[ii] + factor * gh2[ii];
  }

  return (nmax);
}

int16_t mag_calc(int16_t igdgc, double flat, double flon, double elev, int16_t nmax, double *gh, double *geo_mag_x,
                 double *geo_mag_y, double *geo_mag_z, int16_t iext, double ext1, double ext2, double ext3)
{

  double earths_radius = 6371.2;
  double dtr = 0.01745329;
  double slat;
  double clat;
  double ratio;
  double aa, bb, cc, dd;
  double sd;
  double cd;
  double r;
  double a2;
  double b2;
  double rr = 0;
  double fm, fn = 0;
  double sl[14];
  double cl[14];
#ifdef GEO_MAG_DOUBLE
  double p[119];
  double q[119];
#else
  float p[119];
  float q[119];
#endif
  int ii, j, k, l, m, n;
  int npq;
  int ios;
  double argument;
  double power;
  a2 = 40680631.59;            /* WGS84 */
  b2 = 40408299.98;            /* WGS84 */
  ios = 0;
  r = elev;
  argument = flat * dtr;
  slat = sinf(argument);
  if ((90.0 - flat) < 0.001) {
    aa = 89.999;            /*  300 ft. from North pole  */
  } else {
    if ((90.0 + flat) < 0.001) {
      aa = -89.999;        /*  300 ft. from South pole  */
    } else {
      aa = flat;
    }
  }
  argument = aa * dtr;
  clat = cosf(argument);
  argument = flon * dtr;
  sl[1] = sinf(argument);
  cl[1] = cosf(argument);

  *geo_mag_x = 0;
  *geo_mag_y = 0;
  *geo_mag_z = 0;

  sd = 0.0;
  cd = 1.0;
  l = 1;
  n = 0;
  m = 1;
  npq = (nmax * (nmax + 3)) / 2;

  if (igdgc == 1) {
    aa = a2 * clat * clat;
    bb = b2 * slat * slat;
    cc = aa + bb;
    argument = cc;
    dd = sqrt(argument);
    argument = elev * (elev + 2.0 * dd) + (a2 * aa + b2 * bb) / cc;
    r = sqrt(argument);
    cd = (elev + dd) / r;
    sd = (a2 - b2) / dd * slat * clat / r;
    aa = slat;
    slat = slat * cd - clat * sd;
    clat = clat * cd + aa * sd;
  }

  ratio = earths_radius / r;
  argument = 3.0;
  aa = sqrt(argument);
  p[1] = 2.0 * slat;
  p[2] = 2.0 * clat;
  p[3] = 4.5 * slat * slat - 1.5;
  p[4] = 3.0 * aa * clat * slat;
  q[1] = -clat;
  q[2] = slat;
  q[3] = -3.0 * clat * slat;
  q[4] = aa * (slat * slat - clat * clat);

  for (k = 1; k <= npq; ++k) {
    if (n < m) {
      m = 0;
      n = n + 1;
      argument = ratio;
      power =  n + 2;
      rr = pow(argument, power);
      fn = n;
    }
    fm = m;
    if (k >= 5) {
      if (m == n) {
        argument = (1.0 - 0.5 / fm);
        aa = sqrt(argument);
        j = k - n - 1;
        p[k] = (1.0 + 1.0 / fm) * aa * clat * p[j];
        q[k] = aa * (clat * q[j] + slat / fm * p[j]);
        sl[m] = sl[m - 1] * cl[1] + cl[m - 1] * sl[1];
        cl[m] = cl[m - 1] * cl[1] - sl[m - 1] * sl[1];
      } else {
        argument = fn * fn - fm * fm;
        aa = sqrt(argument);
        argument = ((fn - 1.0) * (fn - 1.0)) - (fm * fm);
        bb = sqrt(argument) / aa;
        cc = (2.0 * fn - 1.0) / aa;
        ii = k - n;
        j = k - 2 * n + 1;
        p[k] = (fn + 1.0) * (cc * slat / fn * p[ii] - bb / (fn - 1.0) * p[j]);
        q[k] = cc * (slat * q[ii] - clat / fn * p[ii]) - bb * q[j];
      }
    }

    aa = rr * gh[l];

    if (m == 0) {
      *geo_mag_x = *geo_mag_x + aa * q[k];
      *geo_mag_z = *geo_mag_z - aa * p[k];
      l = l + 1;
    } else {
      bb = rr * gh[l + 1];
      cc = aa * cl[m] + bb * sl[m];
      *geo_mag_x = *geo_mag_x + cc * q[k];
      *geo_mag_z = *geo_mag_z - cc * p[k];
      if (clat > 0) {
        *geo_mag_y = *geo_mag_y + (aa * sl[m] - bb * cl[m]) *
                     fm * p[k] / ((fn + 1.0) * clat);
      } else {
        *geo_mag_y = *geo_mag_y + (aa * sl[m] - bb * cl[m]) * q[k] * slat;
      }
      l = l + 2;
    }
    m = m + 1;
  }
  if (iext != 0) {
    aa = ext2 * cl[1] + ext3 * sl[1];
    *geo_mag_x = *geo_mag_x - ext1 * clat + aa * slat;
    *geo_mag_y = *geo_mag_y + ext2 * sl[1] - ext3 * cl[1];
    *geo_mag_z = *geo_mag_z + ext1 * slat + aa * clat;
  }
  aa = *geo_mag_x;
  *geo_mag_x = *geo_mag_x * cd + *geo_mag_z * sd;
  *geo_mag_z = *geo_mag_z * cd - aa * sd;
  return (ios);
}
