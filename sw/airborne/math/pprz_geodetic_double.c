/*
 * Copyright (C) 2008-2014 The Paparazzi Team
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
 * @file pprz_geodetic_double.c
 * @brief Paparazzi double-precision floating point math for geodetic calculations.
 *
 *
 */

#include "pprz_geodetic_double.h"

#include <math.h>
#include "std.h" /* for RadOfDeg */


void ltp_def_from_ecef_d(struct LtpDef_d *def, struct EcefCoor_d *ecef)
{

  /* store the origin of the tangeant plane       */
  VECT3_COPY(def->ecef, *ecef);
  /* compute the lla representation of the origin */
  lla_of_ecef_d(&def->lla, &def->ecef);
  /* store the rotation matrix                    */
  const double sin_lat = sin(def->lla.lat);
  const double cos_lat = cos(def->lla.lat);
  const double sin_lon = sin(def->lla.lon);
  const double cos_lon = cos(def->lla.lon);
  def->ltp_of_ecef.m[0] = -sin_lon;
  def->ltp_of_ecef.m[1] =  cos_lon;
  def->ltp_of_ecef.m[2] =  0.;
  def->ltp_of_ecef.m[3] = -sin_lat * cos_lon;
  def->ltp_of_ecef.m[4] = -sin_lat * sin_lon;
  def->ltp_of_ecef.m[5] =  cos_lat;
  def->ltp_of_ecef.m[6] =  cos_lat * cos_lon;
  def->ltp_of_ecef.m[7] =  cos_lat * sin_lon;
  def->ltp_of_ecef.m[8] =  sin_lat;

}

/* http://en.wikipedia.org/wiki/Geodetic_system */
void lla_of_ecef_d(struct LlaCoor_d *lla, struct EcefCoor_d *ecef)
{

  // FIXME : make an ellipsoid struct
  static const double a = 6378137.0;           /* earth semimajor axis in meters */
  static const double f = 1. / 298.257223563;  /* reciprocal flattening          */
  const double b = a * (1. - f);               /* semi-minor axis                */
  const double b2 = b * b;

  const double e2 = 2.*f - (f * f);            /* first eccentricity squared     */
  const double ep2 = f * (2. - f) / ((1. - f) * (1. - f)); /* second eccentricity squared    */
  const double E2 = a * a - b2;


  const double z2 = ecef->z * ecef->z;
  const double r2 = ecef->x * ecef->x + ecef->y * ecef->y;
  const double r = sqrt(r2);
  const double F = 54.*b2 * z2;
  const double G = r2 + (1 - e2) * z2 - e2 * E2;
  const double c = (e2 * e2 * F * r2) / (G * G * G);
  const double s = pow((1 + c + sqrt(c * c + 2 * c)), 1. / 3.);
  const double s1 = 1 + s + 1 / s;
  const double P = F / (3 * s1 * s1 * G * G);
  const double Q = sqrt(1 + 2 * e2 * e2 * P);
  const double ro = -(e2 * P * r) / (1 + Q) + sqrt((a * a / 2) * (1 + 1 / Q) - ((1 - e2) * P * z2) / (Q *
                    (1 + Q)) - P * r2 / 2);
  const double tmp = (r - e2 * ro) * (r - e2 * ro);
  const double U = sqrt(tmp + z2);
  const double V = sqrt(tmp + (1 - e2) * z2);
  const double zo = (b2 * ecef->z) / (a * V);

  lla->alt = U * (1 - b2 / (a * V));
  lla->lat = atan((ecef->z + ep2 * zo) / r);
  lla->lon = atan2(ecef->y, ecef->x);

}

void ecef_of_lla_d(struct EcefCoor_d *ecef, struct LlaCoor_d *lla)
{

  // FIXME : make an ellipsoid struct
  static const double a = 6378137.0;           /* earth semimajor axis in meters */
  static const double f = 1. / 298.257223563;  /* reciprocal flattening          */
  const double e2 = 2.*f - (f * f);            /* first eccentricity squared     */

  const double sin_lat = sin(lla->lat);
  const double cos_lat = cos(lla->lat);
  const double sin_lon = sin(lla->lon);
  const double cos_lon = cos(lla->lon);
  const double chi = sqrt(1. - e2 * sin_lat * sin_lat);
  const double a_chi = a / chi;

  ecef->x = (a_chi + lla->alt) * cos_lat * cos_lon;
  ecef->y = (a_chi + lla->alt) * cos_lat * sin_lon;
  ecef->z = (a_chi * (1. - e2) + lla->alt) * sin_lat;
}

void enu_of_ecef_point_d(struct EnuCoor_d *enu, struct LtpDef_d *def, struct EcefCoor_d *ecef)
{
  struct EcefCoor_d delta;
  VECT3_DIFF(delta, *ecef, def->ecef);
  MAT33_VECT3_MUL(*enu, def->ltp_of_ecef, delta);
}

void ned_of_ecef_point_d(struct NedCoor_d *ned, struct LtpDef_d *def, struct EcefCoor_d *ecef)
{
  struct EnuCoor_d enu;
  enu_of_ecef_point_d(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);
}

void enu_of_ecef_vect_d(struct EnuCoor_d *enu, struct LtpDef_d *def, struct EcefCoor_d *ecef)
{
  MAT33_VECT3_MUL(*enu, def->ltp_of_ecef, *ecef);
}

void ned_of_ecef_vect_d(struct NedCoor_d *ned, struct LtpDef_d *def, struct EcefCoor_d *ecef)
{
  struct EnuCoor_d enu;
  enu_of_ecef_vect_d(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);
}



void ecef_of_enu_point_d(struct EcefCoor_d *ecef, struct LtpDef_d *def, struct EnuCoor_d *enu)
{
  MAT33_VECT3_TRANSP_MUL(*ecef, def->ltp_of_ecef, *enu);
  VECT3_ADD(*ecef, def->ecef);
}

void ecef_of_ned_point_d(struct EcefCoor_d *ecef, struct LtpDef_d *def, struct NedCoor_d *ned)
{
  struct EnuCoor_d enu;
  ENU_OF_TO_NED(enu, *ned);
  ecef_of_enu_point_d(ecef, def, &enu);
}

void ecef_of_enu_vect_d(struct EcefCoor_d *ecef, struct LtpDef_d *def, struct EnuCoor_d *enu)
{
  MAT33_VECT3_TRANSP_MUL(*ecef, def->ltp_of_ecef, *enu);
}

void ecef_of_ned_vect_d(struct EcefCoor_d *ecef, struct LtpDef_d *def, struct NedCoor_d *ned)
{
  struct EnuCoor_d enu;
  ENU_OF_TO_NED(enu, *ned);
  ecef_of_enu_vect_d(ecef, def, &enu);
}


void enu_of_lla_point_d(struct EnuCoor_d *enu, struct LtpDef_d *def, struct LlaCoor_d *lla)
{
  struct EcefCoor_d ecef;
  ecef_of_lla_d(&ecef, lla);
  enu_of_ecef_point_d(enu, def, &ecef);
}

void ned_of_lla_point_d(struct NedCoor_d *ned, struct LtpDef_d *def, struct LlaCoor_d *lla)
{
  struct EcefCoor_d ecef;
  ecef_of_lla_d(&ecef, lla);
  ned_of_ecef_point_d(ned, def, &ecef);
}


/* geocentric latitude of geodetic latitude */
double gc_of_gd_lat_d(double gd_lat, double hmsl)
{
  const double a = 6378137.0;           /* earth semimajor axis in meters */
  const double f = 1. / 298.257223563;  /* reciprocal flattening          */
  const double c2 = (1. - f) * (1. - f);
  /* geocentric latitude at the planet surface */
  double ls = atan(c2 * tan(gd_lat));
  return atan2(hmsl * sin(gd_lat) + a * sin(ls), hmsl * cos(gd_lat) + a * cos(ls));
}


#include "math/pprz_geodetic_utm.h"

static inline double UNUSED isometric_latitude_d(double phi, double e)
{
  return log(tan(M_PI_4 + phi / 2.0)) - e / 2.0 * log((1.0 + e * sin(phi)) / (1.0 - e * sin(phi)));
}

static inline double isometric_latitude_fast_d(double phi)
{
  return log(tan(M_PI_4 + phi / 2.0));
}

static inline double inverse_isometric_latitude_d(double lat, double e, double epsilon)
{
  double exp_l = exp(lat);
  double phi0 = 2 * atan(exp_l) - M_PI_2;
  double phi_;
  uint8_t max_iter = 3; /* To be sure to return */

  do {
    phi_ = phi0;
    double sin_phi = e * sin(phi_);
    phi0 = 2 * atan(pow((1 + sin_phi) / (1. - sin_phi), e / 2.) * exp_l) - M_PI_2;
    max_iter--;
  } while (max_iter && fabs(phi_ - phi0) > epsilon);

  return phi0;
}

#define CI(v) {         \
    double tmp = v.x;       \
    v.x = -v.y;         \
    v.y = tmp;          \
  }

#define CExp(v) {       \
    double e = exp(v.x);      \
    v.x = e*cosf(v.y);        \
    v.y = e*sinf(v.y);        \
  }

#define CSin(v) {       \
    CI(v);          \
    struct DoubleVect2 vstar = {-v.x, -v.y};  \
    CExp(v);          \
    CExp(vstar);        \
    VECT2_SUB(v, vstar);      \
    VECT2_SMUL(v, v, -0.5);     \
    CI(v);          \
  }

void lla_of_utm_d(struct LlaCoor_d *lla, struct UtmCoor_d *utm)
{

  struct DoubleVect2 v = {utm->north - DELTA_NORTH, utm->east - DELTA_EAST};
  double scale = 1 / N / serie_coeff_proj_mercator[0];
  VECT2_SMUL(v, v, scale);

  // first order taylor serie of something ?
  struct DoubleVect2 v1;
  VECT2_SMUL(v1, v, 2.);
  CSin(v1);
  VECT2_SMUL(v1, v1, serie_coeff_proj_mercator[1]);
  VECT2_SUB(v, v1);

  double lambda_c = LambdaOfUtmZone(utm->zone);
  lla->lon = lambda_c + atan(sinh(v.y) / cos(v.x));
  double phi = asin(sin(v.x) / cosh(v.y));
  double il = isometric_latitude_fast_d(phi);
  lla->lat = inverse_isometric_latitude_d(il, E, 1e-8);

  // copy alt above reference ellipsoid
  lla->alt = utm->alt;

}
