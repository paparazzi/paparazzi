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
 * @file pprz_geodetic_float.c
 * @brief Paparazzi floating point math for geodetic calculations.
 *
 *
 */

#include "pprz_geodetic_float.h"

#include "pprz_algebra_float.h"
#include <math.h>

/* for ecef_of_XX functions the double versions are needed */
#include "pprz_geodetic_double.h"

void ltp_def_from_ecef_f(struct LtpDef_f *def, struct EcefCoor_f *ecef)
{

  /* store the origin of the tangeant plane       */
  VECT3_COPY(def->ecef, *ecef);
  /* compute the lla representation of the origin */
  lla_of_ecef_f(&def->lla, &def->ecef);
  /* store the rotation matrix                    */
  const float sin_lat = sinf(def->lla.lat);
  const float cos_lat = cosf(def->lla.lat);
  const float sin_lon = sinf(def->lla.lon);
  const float cos_lon = cosf(def->lla.lon);
  def->ltp_of_ecef.m[0] = -sin_lon;
  def->ltp_of_ecef.m[1] =  cos_lon;
  /* this element is always zero http://en.wikipedia.org/wiki/Geodetic_system#From_ECEF_to_ENU */
  def->ltp_of_ecef.m[2] = 0.;
  def->ltp_of_ecef.m[3] = -sin_lat * cos_lon;
  def->ltp_of_ecef.m[4] = -sin_lat * sin_lon;
  def->ltp_of_ecef.m[5] =  cos_lat;
  def->ltp_of_ecef.m[6] =  cos_lat * cos_lon;
  def->ltp_of_ecef.m[7] =  cos_lat * sin_lon;
  def->ltp_of_ecef.m[8] =  sin_lat;

}

void ltp_def_from_lla_f(struct LtpDef_f *def, struct LlaCoor_f *lla)
{
  /* store the origin of the tangeant plane */
  LLA_COPY(def->lla, *lla);
  /* compute the ecef representation of the origin */
  ecef_of_lla_f(&def->ecef, &def->lla);

  /* store the rotation matrix                    */
  const float sin_lat = sinf(def->lla.lat);
  const float cos_lat = cosf(def->lla.lat);
  const float sin_lon = sinf(def->lla.lon);
  const float cos_lon = cosf(def->lla.lon);

  def->ltp_of_ecef.m[0] = -sin_lon;
  def->ltp_of_ecef.m[1] =  cos_lon;
  /* this element is always zero http://en.wikipedia.org/wiki/Geodetic_system#From_ECEF_to_ENU */
  def->ltp_of_ecef.m[2] = 0.;
  def->ltp_of_ecef.m[3] = -sin_lat * cos_lon;
  def->ltp_of_ecef.m[4] = -sin_lat * sin_lon;
  def->ltp_of_ecef.m[5] =  cos_lat;
  def->ltp_of_ecef.m[6] =  cos_lat * cos_lon;
  def->ltp_of_ecef.m[7] =  cos_lat * sin_lon;
  def->ltp_of_ecef.m[8] =  sin_lat;
}

void enu_of_ecef_point_f(struct EnuCoor_f *enu, struct LtpDef_f *def, struct EcefCoor_f *ecef)
{
  struct EcefCoor_f delta;
  VECT3_DIFF(delta, *ecef, def->ecef);
  MAT33_VECT3_MUL(*enu, def->ltp_of_ecef, delta);
}

void ned_of_ecef_point_f(struct NedCoor_f *ned, struct LtpDef_f *def, struct EcefCoor_f *ecef)
{
  struct EnuCoor_f enu;
  enu_of_ecef_point_f(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);
}


void enu_of_ecef_vect_f(struct EnuCoor_f *enu, struct LtpDef_f *def, struct EcefCoor_f *ecef)
{
  MAT33_VECT3_MUL(*enu, def->ltp_of_ecef, *ecef);
}

void ned_of_ecef_vect_f(struct NedCoor_f *ned, struct LtpDef_f *def, struct EcefCoor_f *ecef)
{
  struct EnuCoor_f enu;
  enu_of_ecef_vect_f(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);
}

void enu_of_lla_point_f(struct EnuCoor_f *enu, struct LtpDef_f *def, struct LlaCoor_f *lla)
{
  struct EcefCoor_f ecef;
  ecef_of_lla_f(&ecef, lla);
  enu_of_ecef_point_f(enu, def, &ecef);
}

void ned_of_lla_point_f(struct NedCoor_f *ned, struct LtpDef_f *def, struct LlaCoor_f *lla)
{
  struct EcefCoor_f ecef;
  ecef_of_lla_f(&ecef, lla);
  ned_of_ecef_point_f(ned, def, &ecef);
}

/*
 * not enought precision with float - use double
 */
void ecef_of_enu_point_f(struct EcefCoor_f *ecef, struct LtpDef_f *def, struct EnuCoor_f *enu)
{
  /* convert used floats to double */
  struct DoubleRMat ltp_of_ecef_d;
  ltp_of_ecef_d.m[0] = (double) def->ltp_of_ecef.m[0];
  ltp_of_ecef_d.m[1] = (double) def->ltp_of_ecef.m[1];
  ltp_of_ecef_d.m[2] = (double) def->ltp_of_ecef.m[2];
  ltp_of_ecef_d.m[3] = (double) def->ltp_of_ecef.m[3];
  ltp_of_ecef_d.m[4] = (double) def->ltp_of_ecef.m[4];
  ltp_of_ecef_d.m[5] = (double) def->ltp_of_ecef.m[5];
  ltp_of_ecef_d.m[6] = (double) def->ltp_of_ecef.m[6];
  ltp_of_ecef_d.m[7] = (double) def->ltp_of_ecef.m[7];
  ltp_of_ecef_d.m[8] = (double) def->ltp_of_ecef.m[8];
  struct EnuCoor_f enu_d;
  enu_d.x = (double) enu->x;
  enu_d.y = (double) enu->y;
  enu_d.z = (double) enu->z;

  /* compute in double */
  struct EcefCoor_d ecef_d;
  MAT33_VECT3_TRANSP_MUL(ecef_d, ltp_of_ecef_d, enu_d);

  /* convert result back to float and add it*/
  ecef->x = (float) ecef_d.x + def->ecef.x;
  ecef->y = (float) ecef_d.y + def->ecef.y;
  ecef->z = (float) ecef_d.z + def->ecef.z;
}

void ecef_of_ned_point_f(struct EcefCoor_f *ecef, struct LtpDef_f *def, struct NedCoor_f *ned)
{
  struct EnuCoor_f enu;
  ENU_OF_TO_NED(enu, *ned);
  ecef_of_enu_point_f(ecef, def, &enu);
}

void ecef_of_enu_vect_f(struct EcefCoor_f *ecef, struct LtpDef_f *def, struct EnuCoor_f *enu)
{
  /* convert used floats to double */
  struct DoubleRMat ltp_of_ecef_d;
  ltp_of_ecef_d.m[0] = (double) def->ltp_of_ecef.m[0];
  ltp_of_ecef_d.m[1] = (double) def->ltp_of_ecef.m[1];
  ltp_of_ecef_d.m[2] = (double) def->ltp_of_ecef.m[2];
  ltp_of_ecef_d.m[3] = (double) def->ltp_of_ecef.m[3];
  ltp_of_ecef_d.m[4] = (double) def->ltp_of_ecef.m[4];
  ltp_of_ecef_d.m[5] = (double) def->ltp_of_ecef.m[5];
  ltp_of_ecef_d.m[6] = (double) def->ltp_of_ecef.m[6];
  ltp_of_ecef_d.m[7] = (double) def->ltp_of_ecef.m[7];
  ltp_of_ecef_d.m[8] = (double) def->ltp_of_ecef.m[8];
  struct EnuCoor_f enu_d;
  enu_d.x = (double) enu->x;
  enu_d.y = (double) enu->y;
  enu_d.z = (double) enu->z;

  /* compute in double */
  struct EcefCoor_d ecef_d;
  MAT33_VECT3_TRANSP_MUL(ecef_d, ltp_of_ecef_d, enu_d);

  /* convert result back to float*/
  ecef->x = (float) ecef_d.x;
  ecef->y = (float) ecef_d.y;
  ecef->z = (float) ecef_d.z;
}

void ecef_of_ned_vect_f(struct EcefCoor_f *ecef, struct LtpDef_f *def, struct NedCoor_f *ned)
{
  struct EnuCoor_f enu;
  ENU_OF_TO_NED(enu, *ned);
  ecef_of_enu_vect_f(ecef, def, &enu);
}
/* end use double versions */




/* http://en.wikipedia.org/wiki/Geodetic_system */
void lla_of_ecef_f(struct LlaCoor_f *out, struct EcefCoor_f *in)
{

  // FIXME : make an ellipsoid struct
  static const float a = 6378137.0;           /* earth semimajor axis in meters */
  static const float f = 1. / 298.257223563;  /* reciprocal flattening          */
  const float b = a * (1. - f);               /* semi-minor axis                */
  const float b2 = b * b;

  const float e2 = 2.*f - (f * f);            /* first eccentricity squared     */
  const float ep2 = f * (2. - f) / ((1. - f) * (1. - f)); /* second eccentricity squared    */
  const float E2 = a * a - b2;


  const float z2 = in->z * in->z;
  const float r2 = in->x * in->x + in->y * in->y;
  const float r = sqrtf(r2);
  const float F = 54.*b2 * z2;
  const float G = r2 + (1 - e2) * z2 - e2 * E2;
  const float c = (e2 * e2 * F * r2) / (G * G * G);
  const float s = powf((1 + c + sqrtf(c * c + 2 * c)), 1. / 3.);
  const float s1 = 1 + s + 1 / s;
  const float P = F / (3 * s1 * s1 * G * G);
  const float Q = sqrtf(1 + 2 * e2 * e2 * P);
  const float ro = -(e2 * P * r) / (1 + Q) + sqrtf((a * a / 2) * (1 + 1 / Q) - ((1 - e2) * P * z2) / (Q *
                   (1 + Q)) - P * r2 / 2);
  const float tmp = (r - e2 * ro) * (r - e2 * ro);
  const float U = sqrtf(tmp + z2);
  const float V = sqrtf(tmp + (1 - e2) * z2);
  const float zo = (b2 * in->z) / (a * V);

  out->alt = U * (1 - b2 / (a * V));
  out->lat = atanf((in->z + ep2 * zo) / r);
  out->lon = atan2f(in->y, in->x);

}

void ecef_of_lla_f(struct EcefCoor_f *out, struct LlaCoor_f *in)
{

  // FIXME : make an ellipsoid struct
  static const float a = 6378137.0;           /* earth semimajor axis in meters */
  static const float f = 1. / 298.257223563;  /* reciprocal flattening          */
  const float e2 = 2.*f - (f * f);            /* first eccentricity squared     */

  const float sin_lat = sinf(in->lat);
  const float cos_lat = cosf(in->lat);
  const float sin_lon = sinf(in->lon);
  const float cos_lon = cosf(in->lon);
  const float chi = sqrtf(1. - e2 * sin_lat * sin_lat);
  const float a_chi = a / chi;

  out->x = (a_chi + in->alt) * cos_lat * cos_lon;
  out->y = (a_chi + in->alt) * cos_lat * sin_lon;
  out->z = (a_chi * (1. - e2) + in->alt) * sin_lat;
}




#include "math/pprz_geodetic_utm.h"

struct complex { float re; float im; };
#define CScal(k, z) { z.re *= k;  z.im *= k; }
#define CAdd(z1, z2) { z2.re += z1.re;  z2.im += z1.im; }
#define CSub(z1, z2) { z2.re -= z1.re;  z2.im -= z1.im; }
#define CI(z) { float tmp = z.re; z.re = - z.im; z.im = tmp; }
#define CExp(z) { float e = exp(z.re); z.re = e*cos(z.im); z.im = e*sin(z.im); }
/* Expanded #define CSin(z) { CI(z); struct complex _z = {-z.re, -z.im}; CExp(z); CExp(_z); CSub(_z, z); CScal(-0.5, z); CI(z); } */

#define CSin(z) { CI(z); struct complex _z = {-z.re, -z.im}; float e = exp(z.re); float cos_z_im = cos(z.im); z.re = e*cos_z_im; float sin_z_im = sin(z.im); z.im = e*sin_z_im; _z.re = cos_z_im/e; _z.im = -sin_z_im/e; CSub(_z, z); CScal(-0.5, z); CI(z); }


static inline float isometric_latitude_f(float phi, float e)
{
  return log(tan(M_PI_4 + phi / 2.0)) - e / 2.0 * log((1.0 + e * sin(phi)) / (1.0 - e * sin(phi)));
}

static inline float isometric_latitude_fast_f(float phi)
{
  return log(tan(M_PI_4 + phi / 2.0));
}

static inline float inverse_isometric_latitude_f(float lat, float e, float epsilon)
{
  float exp_l = exp(lat);
  float phi0 = 2 * atan(exp_l) - M_PI_2;
  float phi_;
  uint8_t max_iter = 3; /* To be sure to return */

  do {
    phi_ = phi0;
    float sin_phi = e * sin(phi_);
    phi0 = 2 * atan(pow((1 + sin_phi) / (1. - sin_phi), e / 2.) * exp_l) - M_PI_2;
    max_iter--;
  } while (max_iter && fabs(phi_ - phi0) > epsilon);
  return phi0;
}

void utm_of_lla_f(struct UtmCoor_f *utm, struct LlaCoor_f *lla)
{
  float lambda_c = LambdaOfUtmZone(utm->zone);
  float ll = isometric_latitude_f(lla->lat , E);
  float dl = lla->lon - lambda_c;
  float phi_ = asin(sin(dl) / cosh(ll));
  float ll_ = isometric_latitude_fast_f(phi_);
  float lambda_ = atan(sinh(ll) / cos(dl));
  struct complex z_ = { lambda_,  ll_ };
  CScal(serie_coeff_proj_mercator[0], z_);
  uint8_t k;
  for (k = 1; k < 3; k++) {
    struct complex z = { lambda_,  ll_ };
    CScal(2 * k, z);
    CSin(z);
    CScal(serie_coeff_proj_mercator[k], z);
    CAdd(z, z_);
  }
  CScal(N, z_);
  utm->east = DELTA_EAST + z_.im;
  utm->north = DELTA_NORTH + z_.re;

  // copy alt above reference ellipsoid
  utm->alt = lla->alt;
}

void lla_of_utm_f(struct LlaCoor_f *lla, struct UtmCoor_f *utm)
{
  float scale = 1 / N / serie_coeff_proj_mercator[0];
  float real = (utm->north - DELTA_NORTH) * scale;
  float img = (utm->east - DELTA_EAST) * scale;
  struct complex z = { real, img };

  uint8_t k;
  for (k = 1; k < 2; k++) {
    struct complex z_ = { real, img };
    CScal(2 * k, z_);
    CSin(z_);
    CScal(serie_coeff_proj_mercator_inverse[k], z_);
    CSub(z_, z);
  }

  float lambda_c = LambdaOfUtmZone(utm->zone);
  lla->lon = lambda_c + atan(sinh(z.im) / cos(z.re));
  float phi_ = asin(sin(z.re) / cosh(z.im));
  float il = isometric_latitude_fast_f(phi_);
  lla->lat = inverse_isometric_latitude_f(il, E, 1e-8);

  // copy alt above reference ellipsoid
  lla->alt = utm->alt;
}
