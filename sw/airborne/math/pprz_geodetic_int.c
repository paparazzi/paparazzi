/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *               2009-2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *               2010-2014 Felix Ruess <felix.ruess@gmail.com>
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
 * @file pprz_geodetic_int.c
 * @brief Paparazzi fixed point math for geodetic calculations.
 *
 *
 */

#include "pprz_geodetic_int.h"
#include "pprz_algebra_int.h"


void ltp_of_ecef_rmat_from_lla_i(struct Int32RMat *ltp_of_ecef, struct LlaCoor_i *lla)
{

#if USE_DOUBLE_PRECISION_TRIG
  int32_t sin_lat = rint(BFP_OF_REAL(sin(RAD_OF_EM7DEG((double)lla->lat)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lat = rint(BFP_OF_REAL(cos(RAD_OF_EM7DEG((double)lla->lat)), HIGH_RES_TRIG_FRAC));
  int32_t sin_lon = rint(BFP_OF_REAL(sin(RAD_OF_EM7DEG((double)lla->lon)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lon = rint(BFP_OF_REAL(cos(RAD_OF_EM7DEG((double)lla->lon)), HIGH_RES_TRIG_FRAC));
#else
  int32_t sin_lat = rint(BFP_OF_REAL(sinf(RAD_OF_EM7DEG((float)lla->lat)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lat = rint(BFP_OF_REAL(cosf(RAD_OF_EM7DEG((float)lla->lat)), HIGH_RES_TRIG_FRAC));
  int32_t sin_lon = rint(BFP_OF_REAL(sinf(RAD_OF_EM7DEG((float)lla->lon)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lon = rint(BFP_OF_REAL(cosf(RAD_OF_EM7DEG((float)lla->lon)), HIGH_RES_TRIG_FRAC));
#endif

  ltp_of_ecef->m[0] = -sin_lon;
  ltp_of_ecef->m[1] =  cos_lon;
  ltp_of_ecef->m[2] =  0; /* this element is always zero http://en.wikipedia.org/wiki/Geodetic_system#From_ECEF_to_ENU */
  ltp_of_ecef->m[3] = (int32_t)((-(int64_t)sin_lat * (int64_t)cos_lon) >> HIGH_RES_TRIG_FRAC);
  ltp_of_ecef->m[4] = (int32_t)((-(int64_t)sin_lat * (int64_t)sin_lon) >> HIGH_RES_TRIG_FRAC);
  ltp_of_ecef->m[5] =  cos_lat;
  ltp_of_ecef->m[6] = (int32_t)(((int64_t)cos_lat * (int64_t)cos_lon) >> HIGH_RES_TRIG_FRAC);
  ltp_of_ecef->m[7] = (int32_t)(((int64_t)cos_lat * (int64_t)sin_lon) >> HIGH_RES_TRIG_FRAC);
  ltp_of_ecef->m[8] =  sin_lat;
}

void ltp_def_from_ecef_i(struct LtpDef_i *def, struct EcefCoor_i *ecef)
{

  /* store the origin of the tangeant plane */
  VECT3_COPY(def->ecef, *ecef);
  /* compute the lla representation of the origin */
  lla_of_ecef_i(&def->lla, &def->ecef);
  /* store the rotation matrix                    */
  ltp_of_ecef_rmat_from_lla_i(&def->ltp_of_ecef, &def->lla);

}

void ltp_def_from_lla_i(struct LtpDef_i *def, struct LlaCoor_i *lla)
{

  /* store the origin of the tangeant plane */
  LLA_COPY(def->lla, *lla);
  /* compute the ecef representation of the origin */
  ecef_of_lla_i(&def->ecef, &def->lla);
  /* store the rotation matrix                    */
  ltp_of_ecef_rmat_from_lla_i(&def->ltp_of_ecef, &def->lla);

}


/** Convert a point from ECEF to local ENU.
 * @param[out] enu  ENU point in cm
 * @param[in]  def  local coordinate system definition
 * @param[in]  ecef ECEF point in cm
 */
void enu_of_ecef_point_i(struct EnuCoor_i *enu, struct LtpDef_i *def, struct EcefCoor_i *ecef)
{

  struct EcefCoor_i delta;
  VECT3_DIFF(delta, *ecef, def->ecef);
  const int64_t tmpx = (int64_t)def->ltp_of_ecef.m[0] * delta.x +
                       (int64_t)def->ltp_of_ecef.m[1] * delta.y +
                       0; /* this element is always zero http://en.wikipedia.org/wiki/Geodetic_system#From_ECEF_to_ENU */
  enu->x = (int32_t)(tmpx >> HIGH_RES_TRIG_FRAC);
  const int64_t tmpy = (int64_t)def->ltp_of_ecef.m[3] * delta.x +
                       (int64_t)def->ltp_of_ecef.m[4] * delta.y +
                       (int64_t)def->ltp_of_ecef.m[5] * delta.z;
  enu->y = (int32_t)(tmpy >> HIGH_RES_TRIG_FRAC);
  const int64_t tmpz = (int64_t)def->ltp_of_ecef.m[6] * delta.x +
                       (int64_t)def->ltp_of_ecef.m[7] * delta.y +
                       (int64_t)def->ltp_of_ecef.m[8] * delta.z;
  enu->z = (int32_t)(tmpz >> HIGH_RES_TRIG_FRAC);

}


/** Convert a point from ECEF to local NED.
 * @param[out] ned  NED point in cm
 * @param[in]  def  local coordinate system definition
 * @param[in]  ecef ECEF point in cm
 */
void ned_of_ecef_point_i(struct NedCoor_i *ned, struct LtpDef_i *def, struct EcefCoor_i *ecef)
{
  struct EnuCoor_i enu;
  enu_of_ecef_point_i(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);
}


/** Convert a ECEF position to local ENU.
 * @param[out] enu  ENU position in meter << #INT32_POS_FRAC
 * @param[in]  def  local coordinate system definition
 * @param[in]  ecef ECEF position in cm
 */
void enu_of_ecef_pos_i(struct EnuCoor_i *enu, struct LtpDef_i *def, struct EcefCoor_i *ecef)
{
  struct EnuCoor_i enu_cm;
  enu_of_ecef_point_i(&enu_cm, def, ecef);

  /* enu = (enu_cm / 100) << INT32_POS_FRAC
   * to loose less range:
   * enu_cm = enu << (INT32_POS_FRAC-2) / 25
   * which puts max enu output Q23.8 range to 8388km / 25 = 335km
   */
  INT32_VECT3_LSHIFT(*enu, enu_cm, INT32_POS_FRAC - 2);
  VECT3_SDIV(*enu, *enu, 25);
}


/** Convert a ECEF position to local NED.
 * @param[out] ned  NED position in meter << #INT32_POS_FRAC
 * @param[in]  def  local coordinate system definition
 * @param[in]  ecef ECEF position in cm
 */
void ned_of_ecef_pos_i(struct NedCoor_i *ned, struct LtpDef_i *def, struct EcefCoor_i *ecef)
{
  struct EnuCoor_i enu;
  enu_of_ecef_pos_i(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);
}


/** Rotate a vector from ECEF to ENU.
 * @param[out] enu  vector in ENU coordinate system
 * @param[in]  def  local coordinate system definition
 * @param[in]  ecef vector in ECEF coordinate system
 */
void enu_of_ecef_vect_i(struct EnuCoor_i *enu, struct LtpDef_i *def, struct EcefCoor_i *ecef)
{

  const int64_t tmpx = (int64_t)def->ltp_of_ecef.m[0] * ecef->x +
                       (int64_t)def->ltp_of_ecef.m[1] * ecef->y +
                       0; /* this element is always zero http://en.wikipedia.org/wiki/Geodetic_system#From_ECEF_to_ENU */
  enu->x = (int32_t)(tmpx >> HIGH_RES_TRIG_FRAC);
  const int64_t tmpy = (int64_t)def->ltp_of_ecef.m[3] * ecef->x +
                       (int64_t)def->ltp_of_ecef.m[4] * ecef->y +
                       (int64_t)def->ltp_of_ecef.m[5] * ecef->z;
  enu->y = (int32_t)(tmpy >> HIGH_RES_TRIG_FRAC);
  const int64_t tmpz = (int64_t)def->ltp_of_ecef.m[6] * ecef->x +
                       (int64_t)def->ltp_of_ecef.m[7] * ecef->y +
                       (int64_t)def->ltp_of_ecef.m[8] * ecef->z;
  enu->z = (int32_t)(tmpz >> HIGH_RES_TRIG_FRAC);

}


/** Rotate a vector from ECEF to NED.
 * @param[out] ned  vector in NED coordinate system
 * @param[in]  def  local coordinate system definition
 * @param[in]  ecef vector in ECEF coordinate system
 */
void ned_of_ecef_vect_i(struct NedCoor_i *ned, struct LtpDef_i *def, struct EcefCoor_i *ecef)
{
  struct EnuCoor_i enu;
  enu_of_ecef_vect_i(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);
}


/** Rotate a vector from ENU to ECEF.
 * @param[out] ecef vector in ECEF coordinate system
 * @param[in]  def  local coordinate system definition
 * @param[in]  enu  vector in ENU coordinate system
 */
void ecef_of_enu_vect_i(struct EcefCoor_i *ecef, struct LtpDef_i *def, struct EnuCoor_i *enu)
{

  const int64_t tmpx = (int64_t)def->ltp_of_ecef.m[0] * enu->x +
                       (int64_t)def->ltp_of_ecef.m[3] * enu->y +
                       (int64_t)def->ltp_of_ecef.m[6] * enu->z;
  ecef->x = (int32_t)(tmpx >> HIGH_RES_TRIG_FRAC);

  const int64_t tmpy = (int64_t)def->ltp_of_ecef.m[1] * enu->x +
                       (int64_t)def->ltp_of_ecef.m[4] * enu->y +
                       (int64_t)def->ltp_of_ecef.m[7] * enu->z;
  ecef->y = (int32_t)(tmpy >> HIGH_RES_TRIG_FRAC);

  /* first element is always zero http://en.wikipedia.org/wiki/Geodetic_system#From_ENU_to_ECEF */
  const int64_t tmpz = (int64_t)def->ltp_of_ecef.m[5] * enu->y +
                       (int64_t)def->ltp_of_ecef.m[8] * enu->z;
  ecef->z = (int32_t)(tmpz >> HIGH_RES_TRIG_FRAC);

}


/** Rotate a vector from NED to ECEF.
 * @param[out] ecef vector in ECEF coordinate system
 * @param[in]  def  local coordinate system definition
 * @param[in]  ned  vector in NED coordinate system
 */
void ecef_of_ned_vect_i(struct EcefCoor_i *ecef, struct LtpDef_i *def, struct NedCoor_i *ned)
{
  struct EnuCoor_i enu;
  ENU_OF_TO_NED(enu, *ned);
  ecef_of_enu_vect_i(ecef, def, &enu);
}


/** Convert a point in local ENU to ECEF.
 * @param[out] ecef ECEF point in cm
 * @param[in]  def  local coordinate system definition
 * @param[in]  enu  ENU point in cm
 */
void ecef_of_enu_point_i(struct EcefCoor_i *ecef, struct LtpDef_i *def, struct EnuCoor_i *enu)
{
  ecef_of_enu_vect_i(ecef, def, enu);
  VECT3_ADD(*ecef, def->ecef);
}


/** Convert a point in local NED to ECEF.
 * @param[out] ecef ECEF point in cm
 * @param[in]  def  local coordinate system definition
 * @param[in]  ned  NED point in cm
 */
void ecef_of_ned_point_i(struct EcefCoor_i *ecef, struct LtpDef_i *def, struct NedCoor_i *ned)
{
  struct EnuCoor_i enu;
  ENU_OF_TO_NED(enu, *ned);
  ecef_of_enu_point_i(ecef, def, &enu);
}


/** Convert a local ENU position to ECEF.
 * @param[out] ecef ECEF position in cm
 * @param[in]  def  local coordinate system definition
 * @param[in]  enu  ENU position in meter << #INT32_POS_FRAC
 */
void ecef_of_enu_pos_i(struct EcefCoor_i *ecef, struct LtpDef_i *def, struct EnuCoor_i *enu)
{
  /* enu_cm = (enu * 100) >> INT32_POS_FRAC
   * to loose less range:
   * enu_cm = (enu * 25) >> (INT32_POS_FRAC-2)
   * which puts max enu input Q23.8 range to 8388km / 25 = 335km
   */
  struct EnuCoor_i enu_cm;
  VECT3_SMUL(enu_cm, *enu, 25);
  INT32_VECT3_RSHIFT(enu_cm, enu_cm, INT32_POS_FRAC - 2);
  ecef_of_enu_vect_i(ecef, def, &enu_cm);
  VECT3_ADD(*ecef, def->ecef);
}


/** Convert a local NED position to ECEF.
 * @param[out] ecef ECEF position in cm
 * @param[in]  def  local coordinate system definition
 * @param[in]  ned  NED position in meter << #INT32_POS_FRAC
 */
void ecef_of_ned_pos_i(struct EcefCoor_i *ecef, struct LtpDef_i *def, struct NedCoor_i *ned)
{
  struct EnuCoor_i enu;
  ENU_OF_TO_NED(enu, *ned);
  ecef_of_enu_pos_i(ecef, def, &enu);
}


void enu_of_lla_point_i(struct EnuCoor_i *enu, struct LtpDef_i *def, struct LlaCoor_i *lla)
{
  struct EcefCoor_i ecef;
  ecef_of_lla_i(&ecef, lla);
  enu_of_ecef_point_i(enu, def, &ecef);
}

void ned_of_lla_point_i(struct NedCoor_i *ned, struct LtpDef_i *def, struct LlaCoor_i *lla)
{
  struct EcefCoor_i ecef;
  ecef_of_lla_i(&ecef, lla);
  ned_of_ecef_point_i(ned, def, &ecef);
}

void enu_of_lla_vect_i(struct EnuCoor_i *enu, struct LtpDef_i *def, struct LlaCoor_i *lla)
{
  struct EcefCoor_i ecef;
  ecef_of_lla_i(&ecef, lla);
  enu_of_ecef_vect_i(enu, def, &ecef);
}

void ned_of_lla_vect_i(struct NedCoor_i *ned, struct LtpDef_i *def, struct LlaCoor_i *lla)
{
  struct EcefCoor_i ecef;
  ecef_of_lla_i(&ecef, lla);
  ned_of_ecef_vect_i(ned, def, &ecef);
}

/*
   For now we cheat and call the floating point version
   Anyone up for writing it in fixed point ?
*/
#include "pprz_geodetic_float.h"
#include "pprz_geodetic_double.h"

void lla_of_ecef_i(struct LlaCoor_i *out, struct EcefCoor_i *in)
{

#if USE_SINGLE_PRECISION_LLA_ECEF
  /* convert our input to floating point */
  struct EcefCoor_f in_f;
  in_f.x = M_OF_CM((float)in->x);
  in_f.y = M_OF_CM((float)in->y);
  in_f.z = M_OF_CM((float)in->z);
  /* calls the floating point transformation */
  struct LlaCoor_f out_f;
  lla_of_ecef_f(&out_f, &in_f);
  /* convert the output to fixed point       */
  out->lon = (int32_t)rint(EM7DEG_OF_RAD(out_f.lon));
  out->lat = (int32_t)rint(EM7DEG_OF_RAD(out_f.lat));
  out->alt = (int32_t)MM_OF_M(out_f.alt);
#else // use double precision by default
  /* convert our input to floating point */
  struct EcefCoor_d in_d;
  in_d.x = M_OF_CM((double)in->x);
  in_d.y = M_OF_CM((double)in->y);
  in_d.z = M_OF_CM((double)in->z);
  /* calls the floating point transformation */
  struct LlaCoor_d out_d;
  lla_of_ecef_d(&out_d, &in_d);
  /* convert the output to fixed point       */
  out->lon = (int32_t)rint(EM7DEG_OF_RAD(out_d.lon));
  out->lat = (int32_t)rint(EM7DEG_OF_RAD(out_d.lat));
  out->alt = (int32_t)MM_OF_M(out_d.alt);
#endif

}

void ecef_of_lla_i(struct EcefCoor_i *out, struct LlaCoor_i *in)
{

#if USE_SINGLE_PRECISION_LLA_ECEF
  /* convert our input to floating point */
  struct LlaCoor_f in_f;
  in_f.lon = RAD_OF_EM7DEG((float)in->lon);
  in_f.lat = RAD_OF_EM7DEG((float)in->lat);
  in_f.alt = M_OF_MM((float)in->alt);
  /* calls the floating point transformation */
  struct EcefCoor_f out_f;
  ecef_of_lla_f(&out_f, &in_f);
  /* convert the output to fixed point       */
  out->x = (int32_t)CM_OF_M(out_f.x);
  out->y = (int32_t)CM_OF_M(out_f.y);
  out->z = (int32_t)CM_OF_M(out_f.z);
#else // use double precision by default
  /* convert our input to floating point */
  struct LlaCoor_d in_d;
  in_d.lon = RAD_OF_EM7DEG((double)in->lon);
  in_d.lat = RAD_OF_EM7DEG((double)in->lat);
  in_d.alt = M_OF_MM((double)in->alt);
  /* calls the floating point transformation */
  struct EcefCoor_d out_d;
  ecef_of_lla_d(&out_d, &in_d);
  /* convert the output to fixed point       */
  out->x = (int32_t)CM_OF_M(out_d.x);
  out->y = (int32_t)CM_OF_M(out_d.y);
  out->z = (int32_t)CM_OF_M(out_d.z);
#endif

}
