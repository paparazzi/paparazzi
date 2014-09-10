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
 * @file pprz_algebra_double.c
 * @brief Paparazzi double precision floating point algebra.
 *
 */

#include "pprz_algebra_double.h"

void double_rmat_of_eulers_321(struct DoubleRMat* rm, struct DoubleEulers* e)
{
  const double sphi   = sin(e->phi);
  const double cphi   = cos(e->phi);
  const double stheta = sin(e->theta);
  const double ctheta = cos(e->theta);
  const double spsi   = sin(e->psi);
  const double cpsi   = cos(e->psi);

  RMAT_ELMT(*rm, 0, 0) = ctheta * cpsi;
  RMAT_ELMT(*rm, 0, 1) = ctheta * spsi;
  RMAT_ELMT(*rm, 0, 2) = -stheta;
  RMAT_ELMT(*rm, 1, 0) = sphi * stheta * cpsi - cphi * spsi;
  RMAT_ELMT(*rm, 1, 1) = sphi * stheta * spsi + cphi * cpsi;
  RMAT_ELMT(*rm, 1, 2) = sphi * ctheta;
  RMAT_ELMT(*rm, 2, 0) = cphi * stheta * cpsi + sphi * spsi;
  RMAT_ELMT(*rm, 2, 1) = cphi * stheta * spsi - sphi * cpsi;
  RMAT_ELMT(*rm, 2, 2) = cphi * ctheta;
}

void double_quat_of_eulers(struct DoubleQuat* q, struct DoubleEulers* e)
{
  const double phi2   = e->phi / 2.0;
  const double theta2 = e->theta / 2.0;
  const double psi2   = e->psi / 2.0;

  const double s_phi2   = sin(phi2);
  const double c_phi2   = cos(phi2);
  const double s_theta2 = sin(theta2);
  const double c_theta2 = cos(theta2);
  const double s_psi2   = sin(psi2);
  const double c_psi2   = cos(psi2);

  q->qi =  c_phi2 * c_theta2 * c_psi2 + s_phi2 * s_theta2 * s_psi2;
  q->qx = -c_phi2 * s_theta2 * s_psi2 + s_phi2 * c_theta2 * c_psi2;
  q->qy =  c_phi2 * s_theta2 * c_psi2 + s_phi2 * c_theta2 * s_psi2;
  q->qz =  c_phi2 * c_theta2 * s_psi2 - s_phi2 * s_theta2 * c_psi2;
}

void double_eulers_of_quat(struct DoubleEulers* e, struct DoubleQuat* q)
{
  const double qx2  = q->qx * q->qx;
  const double qy2  = q->qy * q->qy;
  const double qz2  = q->qz * q->qz;
  const double qiqx = q->qi * q->qx;
  const double qiqy = q->qi * q->qy;
  const double qiqz = q->qi * q->qz;
  const double qxqy = q->qx * q->qy;
  const double qxqz = q->qx * q->qz;
  const double qyqz = q->qy * q->qz;
  const double dcm00 = 1.0 - 2.*(qy2 +  qz2);
  const double dcm01 =       2.*(qxqy + qiqz);
  const double dcm02 =       2.*(qxqz - qiqy);
  const double dcm12 =       2.*(qyqz + qiqx);
  const double dcm22 = 1.0 - 2.*(qx2 +  qy2);

  e->phi = atan2(dcm12, dcm22);
  e->theta = -asin(dcm02);
  e->psi = atan2(dcm01, dcm00);
}

void double_quat_vmult(struct DoubleVect3* v_out, struct DoubleQuat* q, struct DoubleVect3* v_in)
{
  const double qi2_M1_2  = q->qi * q->qi - 0.5;
  const double qiqx = q->qi * q->qx;
  const double qiqy = q->qi * q->qy;
  const double qiqz = q->qi * q->qz;
  double m01  = q->qx * q->qy; /* aka qxqy */
  double m02  = q->qx * q->qz; /* aka qxqz */
  double m12  = q->qy * q->qz; /* aka qyqz */

  const double m00  = qi2_M1_2 + q->qx * q->qx;
  const double m10  = m01 - qiqz;
  const double m20  = m02 + qiqy;
  const double m21  = m12 - qiqx;
  m01 += qiqz;
  m02 -= qiqy;
  m12 += qiqx;
  const double m11  = qi2_M1_2 + q->qy * q->qy;
  const double m22  = qi2_M1_2 + q->qz * q->qz;
  v_out->x = 2 * (m00 * v_in->x + m01 * v_in->y + m02 * v_in->z);
  v_out->y = 2 * (m10 * v_in->x + m11 * v_in->y + m12 * v_in->z);
  v_out->z = 2 * (m20 * v_in->x + m21 * v_in->y + m22 * v_in->z);
}
