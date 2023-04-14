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

void double_rmat_of_eulers_321(struct DoubleRMat *rm, struct DoubleEulers *e)
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

void double_quat_of_eulers(struct DoubleQuat *q, struct DoubleEulers *e)
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

void double_eulers_of_quat(struct DoubleEulers *e, struct DoubleQuat *q)
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

void double_quat_vmult(struct DoubleVect3 *v_out, struct DoubleQuat *q, struct DoubleVect3 *v_in)
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

void double_quat_comp(struct DoubleQuat *a2c, struct DoubleQuat *a2b, struct DoubleQuat *b2c)
{
  a2c->qi = a2b->qi * b2c->qi - a2b->qx * b2c->qx - a2b->qy * b2c->qy - a2b->qz * b2c->qz;
  a2c->qx = a2b->qi * b2c->qx + a2b->qx * b2c->qi + a2b->qy * b2c->qz - a2b->qz * b2c->qy;
  a2c->qy = a2b->qi * b2c->qy - a2b->qx * b2c->qz + a2b->qy * b2c->qi + a2b->qz * b2c->qx;
  a2c->qz = a2b->qi * b2c->qz + a2b->qx * b2c->qy - a2b->qy * b2c->qx + a2b->qz * b2c->qi;
}


void double_rmat_inv(struct DoubleRMat *m_b2a, struct DoubleRMat *m_a2b)
{
  /*RMAT_ELMT(*m_b2a, 0, 0) = RMAT_ELMT(*m_a2b, 0, 0);*/
  RMAT_ELMT(*m_b2a, 0, 1) = RMAT_ELMT(*m_a2b, 1, 0);
  RMAT_ELMT(*m_b2a, 0, 2) = RMAT_ELMT(*m_a2b, 2, 0);
  RMAT_ELMT(*m_b2a, 1, 0) = RMAT_ELMT(*m_a2b, 0, 1);
  /*RMAT_ELMT(*m_b2a, 1, 1) = RMAT_ELMT(*m_a2b, 1, 1);*/
  RMAT_ELMT(*m_b2a, 1, 2) = RMAT_ELMT(*m_a2b, 2, 1);
  RMAT_ELMT(*m_b2a, 2, 0) = RMAT_ELMT(*m_a2b, 0, 2);
  RMAT_ELMT(*m_b2a, 2, 1) = RMAT_ELMT(*m_a2b, 1, 2);
  /*RMAT_ELMT(*m_b2a, 2, 2) = RMAT_ELMT(*m_a2b, 2, 2);*/
}

/** Composition (multiplication) of two rotation matrices.
 * m_a2c = m_a2b comp m_b2c , aka  m_a2c = m_b2c * m_a2b
 */
void double_rmat_comp(struct DoubleRMat *m_a2c, struct DoubleRMat *m_a2b, struct DoubleRMat *m_b2c)
{
  m_a2c->m[0] = m_b2c->m[0] * m_a2b->m[0] + m_b2c->m[1] * m_a2b->m[3] + m_b2c->m[2] * m_a2b->m[6];
  m_a2c->m[1] = m_b2c->m[0] * m_a2b->m[1] + m_b2c->m[1] * m_a2b->m[4] + m_b2c->m[2] * m_a2b->m[7];
  m_a2c->m[2] = m_b2c->m[0] * m_a2b->m[2] + m_b2c->m[1] * m_a2b->m[5] + m_b2c->m[2] * m_a2b->m[8];
  m_a2c->m[3] = m_b2c->m[3] * m_a2b->m[0] + m_b2c->m[4] * m_a2b->m[3] + m_b2c->m[5] * m_a2b->m[6];
  m_a2c->m[4] = m_b2c->m[3] * m_a2b->m[1] + m_b2c->m[4] * m_a2b->m[4] + m_b2c->m[5] * m_a2b->m[7];
  m_a2c->m[5] = m_b2c->m[3] * m_a2b->m[2] + m_b2c->m[4] * m_a2b->m[5] + m_b2c->m[5] * m_a2b->m[8];
  m_a2c->m[6] = m_b2c->m[6] * m_a2b->m[0] + m_b2c->m[7] * m_a2b->m[3] + m_b2c->m[8] * m_a2b->m[6];
  m_a2c->m[7] = m_b2c->m[6] * m_a2b->m[1] + m_b2c->m[7] * m_a2b->m[4] + m_b2c->m[8] * m_a2b->m[7];
  m_a2c->m[8] = m_b2c->m[6] * m_a2b->m[2] + m_b2c->m[7] * m_a2b->m[5] + m_b2c->m[8] * m_a2b->m[8];
}

/** rotate 3D vector by rotation matrix.
 * vb = m_a2b * va
 */
void double_rmat_vmult(struct DoubleVect3 *vb, struct DoubleRMat *m_a2b, struct DoubleVect3 *va)
{
  vb->x = m_a2b->m[0] * va->x + m_a2b->m[1] * va->y + m_a2b->m[2] * va->z;
  vb->y = m_a2b->m[3] * va->x + m_a2b->m[4] * va->y + m_a2b->m[5] * va->z;
  vb->z = m_a2b->m[6] * va->x + m_a2b->m[7] * va->y + m_a2b->m[8] * va->z;
}

/** rotate 3D vector by transposed rotation matrix.
 * vb = m_b2a^T * va
 */
void double_rmat_transp_vmult(struct DoubleVect3 *vb, struct DoubleRMat *m_b2a, struct DoubleVect3 *va)
{
  vb->x = m_b2a->m[0] * va->x + m_b2a->m[3] * va->y + m_b2a->m[6] * va->z;
  vb->y = m_b2a->m[1] * va->x + m_b2a->m[4] * va->y + m_b2a->m[7] * va->z;
  vb->z = m_b2a->m[2] * va->x + m_b2a->m[5] * va->y + m_b2a->m[8] * va->z;
}

/* C n->b rotation matrix */
void double_rmat_of_quat(struct DoubleRMat *rm, struct DoubleQuat *q)
{
  const double _a = M_SQRT2 * q->qi;
  const double _b = M_SQRT2 * q->qx;
  const double _c = M_SQRT2 * q->qy;
  const double _d = M_SQRT2 * q->qz;
  const double a2_1 = _a * _a - 1;
  const double ab = _a * _b;
  const double ac = _a * _c;
  const double ad = _a * _d;
  const double bc = _b * _c;
  const double bd = _b * _d;
  const double cd = _c * _d;
  RMAT_ELMT(*rm, 0, 0) = a2_1 + _b * _b;
  RMAT_ELMT(*rm, 0, 1) = bc + ad;
  RMAT_ELMT(*rm, 0, 2) = bd - ac;
  RMAT_ELMT(*rm, 1, 0) = bc - ad;
  RMAT_ELMT(*rm, 1, 1) = a2_1 + _c * _c;
  RMAT_ELMT(*rm, 1, 2) = cd + ab;
  RMAT_ELMT(*rm, 2, 0) = bd + ac;
  RMAT_ELMT(*rm, 2, 1) = cd - ab;
  RMAT_ELMT(*rm, 2, 2) = a2_1 + _d * _d;
}
