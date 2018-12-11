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
 * @file pprz_algebra_float.c
 * @brief Paparazzi floating point algebra.
 *
 */

#include "pprz_algebra_float.h"

/** in place first order integration of a 3D-vector */
void float_vect3_integrate_fi(struct FloatVect3 *vec, struct FloatVect3 *dv, float dt)
{
  vec->x += dv->x * dt;
  vec->y += dv->y * dt;
  vec->z += dv->z * dt;
}

/** in place first order integration of angular rates */
void float_rates_integrate_fi(struct FloatRates *r, struct FloatRates *dr, float dt)
{
  r->p += dr->p * dt;
  r->q += dr->q * dt;
  r->r += dr->r * dt;
}

void float_rates_of_euler_dot(struct FloatRates *r, struct FloatEulers *e, struct FloatEulers *edot)
{
  r->p = edot->phi                             -                sinf(e->theta) * edot->psi;
  r->q =            cosf(e->phi) * edot->theta + sinf(e->phi) * cosf(e->theta) * edot->psi;
  r->r =           -sinf(e->phi) * edot->theta + cosf(e->phi) * cosf(e->theta) * edot->psi;
}




void float_rmat_inv(struct FloatRMat *m_b2a, struct FloatRMat *m_a2b)
{
  RMAT_ELMT(*m_b2a, 0, 0) = RMAT_ELMT(*m_a2b, 0, 0);
  RMAT_ELMT(*m_b2a, 0, 1) = RMAT_ELMT(*m_a2b, 1, 0);
  RMAT_ELMT(*m_b2a, 0, 2) = RMAT_ELMT(*m_a2b, 2, 0);
  RMAT_ELMT(*m_b2a, 1, 0) = RMAT_ELMT(*m_a2b, 0, 1);
  RMAT_ELMT(*m_b2a, 1, 1) = RMAT_ELMT(*m_a2b, 1, 1);
  RMAT_ELMT(*m_b2a, 1, 2) = RMAT_ELMT(*m_a2b, 2, 1);
  RMAT_ELMT(*m_b2a, 2, 0) = RMAT_ELMT(*m_a2b, 0, 2);
  RMAT_ELMT(*m_b2a, 2, 1) = RMAT_ELMT(*m_a2b, 1, 2);
  RMAT_ELMT(*m_b2a, 2, 2) = RMAT_ELMT(*m_a2b, 2, 2);
}

float float_rmat_norm(struct FloatRMat *rm)
{
  return sqrtf(SQUARE(rm->m[0]) + SQUARE(rm->m[1]) + SQUARE(rm->m[2]) +
               SQUARE(rm->m[3]) + SQUARE(rm->m[4]) + SQUARE(rm->m[5]) +
               SQUARE(rm->m[6]) + SQUARE(rm->m[7]) + SQUARE(rm->m[8]));
}

/** Composition (multiplication) of two rotation matrices.
 * m_a2c = m_a2b comp m_b2c , aka  m_a2c = m_b2c * m_a2b
 */
void float_rmat_comp(struct FloatRMat *m_a2c, struct FloatRMat *m_a2b, struct FloatRMat *m_b2c)
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

/** Composition (multiplication) of two rotation matrices.
 * m_a2b = m_a2c comp_inv m_b2c , aka  m_a2b = inv(_m_b2c) * m_a2c
 */
void float_rmat_comp_inv(struct FloatRMat *m_a2b, struct FloatRMat *m_a2c, struct FloatRMat *m_b2c)
{
  m_a2b->m[0] = m_b2c->m[0] * m_a2c->m[0] + m_b2c->m[3] * m_a2c->m[3] + m_b2c->m[6] * m_a2c->m[6];
  m_a2b->m[1] = m_b2c->m[0] * m_a2c->m[1] + m_b2c->m[3] * m_a2c->m[4] + m_b2c->m[6] * m_a2c->m[7];
  m_a2b->m[2] = m_b2c->m[0] * m_a2c->m[2] + m_b2c->m[3] * m_a2c->m[5] + m_b2c->m[6] * m_a2c->m[8];
  m_a2b->m[3] = m_b2c->m[1] * m_a2c->m[0] + m_b2c->m[4] * m_a2c->m[3] + m_b2c->m[7] * m_a2c->m[6];
  m_a2b->m[4] = m_b2c->m[1] * m_a2c->m[1] + m_b2c->m[4] * m_a2c->m[4] + m_b2c->m[7] * m_a2c->m[7];
  m_a2b->m[5] = m_b2c->m[1] * m_a2c->m[2] + m_b2c->m[4] * m_a2c->m[5] + m_b2c->m[7] * m_a2c->m[8];
  m_a2b->m[6] = m_b2c->m[2] * m_a2c->m[0] + m_b2c->m[5] * m_a2c->m[3] + m_b2c->m[8] * m_a2c->m[6];
  m_a2b->m[7] = m_b2c->m[2] * m_a2c->m[1] + m_b2c->m[5] * m_a2c->m[4] + m_b2c->m[8] * m_a2c->m[7];
  m_a2b->m[8] = m_b2c->m[2] * m_a2c->m[2] + m_b2c->m[5] * m_a2c->m[5] + m_b2c->m[8] * m_a2c->m[8];
}

/** rotate 3D vector by rotation matrix.
 * vb = m_a2b * va
 */
void float_rmat_vmult(struct FloatVect3 *vb, struct FloatRMat *m_a2b, struct FloatVect3 *va)
{
  vb->x = m_a2b->m[0] * va->x + m_a2b->m[1] * va->y + m_a2b->m[2] * va->z;
  vb->y = m_a2b->m[3] * va->x + m_a2b->m[4] * va->y + m_a2b->m[5] * va->z;
  vb->z = m_a2b->m[6] * va->x + m_a2b->m[7] * va->y + m_a2b->m[8] * va->z;
}

/** rotate 3D vector by transposed rotation matrix.
 * vb = m_b2a^T * va
 */
void float_rmat_transp_vmult(struct FloatVect3 *vb, struct FloatRMat *m_b2a, struct FloatVect3 *va)
{
  vb->x = m_b2a->m[0] * va->x + m_b2a->m[3] * va->y + m_b2a->m[6] * va->z;
  vb->y = m_b2a->m[1] * va->x + m_b2a->m[4] * va->y + m_b2a->m[7] * va->z;
  vb->z = m_b2a->m[2] * va->x + m_b2a->m[5] * va->y + m_b2a->m[8] * va->z;
}

/** rotate angle by rotation matrix.
 * rb = m_a2b * ra
 */
void float_rmat_mult(struct FloatEulers *rb, struct FloatRMat *m_a2b, struct FloatEulers *ra)
{
  rb->phi = m_a2b->m[0] * ra->phi + m_a2b->m[1] * ra->theta + m_a2b->m[2] * ra->psi;
  rb->theta = m_a2b->m[3] * ra->phi + m_a2b->m[4] * ra->theta + m_a2b->m[5] * ra->psi;
  rb->psi = m_a2b->m[6] * ra->phi + m_a2b->m[7] * ra->theta + m_a2b->m[8] * ra->psi;
}

/** rotate angle by transposed rotation matrix.
 * rb = m_b2a^T * ra
 */
void float_rmat_transp_mult(struct FloatEulers *rb, struct FloatRMat *m_b2a, struct FloatEulers *ra)
{
  rb->phi = m_b2a->m[0] * ra->phi + m_b2a->m[3] * ra->theta + m_b2a->m[6] * ra->psi;
  rb->theta = m_b2a->m[1] * ra->phi + m_b2a->m[4] * ra->theta + m_b2a->m[7] * ra->psi;
  rb->psi = m_b2a->m[2] * ra->phi + m_b2a->m[5] * ra->theta + m_b2a->m[8] * ra->psi;
}

/** rotate anglular rates by rotation matrix.
 * rb = m_a2b * ra
 */
void float_rmat_ratemult(struct FloatRates *rb, struct FloatRMat *m_a2b, struct FloatRates *ra)
{
  rb->p = m_a2b->m[0] * ra->p + m_a2b->m[1] * ra->q + m_a2b->m[2] * ra->r;
  rb->q = m_a2b->m[3] * ra->p + m_a2b->m[4] * ra->q + m_a2b->m[5] * ra->r;
  rb->r = m_a2b->m[6] * ra->p + m_a2b->m[7] * ra->q + m_a2b->m[8] * ra->r;
}

/** rotate anglular rates by transposed rotation matrix.
 * rb = m_b2a^T * ra
 */
void float_rmat_transp_ratemult(struct FloatRates *rb, struct FloatRMat *m_b2a, struct FloatRates *ra)
{
  rb->p = m_b2a->m[0] * ra->p + m_b2a->m[3] * ra->q + m_b2a->m[6] * ra->r;
  rb->q = m_b2a->m[1] * ra->p + m_b2a->m[4] * ra->q + m_b2a->m[7] * ra->r;
  rb->r = m_b2a->m[2] * ra->p + m_b2a->m[5] * ra->q + m_b2a->m[8] * ra->r;
}


/** initialises a rotation matrix from unit vector axis and angle */
void float_rmat_of_axis_angle(struct FloatRMat *rm, struct FloatVect3 *uv, float angle)
{
  const float ux2  = uv->x * uv->x;
  const float uy2  = uv->y * uv->y;
  const float uz2  = uv->z * uv->z;
  const float uxuy = uv->x * uv->y;
  const float uyuz = uv->y * uv->z;
  const float uxuz = uv->x * uv->z;
  const float can  = cosf(angle);
  const float san  = sinf(angle);
  const float one_m_can = (1. - can);

  RMAT_ELMT(*rm, 0, 0) = ux2 + (1. - ux2) * can;
  RMAT_ELMT(*rm, 0, 1) = uxuy * one_m_can + uv->z * san;
  RMAT_ELMT(*rm, 0, 2) = uxuz * one_m_can - uv->y * san;
  RMAT_ELMT(*rm, 1, 0) = RMAT_ELMT(*rm, 0, 1);
  RMAT_ELMT(*rm, 1, 1) = uy2 + (1. - uy2) * can;
  RMAT_ELMT(*rm, 1, 2) = uyuz * one_m_can + uv->x * san;
  RMAT_ELMT(*rm, 2, 0) = RMAT_ELMT(*rm, 0, 2);
  RMAT_ELMT(*rm, 2, 1) = RMAT_ELMT(*rm, 1, 2);
  RMAT_ELMT(*rm, 2, 2) = uz2 + (1. - uz2) * can;
}


/* C n->b rotation matrix */
void float_rmat_of_eulers_321(struct FloatRMat *rm, struct FloatEulers *e)
{
  const float sphi   = sinf(e->phi);
  const float cphi   = cosf(e->phi);
  const float stheta = sinf(e->theta);
  const float ctheta = cosf(e->theta);
  const float spsi   = sinf(e->psi);
  const float cpsi   = cosf(e->psi);

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

void float_rmat_of_eulers_312(struct FloatRMat *rm, struct FloatEulers *e)
{
  const float sphi   = sinf(e->phi);
  const float cphi   = cosf(e->phi);
  const float stheta = sinf(e->theta);
  const float ctheta = cosf(e->theta);
  const float spsi   = sinf(e->psi);
  const float cpsi   = cosf(e->psi);

  RMAT_ELMT(*rm, 0, 0) =  ctheta * cpsi - sphi * stheta * spsi;
  RMAT_ELMT(*rm, 0, 1) =  ctheta * spsi + sphi * stheta * cpsi;
  RMAT_ELMT(*rm, 0, 2) = -cphi * stheta;
  RMAT_ELMT(*rm, 1, 0) = -cphi * spsi;
  RMAT_ELMT(*rm, 1, 1) =  cphi * cpsi;
  RMAT_ELMT(*rm, 1, 2) =  sphi;
  RMAT_ELMT(*rm, 2, 0) =  stheta * cpsi + sphi * ctheta * spsi;
  RMAT_ELMT(*rm, 2, 1) =  stheta * spsi - sphi * ctheta * cpsi;
  RMAT_ELMT(*rm, 2, 2) =  cphi * ctheta;
}


/* C n->b rotation matrix */
void float_rmat_of_quat(struct FloatRMat *rm, struct FloatQuat *q)
{
  const float _a = M_SQRT2 * q->qi;
  const float _b = M_SQRT2 * q->qx;
  const float _c = M_SQRT2 * q->qy;
  const float _d = M_SQRT2 * q->qz;
  const float a2_1 = _a * _a - 1;
  const float ab = _a * _b;
  const float ac = _a * _c;
  const float ad = _a * _d;
  const float bc = _b * _c;
  const float bd = _b * _d;
  const float cd = _c * _d;
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

/** in place first order integration of a rotation matrix */
void float_rmat_integrate_fi(struct FloatRMat *rm, struct FloatRates *omega, float dt)
{
  struct FloatRMat exp_omega_dt = {
    {
      1.        ,  dt *omega->r, -dt *omega->q,
      -dt *omega->r,  1.        ,  dt *omega->p,
      dt *omega->q, -dt *omega->p,  1.
    }
  };
  struct FloatRMat R_tdt;
  float_rmat_comp(&R_tdt, rm, &exp_omega_dt);
  memcpy(rm, &R_tdt, sizeof(R_tdt));
}

static inline float renorm_factor(float n)
{
  if (n < 1.5625f && n > 0.64f) {
    return .5 * (3 - n);
  } else if (n < 100.0f && n > 0.01f) {
    return  1. / sqrtf(n);
  } else {
    return 0.;
  }
}

float float_rmat_reorthogonalize(struct FloatRMat *rm)
{
  const struct FloatVect3 r0 = {RMAT_ELMT(*rm, 0, 0),
          RMAT_ELMT(*rm, 0, 1),
          RMAT_ELMT(*rm, 0, 2)
  };
  const struct FloatVect3 r1 = {RMAT_ELMT(*rm, 1, 0),
          RMAT_ELMT(*rm, 1, 1),
          RMAT_ELMT(*rm, 1, 2)
  };
  float _err = -0.5 * VECT3_DOT_PRODUCT(r0, r1);
  struct FloatVect3 r0_t;
  VECT3_SUM_SCALED(r0_t, r0, r1, _err);
  struct FloatVect3 r1_t;
  VECT3_SUM_SCALED(r1_t,  r1, r0, _err);
  struct FloatVect3 r2_t;
  VECT3_CROSS_PRODUCT(r2_t, r0_t, r1_t);
  float s = renorm_factor(VECT3_NORM2(r0_t));
  MAT33_ROW_VECT3_SMUL(*rm, 0, r0_t, s);
  s = renorm_factor(VECT3_NORM2(r1_t));
  MAT33_ROW_VECT3_SMUL(*rm, 1, r1_t, s);
  s = renorm_factor(VECT3_NORM2(r2_t));
  MAT33_ROW_VECT3_SMUL(*rm, 2, r2_t, s);

  return _err;
}


/*
 *
 * Quaternion functions.
 *
 */

void float_quat_comp(struct FloatQuat *a2c, struct FloatQuat *a2b, struct FloatQuat *b2c)
{
  a2c->qi = a2b->qi * b2c->qi - a2b->qx * b2c->qx - a2b->qy * b2c->qy - a2b->qz * b2c->qz;
  a2c->qx = a2b->qi * b2c->qx + a2b->qx * b2c->qi + a2b->qy * b2c->qz - a2b->qz * b2c->qy;
  a2c->qy = a2b->qi * b2c->qy - a2b->qx * b2c->qz + a2b->qy * b2c->qi + a2b->qz * b2c->qx;
  a2c->qz = a2b->qi * b2c->qz + a2b->qx * b2c->qy - a2b->qy * b2c->qx + a2b->qz * b2c->qi;
}

void float_quat_comp_inv(struct FloatQuat *a2b, struct FloatQuat *a2c, struct FloatQuat *b2c)
{
  a2b->qi =  a2c->qi * b2c->qi + a2c->qx * b2c->qx + a2c->qy * b2c->qy + a2c->qz * b2c->qz;
  a2b->qx = -a2c->qi * b2c->qx + a2c->qx * b2c->qi - a2c->qy * b2c->qz + a2c->qz * b2c->qy;
  a2b->qy = -a2c->qi * b2c->qy + a2c->qx * b2c->qz + a2c->qy * b2c->qi - a2c->qz * b2c->qx;
  a2b->qz = -a2c->qi * b2c->qz - a2c->qx * b2c->qy + a2c->qy * b2c->qx + a2c->qz * b2c->qi;
}

void float_quat_inv_comp(struct FloatQuat *b2c, struct FloatQuat *a2b, struct FloatQuat *a2c)
{
  b2c->qi = a2b->qi * a2c->qi + a2b->qx * a2c->qx + a2b->qy * a2c->qy + a2b->qz * a2c->qz;
  b2c->qx = a2b->qi * a2c->qx - a2b->qx * a2c->qi - a2b->qy * a2c->qz + a2b->qz * a2c->qy;
  b2c->qy = a2b->qi * a2c->qy + a2b->qx * a2c->qz - a2b->qy * a2c->qi - a2b->qz * a2c->qx;
  b2c->qz = a2b->qi * a2c->qz - a2b->qx * a2c->qy + a2b->qy * a2c->qx - a2b->qz * a2c->qi;
}

void float_quat_comp_norm_shortest(struct FloatQuat *a2c, struct FloatQuat *a2b, struct FloatQuat *b2c)
{
  float_quat_comp(a2c, a2b, b2c);
  float_quat_wrap_shortest(a2c);
  float_quat_normalize(a2c);
}

void float_quat_comp_inv_norm_shortest(struct FloatQuat *a2b, struct FloatQuat *a2c, struct FloatQuat *b2c)
{
  float_quat_comp_inv(a2b, a2c, b2c);
  float_quat_wrap_shortest(a2b);
  float_quat_normalize(a2b);
}

void float_quat_inv_comp_norm_shortest(struct FloatQuat *b2c, struct FloatQuat *a2b, struct FloatQuat *a2c)
{
  float_quat_inv_comp(b2c, a2b, a2c);
  float_quat_wrap_shortest(b2c);
  float_quat_normalize(b2c);
}

void float_quat_differential(struct FloatQuat *q_out, struct FloatRates *w, float dt)
{
  const float v_norm = sqrtf(w->p * w->p + w->q * w->q + w->r * w->r);
  const float c2 = cos(dt * v_norm / 2.0);
  const float s2 = sin(dt * v_norm / 2.0);
  if (v_norm < 1e-8) {
    q_out->qi = 1;
    q_out->qx = 0;
    q_out->qy = 0;
    q_out->qz = 0;
  } else {
    q_out->qi = c2;
    q_out->qx = w->p / v_norm * s2;
    q_out->qy = w->q / v_norm * s2;
    q_out->qz = w->r / v_norm * s2;
  }
}

/** in place first order quaternion integration with constant rotational velocity */
void float_quat_integrate_fi(struct FloatQuat *q, struct FloatRates *omega, float dt)
{
  const float qi = q->qi;
  const float qx = q->qx;
  const float qy = q->qy;
  const float qz = q->qz;
  const float dp = 0.5 * dt * omega->p;
  const float dq = 0.5 * dt * omega->q;
  const float dr = 0.5 * dt * omega->r;
  q->qi = qi    - dp * qx - dq * qy - dr * qz;
  q->qx = dp * qi +    qx + dr * qy - dq * qz;
  q->qy = dq * qi - dr * qx +    qy + dp * qz;
  q->qz = dr * qi + dq * qx - dp * qy +    qz;
}

/** in place quaternion integration with constant rotational velocity */
void float_quat_integrate(struct FloatQuat *q, struct FloatRates *omega, float dt)
{
  const float no = FLOAT_RATES_NORM(*omega);
  if (no > FLT_MIN) {
    const float a  = 0.5 * no * dt;
    const float ca = cosf(a);
    const float sa_ov_no = sinf(a) / no;
    const float dp = sa_ov_no * omega->p;
    const float dq = sa_ov_no * omega->q;
    const float dr = sa_ov_no * omega->r;
    const float qi = q->qi;
    const float qx = q->qx;
    const float qy = q->qy;
    const float qz = q->qz;
    q->qi = ca * qi - dp * qx - dq * qy - dr * qz;
    q->qx = dp * qi + ca * qx + dr * qy - dq * qz;
    q->qy = dq * qi - dr * qx + ca * qy + dp * qz;
    q->qz = dr * qi + dq * qx - dp * qy + ca * qz;
  }
}

void float_quat_vmult(struct FloatVect3 *v_out, struct FloatQuat *q, const struct FloatVect3 *v_in)
{
  const float qi2_M1_2  = q->qi * q->qi - 0.5;
  const float qiqx = q->qi * q->qx;
  const float qiqy = q->qi * q->qy;
  const float qiqz = q->qi * q->qz;
  float m01  = q->qx * q->qy; /* aka qxqy */
  float m02  = q->qx * q->qz; /* aka qxqz */
  float m12  = q->qy * q->qz; /* aka qyqz */

  const float m00  = qi2_M1_2 + q->qx * q->qx;
  const float m10  = m01 - qiqz;
  const float m20  = m02 + qiqy;
  const float m21  = m12 - qiqx;
  m01 += qiqz;
  m02 -= qiqy;
  m12 += qiqx;
  const float m11  = qi2_M1_2 + q->qy * q->qy;
  const float m22  = qi2_M1_2 + q->qz * q->qz;
  v_out->x = 2 * (m00 * v_in->x + m01 * v_in->y + m02 * v_in->z);
  v_out->y = 2 * (m10 * v_in->x + m11 * v_in->y + m12 * v_in->z);
  v_out->z = 2 * (m20 * v_in->x + m21 * v_in->y + m22 * v_in->z);
}

/** Quaternion derivative from rotational velocity.
 * qd = -0.5*omega(r) * q
 * or equally:
 * qd = 0.5 * q * omega(r)
 */
void float_quat_derivative(struct FloatQuat *qd, struct FloatRates *r, struct FloatQuat *q)
{
  qd->qi = -0.5 * (r->p * q->qx + r->q * q->qy + r->r * q->qz);
  qd->qx = -0.5 * (-r->p * q->qi - r->r * q->qy + r->q * q->qz);
  qd->qy = -0.5 * (-r->q * q->qi + r->r * q->qx - r->p * q->qz);
  qd->qz = -0.5 * (-r->r * q->qi - r->q * q->qx + r->p * q->qy);
}

/** Quaternion derivative from rotational velocity.
 * qd = -0.5*omega(r) * q
 */
void float_quat_derivative_lagrange(struct FloatQuat *qd, struct FloatRates *r, struct FloatQuat *q)
{
  const float K_LAGRANGE = 1.;
  const float c = K_LAGRANGE * (1 - float_quat_norm(q)) / -0.5;
  qd->qi = -0.5 * (c * q->qi + r->p * q->qx + r->q * q->qy + r->r * q->qz);
  qd->qx = -0.5 * (-r->p * q->qi +      c * q->qx - r->r * q->qy + r->q * q->qz);
  qd->qy = -0.5 * (-r->q * q->qi + r->r * q->qx +      c * q->qy - r->p * q->qz);
  qd->qz = -0.5 * (-r->r * q->qi - r->q * q->qx + r->p * q->qy +      c * q->qz);
}

/**
 * @brief quat of euler roation 'ZYX'
 *
 * @param q Quat output
 * @param e Euler input
 */
void float_quat_of_eulers(struct FloatQuat *q, struct FloatEulers *e)
{

  const float phi2   = e->phi / 2.f;
  const float theta2 = e->theta / 2.f;
  const float psi2   = e->psi / 2.f;

  const float s_phi2   = sinf(phi2);
  const float c_phi2   = cosf(phi2);
  const float s_theta2 = sinf(theta2);
  const float c_theta2 = cosf(theta2);
  const float s_psi2   = sinf(psi2);
  const float c_psi2   = cosf(psi2);

  q->qi =  c_phi2 * c_theta2 * c_psi2 + s_phi2 * s_theta2 * s_psi2;
  q->qx = -c_phi2 * s_theta2 * s_psi2 + s_phi2 * c_theta2 * c_psi2;
  q->qy =  c_phi2 * s_theta2 * c_psi2 + s_phi2 * c_theta2 * s_psi2;
  q->qz =  c_phi2 * c_theta2 * s_psi2 - s_phi2 * s_theta2 * c_psi2;
}

/**
 * @brief quat from euler rotation 'ZXY'
 * This rotation order is useful if you need 90 deg pitch
 *
 * @param q Quat output
 * @param e Euler input
 */
void float_quat_of_eulers_zxy(struct FloatQuat *q, struct FloatEulers *e)
{
  const float phi2   = e->phi / 2.f;
  const float theta2 = e->theta / 2.f;
  const float psi2   = e->psi / 2.f;

  const float s_phi2   = sinf(phi2);
  const float c_phi2   = cosf(phi2);
  const float s_theta2 = sinf(theta2);
  const float c_theta2 = cosf(theta2);
  const float s_psi2   = sinf(psi2);
  const float c_psi2   = cosf(psi2);

  q->qi =  c_phi2 * c_theta2 * c_psi2 - s_phi2 * s_theta2 * s_psi2;
  q->qx =  s_phi2 * c_theta2 * c_psi2 - c_phi2 * s_theta2 * s_psi2;
  q->qy =  c_phi2 * s_theta2 * c_psi2 + s_phi2 * c_theta2 * s_psi2;
  q->qz =  s_phi2 * s_theta2 * c_psi2 + c_phi2 * c_theta2 * s_psi2;
}

/**
 * @brief quat from euler rotation 'YXZ'
 * This function calculates a quaternion from Euler angles with the order YXZ,
 * so pitch, roll, yaw, instead of the conventional ZYX order.
 * See https://en.wikipedia.org/wiki/Euler_angles
 *
 * @param q Quat output
 * @param e Euler input
 */
void float_quat_of_eulers_yxz(struct FloatQuat *q, struct FloatEulers *e)
{
  const float phi2   = e->phi / 2.f;
  const float theta2 = e->theta / 2.f;
  const float psi2   = e->psi / 2.f;

  const float s_phi2   = sinf(phi2);
  const float c_phi2   = cosf(phi2);
  const float s_theta2 = sinf(theta2);
  const float c_theta2 = cosf(theta2);
  const float s_psi2   = sinf(psi2);
  const float c_psi2   = cosf(psi2);

  q->qi =  c_theta2 * c_phi2 * c_psi2 + s_theta2 * s_phi2 * s_psi2;
  q->qx =  c_theta2 * s_phi2 * c_psi2 + s_theta2 * c_phi2 * s_psi2;
  q->qy =  s_theta2 * c_phi2 * c_psi2 - c_theta2 * s_phi2 * s_psi2;
  q->qz =  c_theta2 * c_phi2 * s_psi2 - s_theta2 * s_phi2 * c_psi2;
}

void float_quat_of_axis_angle(struct FloatQuat *q, const struct FloatVect3 *uv, float angle)
{
  const float san = sinf(angle / 2.f);
  q->qi = cosf(angle / 2.f);
  q->qx = san * uv->x;
  q->qy = san * uv->y;
  q->qz = san * uv->z;
}

void float_quat_of_orientation_vect(struct FloatQuat *q, const struct FloatVect3 *ov)
{
  const float ov_norm = sqrtf(ov->x * ov->x + ov->y * ov->y + ov->z * ov->z);
  if (ov_norm < 1e-8) {
    q->qi = 1;
    q->qx = 0;
    q->qy = 0;
    q->qz = 0;
  } else {
    const float s2_normalized = sinf(ov_norm / 2.0) / ov_norm;
    q->qi = cosf(ov_norm / 2.0);
    q->qx = ov->x * s2_normalized;
    q->qy = ov->y * s2_normalized;
    q->qz = ov->z * s2_normalized;
  }
}

void float_quat_of_rmat(struct FloatQuat *q, struct FloatRMat *rm)
{
  const float tr = RMAT_TRACE(*rm);
  if (tr > 0) {
    const float two_qi = sqrtf(1. + tr);
    const float four_qi = 2. * two_qi;
    q->qi = 0.5 * two_qi;
    q->qx = (RMAT_ELMT(*rm, 1, 2) - RMAT_ELMT(*rm, 2, 1)) / four_qi;
    q->qy = (RMAT_ELMT(*rm, 2, 0) - RMAT_ELMT(*rm, 0, 2)) / four_qi;
    q->qz = (RMAT_ELMT(*rm, 0, 1) - RMAT_ELMT(*rm, 1, 0)) / four_qi;
    /*printf("tr > 0\n");*/
  } else {
    if (RMAT_ELMT(*rm, 0, 0) > RMAT_ELMT(*rm, 1, 1) &&
        RMAT_ELMT(*rm, 0, 0) > RMAT_ELMT(*rm, 2, 2)) {
      const float two_qx = sqrtf(RMAT_ELMT(*rm, 0, 0) - RMAT_ELMT(*rm, 1, 1)
                                 - RMAT_ELMT(*rm, 2, 2) + 1);
      const float four_qx = 2. * two_qx;
      q->qi = (RMAT_ELMT(*rm, 1, 2) - RMAT_ELMT(*rm, 2, 1)) / four_qx;
      q->qx = 0.5 * two_qx;
      q->qy = (RMAT_ELMT(*rm, 0, 1) + RMAT_ELMT(*rm, 1, 0)) / four_qx;
      q->qz = (RMAT_ELMT(*rm, 2, 0) + RMAT_ELMT(*rm, 0, 2)) / four_qx;
      /*printf("m00 largest\n");*/
    } else if (RMAT_ELMT(*rm, 1, 1) > RMAT_ELMT(*rm, 2, 2)) {
      const float two_qy =
        sqrtf(RMAT_ELMT(*rm, 1, 1) - RMAT_ELMT(*rm, 0, 0) - RMAT_ELMT(*rm, 2, 2) + 1);
      const float four_qy = 2. * two_qy;
      q->qi = (RMAT_ELMT(*rm, 2, 0) - RMAT_ELMT(*rm, 0, 2)) / four_qy;
      q->qx = (RMAT_ELMT(*rm, 0, 1) + RMAT_ELMT(*rm, 1, 0)) / four_qy;
      q->qy = 0.5 * two_qy;
      q->qz = (RMAT_ELMT(*rm, 1, 2) + RMAT_ELMT(*rm, 2, 1)) / four_qy;
      /*printf("m11 largest\n");*/
    } else {
      const float two_qz =
        sqrtf(RMAT_ELMT(*rm, 2, 2) - RMAT_ELMT(*rm, 0, 0) - RMAT_ELMT(*rm, 1, 1) + 1);
      const float four_qz = 2. * two_qz;
      q->qi = (RMAT_ELMT(*rm, 0, 1) - RMAT_ELMT(*rm, 1, 0)) / four_qz;
      q->qx = (RMAT_ELMT(*rm, 2, 0) + RMAT_ELMT(*rm, 0, 2)) / four_qz;
      q->qy = (RMAT_ELMT(*rm, 1, 2) + RMAT_ELMT(*rm, 2, 1)) / four_qz;
      q->qz = 0.5 * two_qz;
      /*printf("m22 largest\n");*/
    }
  }
}


/*
 *
 * Euler angle functions.
 *
 */

void float_eulers_of_rmat(struct FloatEulers *e, struct FloatRMat *rm)
{
  const float dcm00 = rm->m[0];
  const float dcm01 = rm->m[1];
  const float dcm02 = rm->m[2];
  const float dcm12 = rm->m[5];
  const float dcm22 = rm->m[8];
  e->phi   = atan2f(dcm12, dcm22);
  e->theta = -asinf(dcm02);
  e->psi   = atan2f(dcm01, dcm00);
}

/**
 * @brief euler rotation 'ZYX'
 *
 * @param e Euler output
 * @param q Quat input
 */
void float_eulers_of_quat(struct FloatEulers *e, struct FloatQuat *q)
{
  const float qx2  = q->qx * q->qx;
  const float qy2  = q->qy * q->qy;
  const float qz2  = q->qz * q->qz;
  const float qiqx = q->qi * q->qx;
  const float qiqy = q->qi * q->qy;
  const float qiqz = q->qi * q->qz;
  const float qxqy = q->qx * q->qy;
  const float qxqz = q->qx * q->qz;
  const float qyqz = q->qy * q->qz;
  const float dcm00 = 1.0 - 2.*(qy2 +  qz2);
  const float dcm01 =       2.*(qxqy + qiqz);
  const float dcm02 =       2.*(qxqz - qiqy);
  const float dcm12 =       2.*(qyqz + qiqx);
  const float dcm22 = 1.0 - 2.*(qx2 +  qy2);

  e->phi = atan2f(dcm12, dcm22);
  e->theta = -asinf(dcm02);
  e->psi = atan2f(dcm01, dcm00);
}

/**
 * @brief euler rotation 'YXZ'
 * This function calculates from a quaternion the Euler angles with the order YXZ,
 * so pitch, roll, yaw, instead of the conventional ZYX order.
 * See https://en.wikipedia.org/wiki/Euler_angles
 *
 * @param e Euler output
 * @param q Quat input
 */
void float_eulers_of_quat_yxz(struct FloatEulers *e, struct FloatQuat *q)
{
  const float qx2  = q->qx * q->qx;
  const float qy2  = q->qy * q->qy;
  const float qz2  = q->qz * q->qz;
  const float qi2  = q->qi * q->qi;
  const float qiqx = q->qi * q->qx;
  const float qiqy = q->qi * q->qy;
  const float qiqz = q->qi * q->qz;
  const float qxqy = q->qx * q->qy;
  const float qxqz = q->qx * q->qz;
  const float qyqz = q->qy * q->qz;
  const float r11  = 2.f * (qxqz + qiqy);
  const float r12  = qi2 - qx2 + qy2 + qz2;
  const float r21  = -2.f * (qyqz - qiqx);
  const float r31  = 2.f * (qxqy + qiqz);
  const float r32  = qi2 - qx2 + qy2 - qz2;

  e->theta = atan2f(r11, r12);
  e->phi = asinf(r21);
  e->psi = atan2f(r31, r32);
}

/**
 * @brief euler rotation 'ZXY'
 * This rotation order is useful if you need 90 deg pitch
 *
 * @param e Euler output
 * @param q Quat input
 */
void float_eulers_of_quat_zxy(struct FloatEulers *e, struct FloatQuat *q)
{
  const float qx2  = q->qx * q->qx;
  const float qy2  = q->qy * q->qy;
  const float qz2  = q->qz * q->qz;
  const float qi2  = q->qi * q->qi;
  const float qiqx = q->qi * q->qx;
  const float qiqy = q->qi * q->qy;
  const float qiqz = q->qi * q->qz;
  const float qxqy = q->qx * q->qy;
  const float qxqz = q->qx * q->qz;
  const float qyqz = q->qy * q->qz;
  const float r11  = -2 * (qxqy - qiqz);
  const float r12  = qi2 - qx2 + qy2 - qz2;
  const float r21  =  2 * (qyqz + qiqx);
  const float r31  = -2 * (qxqz - qiqy);
  const float r32  = qi2 - qx2 - qy2 + qz2;

  e->psi = atan2f(r11, r12);
  e->phi = asinf(r21);
  e->theta = atan2f(r31, r32);
}

/**
 * @brief 2x2 matrix inverse
 *
 * @param inv_out[4] inverted matrix output
 * @param mat_in[4] matrix to be inverted
 *
 * @return success (0) or not invertible (1)
 */
bool float_mat_inv_2d(float inv_out[4], float mat_in[4])
{
  float det = mat_in[0] * mat_in[3] - mat_in[1] * mat_in[2];

  if (fabsf(det) < 1e-4) { return 1; } //not invertible

  inv_out[0] =  mat_in[3] / det;
  inv_out[1] = -mat_in[1] / det;
  inv_out[2] = -mat_in[2] / det;
  inv_out[3] =  mat_in[0] / det;

  return 0; //return success
}

/**
 * @brief Multiply 2D matrix with vector
 *
 * @param vect_out output vector
 * @param mat[4] Matrix input
 * @param vect_in Vector input
 */
void float_mat2_mult(struct FloatVect2 *vect_out, float mat[4], struct FloatVect2 vect_in)
{
  vect_out->x = mat[0] * vect_in.x + mat[1] * vect_in.y;
  vect_out->y = mat[2] * vect_in.x + mat[3] * vect_in.y;
}

/*
 * 4x4 Matrix inverse.
 * obtained from: http://rodolphe-vaillant.fr/?e=7
 */

static float float_mat_minor_4d(float m[16], int r0, int r1, int r2, int c0, int c1, int c2)
{
  return m[4 * r0 + c0] * (m[4 * r1 + c1] * m[4 * r2 + c2] - m[4 * r2 + c1] * m[4 * r1 + c2]) -
         m[4 * r0 + c1] * (m[4 * r1 + c0] * m[4 * r2 + c2] - m[4 * r2 + c0] * m[4 * r1 + c2]) +
         m[4 * r0 + c2] * (m[4 * r1 + c0] * m[4 * r2 + c1] - m[4 * r2 + c0] * m[4 * r1 + c1]);
}


static void float_mat_adjoint_4d(float adjOut[16], float m[16])
{
  adjOut[ 0] =  float_mat_minor_4d(m, 1, 2, 3, 1, 2, 3);
  adjOut[ 1] = -float_mat_minor_4d(m, 0, 2, 3, 1, 2, 3);
  adjOut[ 2] =  float_mat_minor_4d(m, 0, 1, 3, 1, 2, 3);
  adjOut[ 3] = -float_mat_minor_4d(m, 0, 1, 2, 1, 2, 3);
  adjOut[ 4] = -float_mat_minor_4d(m, 1, 2, 3, 0, 2, 3);
  adjOut[ 5] =  float_mat_minor_4d(m, 0, 2, 3, 0, 2, 3);
  adjOut[ 6] = -float_mat_minor_4d(m, 0, 1, 3, 0, 2, 3);
  adjOut[ 7] =  float_mat_minor_4d(m, 0, 1, 2, 0, 2, 3);
  adjOut[ 8] =  float_mat_minor_4d(m, 1, 2, 3, 0, 1, 3);
  adjOut[ 9] = -float_mat_minor_4d(m, 0, 2, 3, 0, 1, 3);
  adjOut[10] =  float_mat_minor_4d(m, 0, 1, 3, 0, 1, 3);
  adjOut[11] = -float_mat_minor_4d(m, 0, 1, 2, 0, 1, 3);
  adjOut[12] = -float_mat_minor_4d(m, 1, 2, 3, 0, 1, 2);
  adjOut[13] =  float_mat_minor_4d(m, 0, 2, 3, 0, 1, 2);
  adjOut[14] = -float_mat_minor_4d(m, 0, 1, 3, 0, 1, 2);
  adjOut[15] =  float_mat_minor_4d(m, 0, 1, 2, 0, 1, 2);
}

static float float_mat_det_4d(float m[16])
{
  return m[0] * float_mat_minor_4d(m, 1, 2, 3, 1, 2, 3) -
         m[1] * float_mat_minor_4d(m, 1, 2, 3, 0, 2, 3) +
         m[2] * float_mat_minor_4d(m, 1, 2, 3, 0, 1, 3) -
         m[3] * float_mat_minor_4d(m, 1, 2, 3, 0, 1, 2);
}

/**
 * 4x4 Matrix inverse
 *
 * @param invOut output array, inverse of mat_in
 * @param mat_in input array
 */
bool float_mat_inv_4d(float invOut[16], float mat_in[16])
{
  float_mat_adjoint_4d(invOut, mat_in);

  float det = float_mat_det_4d(mat_in);
  if (fabsf(det) < 1e-4) { return 1; } //not invertible

  float inv_det = 1.0f / det;
  int i;
  for (i = 0; i < 16; ++i) {
    invOut[i] = invOut[i] * inv_det;
  }

  return 0; //success
}

/** Calculate inverse of any n x n matrix (passed as C array) o = mat^-1
Algorithm verified with Matlab.
Thanks to: https://www.quora.com/How-do-I-make-a-C++-program-to-get-the-inverse-of-a-matrix-100-X-100
*/
void float_mat_invert(float **o, float **mat, int n)
{
  int i, j, k;
  float t;
  float a[n][2 * n];

  // Append an identity matrix on the right of the original matrix
  for (i = 0; i < n; i++) {
    for (j = 0; j < 2 * n; j++) {
      if (j < n) {
        a[i][j] = mat[i][j];
      } else if ((j >= n) && (j == i + n)) {
        a[i][j] = 1.0;
      } else {
        a[i][j] = 0.0;
      }
    }
  }

  // Do the inversion
  for (i = 0; i < n; i++) {
    t = a[i][i]; // Store diagonal variable (temp)

    for (j = i; j < 2 * n; j++) {
      a[i][j] = a[i][j] / t; // Divide by the diagonal value
    }

    for (j = 0; j < n; j++) {
      if (i != j) {
        t = a[j][i];
        for (k = 0; k < 2 * n; k++) {
          a[j][k] = a[j][k] - t * a[i][k];
        }
      }
    }
  }

  // Cut out the identity, which has now moved to the left side
  for (i = 0 ; i < n ; i++) {
    for (j = n; j < 2 * n; j++) {
      o[i][j - n] = a[i][j];
    }
  }
}

/*
 * o[n][n] = e^a[n][n]
 * Replicates expm(a) in Matlab
 *
 * Adapted from the following reference:
 * Cleve Moler, Charles VanLoan,
 * Nineteen Dubious Ways to Compute the Exponential of a Matrix,
 * Twenty-Five Years Later, SIAM Review, Volume 45, Number 1, March 2003, pages 3-49.
 * https://people.sc.fsu.edu/~jburkardt/c_src/matrix_exponential/matrix_exponential.c
 */
void float_mat_exp(float **a, float **o, int n)
{
  float a_norm, c, t;
  const int q = 6;
  float d[n][n];
  float x[n][n];
  float a_copy[n][n];
  int ee, k, s;
  int p;

  MAKE_MATRIX_PTR(_a,  a,  n);
  MAKE_MATRIX_PTR(_o,  o,  n);
  MAKE_MATRIX_PTR(_d,  d,  n);
  MAKE_MATRIX_PTR(_x,  x,  n);
  MAKE_MATRIX_PTR(_a_copy, a_copy, n);

  float_mat_copy(_a_copy, _a, n, n); // Make a copy of a to compute on
  a_norm = float_mat_norm_li(_a_copy, n, n);  // Compute the infinity norm of the matrix
  ee = (int)(float_log_n(a_norm, 2)) + 1;
  s = Max(0, ee + 1);
  t = 1.0 / powf(2.0, s);
  float_mat_scale(_a_copy, t, n, n);
  float_mat_copy(_x, _a_copy, n, n);  // x = a_copy
  c = 0.5;

  float_mat_diagonal_scal(_o, 1.0, n);  // make identiy
  float_mat_sum_scaled(_o, _a_copy, c, n, n);

  float_mat_diagonal_scal(_d, 1.0, n);
  float_mat_sum_scaled(_d, _a_copy, -c, n, n);

  p = 1;
  for (k = 2; k <= q; k++) {
    c = c * (float)(q - k + 1) / (float)(k * (2 * q - k + 1));
    float_mat_mul_copy(_x, _x, _a_copy, n, n, n);

    float_mat_sum_scaled(_o, _x, c, n, n);

    if (p) {
      float_mat_sum_scaled(_d, _x, c, n, n);
    } else {
      float_mat_sum_scaled(_d, _x, -c, n, n);
    }
    p = !p;
  }

  // E -> inverse(D) * E
  float temp[n][n];
  MAKE_MATRIX_PTR(_temp, temp, n);
  float_mat_invert(_temp, _d, n);
  float_mat_mul_copy(_o, _temp, _o, n, n, n);

  // E -> E^(2*S)
  for (k = 1; k <= s; k++) {
    float_mat_mul_copy(_o, _o, _o, n, n, n);
  }

}

/* Returns L-oo of matrix a */
float float_mat_norm_li(float **a, int m, int n)
{
  float row_sum;
  float value;

  value = 0.0;
  for (int i = 0; i < m; i++) {
    row_sum = 0.0;
    for (int j = 0; j < n; j++) {
      row_sum = row_sum + fabsf(a[i][j]);
    }
    value = Max(value, row_sum);
  }
  return value;
}
