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
 * @file pprz_algebra_int.c
 * @brief Paparazzi fixed point algebra.
 *
 */

#include "pprz_algebra_int.h"

#define INT32_SQRT_MAX_ITER 40
uint32_t int32_sqrt(uint32_t in)
{
  if (in == 0) {
    return 0;
  } else {
    uint32_t s1, s2;
    uint8_t iter = 0;
    s2 = in;
    do {
      s1 = s2;
      s2 = in / s1;
      s2 += s1;
      s2 /= 2;
      iter++;
    } while (((s1 - s2) > 1) && (iter < INT32_SQRT_MAX_ITER));
    return s2;
  }
}


/*
 *
 * Rotation matrices
 *
 */

/** Composition (multiplication) of two rotation matrices.
 * _m_a2c = _m_a2b comp _m_b2c , aka  _m_a2c = _m_b2c * _m_a2b
 */
void int32_rmat_comp(struct Int32RMat* m_a2c, struct Int32RMat* m_a2b, struct Int32RMat* m_b2c)
{
  m_a2c->m[0] = (m_b2c->m[0] * m_a2b->m[0] + m_b2c->m[1] * m_a2b->m[3] + m_b2c->m[2] * m_a2b->m[6]) >> INT32_TRIG_FRAC;
  m_a2c->m[1] = (m_b2c->m[0] * m_a2b->m[1] + m_b2c->m[1] * m_a2b->m[4] + m_b2c->m[2] * m_a2b->m[7]) >> INT32_TRIG_FRAC;
  m_a2c->m[2] = (m_b2c->m[0] * m_a2b->m[2] + m_b2c->m[1] * m_a2b->m[5] + m_b2c->m[2] * m_a2b->m[8]) >> INT32_TRIG_FRAC;
  m_a2c->m[3] = (m_b2c->m[3] * m_a2b->m[0] + m_b2c->m[4] * m_a2b->m[3] + m_b2c->m[5] * m_a2b->m[6]) >> INT32_TRIG_FRAC;
  m_a2c->m[4] = (m_b2c->m[3] * m_a2b->m[1] + m_b2c->m[4] * m_a2b->m[4] + m_b2c->m[5] * m_a2b->m[7]) >> INT32_TRIG_FRAC;
  m_a2c->m[5] = (m_b2c->m[3] * m_a2b->m[2] + m_b2c->m[4] * m_a2b->m[5] + m_b2c->m[5] * m_a2b->m[8]) >> INT32_TRIG_FRAC;
  m_a2c->m[6] = (m_b2c->m[6] * m_a2b->m[0] + m_b2c->m[7] * m_a2b->m[3] + m_b2c->m[8] * m_a2b->m[6]) >> INT32_TRIG_FRAC;
  m_a2c->m[7] = (m_b2c->m[6] * m_a2b->m[1] + m_b2c->m[7] * m_a2b->m[4] + m_b2c->m[8] * m_a2b->m[7]) >> INT32_TRIG_FRAC;
  m_a2c->m[8] = (m_b2c->m[6] * m_a2b->m[2] + m_b2c->m[7] * m_a2b->m[5] + m_b2c->m[8] * m_a2b->m[8]) >> INT32_TRIG_FRAC;
}

/** Composition (multiplication) of two rotation matrices.
 * _m_a2b = _m_a2c comp_inv _m_b2c , aka  _m_a2b = inv(_m_b2c) * _m_a2c
 */
void int32_rmat_comp_inv(struct Int32RMat* m_a2b, struct Int32RMat* m_a2c, struct Int32RMat* m_b2c)
{
  m_a2b->m[0] = (m_b2c->m[0] * m_a2c->m[0] + m_b2c->m[3] * m_a2c->m[3] + m_b2c->m[6] * m_a2c->m[6]) >> INT32_TRIG_FRAC;
  m_a2b->m[1] = (m_b2c->m[0] * m_a2c->m[1] + m_b2c->m[3] * m_a2c->m[4] + m_b2c->m[6] * m_a2c->m[7]) >> INT32_TRIG_FRAC;
  m_a2b->m[2] = (m_b2c->m[0] * m_a2c->m[2] + m_b2c->m[3] * m_a2c->m[5] + m_b2c->m[6] * m_a2c->m[8]) >> INT32_TRIG_FRAC;
  m_a2b->m[3] = (m_b2c->m[1] * m_a2c->m[0] + m_b2c->m[4] * m_a2c->m[3] + m_b2c->m[7] * m_a2c->m[6]) >> INT32_TRIG_FRAC;
  m_a2b->m[4] = (m_b2c->m[1] * m_a2c->m[1] + m_b2c->m[4] * m_a2c->m[4] + m_b2c->m[7] * m_a2c->m[7]) >> INT32_TRIG_FRAC;
  m_a2b->m[5] = (m_b2c->m[1] * m_a2c->m[2] + m_b2c->m[4] * m_a2c->m[5] + m_b2c->m[7] * m_a2c->m[8]) >> INT32_TRIG_FRAC;
  m_a2b->m[6] = (m_b2c->m[2] * m_a2c->m[0] + m_b2c->m[5] * m_a2c->m[3] + m_b2c->m[8] * m_a2c->m[6]) >> INT32_TRIG_FRAC;
  m_a2b->m[7] = (m_b2c->m[2] * m_a2c->m[1] + m_b2c->m[5] * m_a2c->m[4] + m_b2c->m[8] * m_a2c->m[7]) >> INT32_TRIG_FRAC;
  m_a2b->m[8] = (m_b2c->m[2] * m_a2c->m[2] + m_b2c->m[5] * m_a2c->m[5] + m_b2c->m[8] * m_a2c->m[8]) >> INT32_TRIG_FRAC;
}

/** rotate 3D vector by rotation matrix.
 * vb = m_a2b * va
 */
void int32_rmat_vmult(struct Int32Vect3* vb, struct Int32RMat* m_a2b, struct Int32Vect3* va)
{
  vb->x = (m_a2b->m[0] * va->x + m_a2b->m[1] * va->y + m_a2b->m[2] * va->z)>>INT32_TRIG_FRAC;
  vb->y = (m_a2b->m[3] * va->x + m_a2b->m[4] * va->y + m_a2b->m[5] * va->z)>>INT32_TRIG_FRAC;
  vb->z = (m_a2b->m[6] * va->x + m_a2b->m[7] * va->y + m_a2b->m[8] * va->z)>>INT32_TRIG_FRAC;
}

/** rotate 3D vector by transposed rotation matrix.
 * vb = m_b2a^T * va
 */
void int32_rmat_transp_vmult(struct Int32Vect3* vb, struct Int32RMat* m_b2a, struct Int32Vect3* va)
{
  vb->x = (m_b2a->m[0] * va->x + m_b2a->m[3] * va->y + m_b2a->m[6] * va->z)>>INT32_TRIG_FRAC;
  vb->y = (m_b2a->m[1] * va->x + m_b2a->m[4] * va->y + m_b2a->m[7] * va->z)>>INT32_TRIG_FRAC;
  vb->z = (m_b2a->m[2] * va->x + m_b2a->m[5] * va->y + m_b2a->m[8] * va->z)>>INT32_TRIG_FRAC;
}

/** rotate anglular rates by rotation matrix.
 * rb = m_a2b * ra
 */
void int32_rmat_ratemult(struct Int32Rates* rb, struct Int32RMat* m_a2b, struct Int32Rates* ra)
{
  rb->p = (m_a2b->m[0] * ra->p + m_a2b->m[1] * ra->q + m_a2b->m[2] * ra->r)>>INT32_TRIG_FRAC;
  rb->q = (m_a2b->m[3] * ra->p + m_a2b->m[4] * ra->q + m_a2b->m[5] * ra->r)>>INT32_TRIG_FRAC;
  rb->r = (m_a2b->m[6] * ra->p + m_a2b->m[7] * ra->q + m_a2b->m[8] * ra->r)>>INT32_TRIG_FRAC;
}

/** rotate anglular rates by transposed rotation matrix.
 * rb = m_b2a^T * ra
 */
void int32_rmat_transp_ratemult(struct Int32Rates* rb, struct Int32RMat* m_b2a, struct Int32Rates* ra)
{
  rb->p = (m_b2a->m[0] * ra->p + m_b2a->m[3] * ra->q + m_b2a->m[6] * ra->r)>>INT32_TRIG_FRAC;
  rb->q = (m_b2a->m[1] * ra->p + m_b2a->m[4] * ra->q + m_b2a->m[7] * ra->r)>>INT32_TRIG_FRAC;
  rb->r = (m_b2a->m[2] * ra->p + m_b2a->m[5] * ra->q + m_b2a->m[8] * ra->r)>>INT32_TRIG_FRAC;
}


/** Convert unit quaternion to rotation matrix.
 * http://www.mathworks.com/access/helpdesk_r13/help/toolbox/aeroblks/quaternionstodirectioncosinematrix.html
 */
void int32_rmat_of_quat(struct Int32RMat* rm, struct Int32Quat* q)
{
  const int32_t _2qi2_m1  = INT_MULT_RSHIFT(q->qi, q->qi,
                            INT32_QUAT_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC - 1) - TRIG_BFP_OF_REAL(1);
  rm->m[0] = INT_MULT_RSHIFT(q->qx, q->qx, INT32_QUAT_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC - 1);
  rm->m[4] = INT_MULT_RSHIFT(q->qy, q->qy, INT32_QUAT_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC - 1);
  rm->m[8] = INT_MULT_RSHIFT(q->qz, q->qz, INT32_QUAT_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC - 1);

  const int32_t _2qiqx = INT_MULT_RSHIFT(q->qi, q->qx, INT32_QUAT_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC - 1);
  const int32_t _2qiqy = INT_MULT_RSHIFT(q->qi, q->qy, INT32_QUAT_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC - 1);
  const int32_t _2qiqz = INT_MULT_RSHIFT(q->qi, q->qz, INT32_QUAT_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC - 1);
  rm->m[1] = INT_MULT_RSHIFT(q->qx, q->qy, INT32_QUAT_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC - 1);
  rm->m[2] = INT_MULT_RSHIFT(q->qx, q->qz, INT32_QUAT_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC - 1);
  rm->m[5] = INT_MULT_RSHIFT(q->qy, q->qz, INT32_QUAT_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC - 1);
  rm->m[0] += _2qi2_m1;
  rm->m[3] = rm->m[1] - _2qiqz;
  rm->m[6] = rm->m[2] + _2qiqy;
  rm->m[7] = rm->m[5] - _2qiqx;
  rm->m[4] += _2qi2_m1;
  rm->m[1] += _2qiqz;
  rm->m[2] -= _2qiqy;
  rm->m[5] += _2qiqx;
  rm->m[8] += _2qi2_m1;
}


/** Rotation matrix from 321 Euler angles.
 * http://www.mathworks.com/access/helpdesk_r13/help/toolbox/aeroblks/euleranglestodirectioncosinematrix.html
 */
void int32_rmat_of_eulers_321(struct Int32RMat* rm, struct Int32Eulers* e)
{
  int32_t sphi;
  PPRZ_ITRIG_SIN(sphi, e->phi);
  int32_t cphi;
  PPRZ_ITRIG_COS(cphi, e->phi);
  int32_t stheta;
  PPRZ_ITRIG_SIN(stheta, e->theta);
  int32_t ctheta;
  PPRZ_ITRIG_COS(ctheta, e->theta);
  int32_t spsi;
  PPRZ_ITRIG_SIN(spsi, e->psi);
  int32_t cpsi;
  PPRZ_ITRIG_COS(cpsi, e->psi);

  int32_t ctheta_cpsi = INT_MULT_RSHIFT(ctheta, cpsi,   INT32_TRIG_FRAC);
  int32_t ctheta_spsi = INT_MULT_RSHIFT(ctheta, spsi,   INT32_TRIG_FRAC);
  int32_t cphi_spsi   = INT_MULT_RSHIFT(cphi,   spsi,   INT32_TRIG_FRAC);
  int32_t cphi_cpsi   = INT_MULT_RSHIFT(cphi,   cpsi,   INT32_TRIG_FRAC);
  int32_t cphi_ctheta = INT_MULT_RSHIFT(cphi,   ctheta, INT32_TRIG_FRAC);
  int32_t cphi_stheta = INT_MULT_RSHIFT(cphi,   stheta, INT32_TRIG_FRAC);
  int32_t sphi_ctheta = INT_MULT_RSHIFT(sphi,   ctheta, INT32_TRIG_FRAC);
  int32_t sphi_stheta = INT_MULT_RSHIFT(sphi,   stheta, INT32_TRIG_FRAC);
  int32_t sphi_spsi   = INT_MULT_RSHIFT(sphi,   spsi,   INT32_TRIG_FRAC);
  int32_t sphi_cpsi   = INT_MULT_RSHIFT(sphi,   cpsi,   INT32_TRIG_FRAC);

  int32_t sphi_stheta_cpsi = INT_MULT_RSHIFT(sphi_stheta, cpsi, INT32_TRIG_FRAC);
  int32_t sphi_stheta_spsi = INT_MULT_RSHIFT(sphi_stheta, spsi, INT32_TRIG_FRAC);
  int32_t cphi_stheta_cpsi = INT_MULT_RSHIFT(cphi_stheta, cpsi, INT32_TRIG_FRAC);
  int32_t cphi_stheta_spsi = INT_MULT_RSHIFT(cphi_stheta, spsi, INT32_TRIG_FRAC);

  RMAT_ELMT(*rm, 0, 0) = ctheta_cpsi;
  RMAT_ELMT(*rm, 0, 1) = ctheta_spsi;
  RMAT_ELMT(*rm, 0, 2) = -stheta;
  RMAT_ELMT(*rm, 1, 0) = sphi_stheta_cpsi - cphi_spsi;
  RMAT_ELMT(*rm, 1, 1) = sphi_stheta_spsi + cphi_cpsi;
  RMAT_ELMT(*rm, 1, 2) = sphi_ctheta;
  RMAT_ELMT(*rm, 2, 0) = cphi_stheta_cpsi + sphi_spsi;
  RMAT_ELMT(*rm, 2, 1) = cphi_stheta_spsi - sphi_cpsi;
  RMAT_ELMT(*rm, 2, 2) = cphi_ctheta;
}


void int32_rmat_of_eulers_312(struct Int32RMat* rm, struct Int32Eulers* e)
{
  int32_t sphi;
  PPRZ_ITRIG_SIN(sphi, e->phi);
  int32_t cphi;
  PPRZ_ITRIG_COS(cphi, e->phi);
  int32_t stheta;
  PPRZ_ITRIG_SIN(stheta, e->theta);
  int32_t ctheta;
  PPRZ_ITRIG_COS(ctheta, e->theta);
  int32_t spsi;
  PPRZ_ITRIG_SIN(spsi, e->psi);
  int32_t cpsi;
  PPRZ_ITRIG_COS(cpsi, e->psi);

  int32_t stheta_spsi = INT_MULT_RSHIFT(stheta, spsi,   INT32_TRIG_FRAC);
  int32_t stheta_cpsi = INT_MULT_RSHIFT(stheta, cpsi,   INT32_TRIG_FRAC);
  int32_t ctheta_spsi = INT_MULT_RSHIFT(ctheta, spsi,   INT32_TRIG_FRAC);
  int32_t ctheta_cpsi = INT_MULT_RSHIFT(ctheta, cpsi,   INT32_TRIG_FRAC);
  int32_t cphi_stheta = INT_MULT_RSHIFT(cphi,   stheta, INT32_TRIG_FRAC);
  int32_t cphi_ctheta = INT_MULT_RSHIFT(cphi,   ctheta, INT32_TRIG_FRAC);
  int32_t cphi_spsi   = INT_MULT_RSHIFT(cphi,   spsi,   INT32_TRIG_FRAC);
  int32_t cphi_cpsi   = INT_MULT_RSHIFT(cphi,   cpsi,   INT32_TRIG_FRAC);
  int32_t sphi_stheta = INT_MULT_RSHIFT(sphi,   stheta, INT32_TRIG_FRAC);
  int32_t sphi_ctheta = INT_MULT_RSHIFT(sphi,   ctheta, INT32_TRIG_FRAC);

  int32_t sphi_stheta_spsi = INT_MULT_RSHIFT(sphi_stheta, spsi, INT32_TRIG_FRAC);
  int32_t sphi_stheta_cpsi = INT_MULT_RSHIFT(sphi_stheta, cpsi, INT32_TRIG_FRAC);
  int32_t sphi_ctheta_spsi = INT_MULT_RSHIFT(sphi_ctheta, spsi, INT32_TRIG_FRAC);
  int32_t sphi_ctheta_cpsi = INT_MULT_RSHIFT(sphi_ctheta, cpsi, INT32_TRIG_FRAC);

  RMAT_ELMT(*rm, 0, 0) =  ctheta_cpsi - sphi_stheta_spsi;
  RMAT_ELMT(*rm, 0, 1) =  ctheta_spsi + sphi_stheta_cpsi;
  RMAT_ELMT(*rm, 0, 2) = -cphi_stheta;
  RMAT_ELMT(*rm, 1, 0) = -cphi_spsi;
  RMAT_ELMT(*rm, 1, 1) =  cphi_cpsi;
  RMAT_ELMT(*rm, 1, 2) =  sphi;
  RMAT_ELMT(*rm, 2, 0) =  stheta_cpsi + sphi_ctheta_spsi;
  RMAT_ELMT(*rm, 2, 1) =  stheta_spsi - sphi_ctheta_cpsi;
  RMAT_ELMT(*rm, 2, 2) =  cphi_ctheta;
}


/*
 *
 * Quaternions
 *
 */

void int32_quat_comp(struct Int32Quat* a2c, struct Int32Quat* a2b, struct Int32Quat* b2c)
{
  a2c->qi = (a2b->qi * b2c->qi - a2b->qx * b2c->qx - a2b->qy * b2c->qy - a2b->qz * b2c->qz) >> INT32_QUAT_FRAC;
  a2c->qx = (a2b->qi * b2c->qx + a2b->qx * b2c->qi + a2b->qy * b2c->qz - a2b->qz * b2c->qy) >> INT32_QUAT_FRAC;
  a2c->qy = (a2b->qi * b2c->qy - a2b->qx * b2c->qz + a2b->qy * b2c->qi + a2b->qz * b2c->qx) >> INT32_QUAT_FRAC;
  a2c->qz = (a2b->qi * b2c->qz + a2b->qx * b2c->qy - a2b->qy * b2c->qx + a2b->qz * b2c->qi) >> INT32_QUAT_FRAC;
}

void int32_quat_comp_inv(struct Int32Quat* a2b, struct Int32Quat* a2c, struct Int32Quat* b2c)
{
  a2b->qi = (a2c->qi * b2c->qi + a2c->qx * b2c->qx + a2c->qy * b2c->qy + a2c->qz * b2c->qz) >> INT32_QUAT_FRAC;
  a2b->qx = (-a2c->qi * b2c->qx + a2c->qx * b2c->qi - a2c->qy * b2c->qz + a2c->qz * b2c->qy) >> INT32_QUAT_FRAC;
  a2b->qy = (-a2c->qi * b2c->qy + a2c->qx * b2c->qz + a2c->qy * b2c->qi - a2c->qz * b2c->qx) >> INT32_QUAT_FRAC;
  a2b->qz = (-a2c->qi * b2c->qz - a2c->qx * b2c->qy + a2c->qy * b2c->qx + a2c->qz * b2c->qi) >> INT32_QUAT_FRAC;
}

void int32_quat_inv_comp(struct Int32Quat* b2c, struct Int32Quat* a2b, struct Int32Quat* a2c)
{
  b2c->qi = (a2b->qi * a2c->qi + a2b->qx * a2c->qx + a2b->qy * a2c->qy + a2b->qz * a2c->qz) >> INT32_QUAT_FRAC;
  b2c->qx = (a2b->qi * a2c->qx - a2b->qx * a2c->qi - a2b->qy * a2c->qz + a2b->qz * a2c->qy) >> INT32_QUAT_FRAC;
  b2c->qy = (a2b->qi * a2c->qy + a2b->qx * a2c->qz - a2b->qy * a2c->qi - a2b->qz * a2c->qx) >> INT32_QUAT_FRAC;
  b2c->qz = (a2b->qi * a2c->qz - a2b->qx * a2c->qy + a2b->qy * a2c->qx - a2b->qz * a2c->qi) >> INT32_QUAT_FRAC;
}

void int32_quat_comp_norm_shortest(struct Int32Quat* a2c, struct Int32Quat* a2b, struct Int32Quat* b2c)
{
  int32_quat_comp(a2c, a2b, b2c);
  int32_quat_wrap_shortest(a2c);
  int32_quat_normalize(a2c);
}

void int32_quat_comp_inv_norm_shortest(struct Int32Quat* a2b, struct Int32Quat* a2c, struct Int32Quat* b2c)
{
  int32_quat_comp_inv(a2b, a2c, b2c);
  int32_quat_wrap_shortest(a2b);
  int32_quat_normalize(a2b);
}

void int32_quat_inv_comp_norm_shortest(struct Int32Quat* b2c, struct Int32Quat* a2b, struct Int32Quat* a2c)
{
  int32_quat_inv_comp(b2c, a2b, a2c);
  int32_quat_wrap_shortest(b2c);
  int32_quat_normalize(b2c);
}

/** Quaternion derivative from rotational velocity.
 * qd = -0.5*omega(r) * q
 * or equally:
 * qd = 0.5 * q * omega(r)
 * Multiplication with 0.5 is done by shifting one more bit to the right.
 */
void int32_quat_derivative(struct Int32Quat* qd, const struct Int32Rates* r, struct Int32Quat* q)
{
  qd->qi = (-(r->p * q->qx + r->q * q->qy + r->r * q->qz)) >> (INT32_RATE_FRAC + 1);
  qd->qx = (-(-r->p * q->qi - r->r * q->qy + r->q * q->qz)) >> (INT32_RATE_FRAC + 1);
  qd->qy = (-(-r->q * q->qi + r->r * q->qx - r->p * q->qz)) >> (INT32_RATE_FRAC + 1);
  qd->qz = (-(-r->r * q->qi - r->q * q->qx + r->p * q->qy)) >> (INT32_RATE_FRAC + 1);
}

/** in place quaternion first order integration with constant rotational velocity. */
void int32_quat_integrate_fi(struct Int32Quat* q, struct Int64Quat* hr, struct Int32Rates* omega, int freq)
{
  hr->qi += - ((int64_t) omega->p) * q->qx - ((int64_t) omega->q) * q->qy - ((int64_t) omega->r) * q->qz;
  hr->qx += ((int64_t) omega->p) * q->qi + ((int64_t) omega->r) * q->qy - ((int64_t) omega->q) * q->qz;
  hr->qy += ((int64_t) omega->q) * q->qi - ((int64_t) omega->r) * q->qx + ((int64_t) omega->p) * q->qz;
  hr->qz += ((int64_t) omega->r) * q->qi + ((int64_t) omega->q) * q->qx - ((int64_t) omega->p) * q->qy;

  lldiv_t _div = lldiv(hr->qi, ((1 << INT32_RATE_FRAC) * freq * 2));
  q->qi += (int32_t) _div.quot;
  hr->qi = _div.rem;

  _div = lldiv(hr->qx, ((1 << INT32_RATE_FRAC) * freq * 2));
  q->qx += (int32_t) _div.quot;
  hr->qx = _div.rem;

  _div = lldiv(hr->qy, ((1 << INT32_RATE_FRAC) * freq * 2));
  q->qy += (int32_t) _div.quot;
  hr->qy = _div.rem;

  _div = lldiv(hr->qz, ((1 << INT32_RATE_FRAC) * freq * 2));
  q->qz += (int32_t) _div.quot;
  hr->qz = _div.rem;
}

void int32_quat_vmult(struct Int32Vect3* v_out, struct Int32Quat* q, struct Int32Vect3* v_in)
{
  const int32_t _2qi2_m1 = ((q->qi * q->qi) >> (INT32_QUAT_FRAC - 1)) - QUAT1_BFP_OF_REAL(1);
  const int32_t _2qx2    = (q->qx * q->qx) >> (INT32_QUAT_FRAC - 1);
  const int32_t _2qy2    = (q->qy * q->qy) >> (INT32_QUAT_FRAC - 1);
  const int32_t _2qz2    = (q->qz * q->qz) >> (INT32_QUAT_FRAC - 1);
  const int32_t _2qiqx   = (q->qi * q->qx) >> (INT32_QUAT_FRAC - 1);
  const int32_t _2qiqy   = (q->qi * q->qy) >> (INT32_QUAT_FRAC - 1);
  const int32_t _2qiqz   = (q->qi * q->qz) >> (INT32_QUAT_FRAC - 1);
  const int32_t m01 = ((q->qx * q->qy) >> (INT32_QUAT_FRAC - 1)) + _2qiqz;
  const int32_t m02 = ((q->qx * q->qz) >> (INT32_QUAT_FRAC - 1)) - _2qiqy;
  const int32_t m12 = ((q->qy * q->qz) >> (INT32_QUAT_FRAC - 1)) + _2qiqx;
  v_out->x = (_2qi2_m1 * v_in->x + _2qx2 * v_in->x + m01 * v_in->y +  m02 * v_in->z) >> INT32_QUAT_FRAC;
  v_out->y = (_2qi2_m1 * v_in->y + m01 * v_in->x - 2 * _2qiqz * v_in->x + _2qy2 * v_in->y + m12 * v_in->z) >>
             INT32_QUAT_FRAC;
  v_out->z = (_2qi2_m1 * v_in->z + m02 * v_in->x + 2 * _2qiqy * v_in->x + m12 * v_in->y - 2 * _2qiqx * v_in->y + _2qz2 *
              v_in->z) >> INT32_QUAT_FRAC;
}

/*
 * http://www.mathworks.com/access/helpdesk_r13/help/toolbox/aeroblks/euleranglestoquaternions.html
 */
void int32_quat_of_eulers(struct Int32Quat* q, struct Int32Eulers* e)
{
  const int32_t phi2   = e->phi   / 2;
  const int32_t theta2 = e->theta / 2;
  const int32_t psi2   = e->psi   / 2;

  int32_t s_phi2;
  PPRZ_ITRIG_SIN(s_phi2, phi2);
  int32_t c_phi2;
  PPRZ_ITRIG_COS(c_phi2, phi2);
  int32_t s_theta2;
  PPRZ_ITRIG_SIN(s_theta2, theta2);
  int32_t c_theta2;
  PPRZ_ITRIG_COS(c_theta2, theta2);
  int32_t s_psi2;
  PPRZ_ITRIG_SIN(s_psi2, psi2);
  int32_t c_psi2;
  PPRZ_ITRIG_COS(c_psi2, psi2);

  int32_t c_th_c_ps = INT_MULT_RSHIFT(c_theta2, c_psi2, INT32_TRIG_FRAC);
  int32_t c_th_s_ps = INT_MULT_RSHIFT(c_theta2, s_psi2, INT32_TRIG_FRAC);
  int32_t s_th_s_ps = INT_MULT_RSHIFT(s_theta2, s_psi2, INT32_TRIG_FRAC);
  int32_t s_th_c_ps = INT_MULT_RSHIFT(s_theta2, c_psi2, INT32_TRIG_FRAC);

  q->qi = INT_MULT_RSHIFT(c_phi2, c_th_c_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC) +
          INT_MULT_RSHIFT(s_phi2, s_th_s_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC);
  q->qx = INT_MULT_RSHIFT(-c_phi2, s_th_s_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC) +
          INT_MULT_RSHIFT(s_phi2, c_th_c_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC);
  q->qy = INT_MULT_RSHIFT(c_phi2, s_th_c_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC) +
          INT_MULT_RSHIFT(s_phi2, c_th_s_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC);
  q->qz = INT_MULT_RSHIFT(c_phi2, c_th_s_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC) +
          INT_MULT_RSHIFT(-s_phi2, s_th_c_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC);
}

void int32_quat_of_axis_angle(struct Int32Quat* q, struct Int32Vect3* uv, int32_t angle)
{
  int32_t san2;
  PPRZ_ITRIG_SIN(san2, (angle / 2));
  int32_t can2;
  PPRZ_ITRIG_COS(can2, (angle / 2));
  q->qi = can2;
  q->qx = san2 * uv->x;
  q->qy = san2 * uv->y;
  q->qz = san2 * uv->z;
}

void int32_quat_of_rmat(struct Int32Quat* q, struct Int32RMat* r)
{
  const int32_t tr = RMAT_TRACE(*r);
  if (tr > 0) {
    const int32_t two_qi_two = TRIG_BFP_OF_REAL(1.) + tr;
    uint32_t two_qi = int32_sqrt(two_qi_two << INT32_TRIG_FRAC);
    two_qi = two_qi << (INT32_QUAT_FRAC - INT32_TRIG_FRAC);
    q->qi = two_qi / 2;
    q->qx = ((RMAT_ELMT(*r, 1, 2) - RMAT_ELMT(*r, 2, 1)) <<
             (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1))
            / two_qi;
    q->qy = ((RMAT_ELMT(*r, 2, 0) - RMAT_ELMT(*r, 0, 2)) <<
             (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1))
            / two_qi;
    q->qz = ((RMAT_ELMT(*r, 0, 1) - RMAT_ELMT(*r, 1, 0)) <<
             (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1))
            / two_qi;
  } else {
    if (RMAT_ELMT(*r, 0, 0) > RMAT_ELMT(*r, 1, 1) &&
        RMAT_ELMT(*r, 0, 0) > RMAT_ELMT(*r, 2, 2)) {
      const int32_t two_qx_two = RMAT_ELMT(*r, 0, 0) - RMAT_ELMT(*r, 1, 1)
                                 - RMAT_ELMT(*r, 2, 2) + TRIG_BFP_OF_REAL(1.);
      uint32_t two_qx = int32_sqrt(two_qx_two << INT32_TRIG_FRAC);
      two_qx = two_qx << (INT32_QUAT_FRAC - INT32_TRIG_FRAC);
      q->qi = ((RMAT_ELMT(*r, 1, 2) - RMAT_ELMT(*r, 2, 1)) <<
               (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1))
              / two_qx;
      q->qx = two_qx / 2;
      q->qy = ((RMAT_ELMT(*r, 0, 1) + RMAT_ELMT(*r, 1, 0)) <<
               (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1))
              / two_qx;
      q->qz = ((RMAT_ELMT(*r, 2, 0) + RMAT_ELMT(*r, 0, 2)) <<
               (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1))
              / two_qx;
    } else if (RMAT_ELMT(*r, 1, 1) > RMAT_ELMT(*r, 2, 2)) {
      const int32_t two_qy_two = RMAT_ELMT(*r, 1, 1) - RMAT_ELMT(*r, 0, 0)
                                 - RMAT_ELMT(*r, 2, 2) + TRIG_BFP_OF_REAL(1.);
      uint32_t two_qy = int32_sqrt(two_qy_two << INT32_TRIG_FRAC);
      two_qy = two_qy << (INT32_QUAT_FRAC - INT32_TRIG_FRAC);
      q->qi = ((RMAT_ELMT(*r, 2, 0) - RMAT_ELMT(*r, 0, 2)) <<
               (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1))
              / two_qy;
      q->qx = ((RMAT_ELMT(*r, 0, 1) + RMAT_ELMT(*r, 1, 0)) <<
               (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1))
              / two_qy;
      q->qy = two_qy / 2;
      q->qz = ((RMAT_ELMT(*r, 1, 2) + RMAT_ELMT(*r, 2, 1)) <<
               (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1))
              / two_qy;
    } else {
      const int32_t two_qz_two = RMAT_ELMT(*r, 2, 2) - RMAT_ELMT(*r, 0, 0)
                                 - RMAT_ELMT(*r, 1, 1) + TRIG_BFP_OF_REAL(1.);
      uint32_t two_qz = int32_sqrt(two_qz_two << INT32_TRIG_FRAC);
      two_qz = two_qz << (INT32_QUAT_FRAC - INT32_TRIG_FRAC);
      q->qi = ((RMAT_ELMT(*r, 0, 1) - RMAT_ELMT(*r, 1, 0)) <<
               (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1))
              / two_qz;
      q->qx = ((RMAT_ELMT(*r, 2, 0) + RMAT_ELMT(*r, 0, 2)) <<
               (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1))
              / two_qz;
      q->qy = ((RMAT_ELMT(*r, 1, 2) + RMAT_ELMT(*r, 2, 1)) <<
               (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1))
              / two_qz;
      q->qz = two_qz / 2;
    }
  }
}


/*
 *
 * Euler angles
 *
 */

void int32_eulers_of_rmat(struct Int32Eulers* e, struct Int32RMat* rm)
{
  const float dcm00 = TRIG_FLOAT_OF_BFP(rm->m[0]);
  const float dcm01 = TRIG_FLOAT_OF_BFP(rm->m[1]);
  const float dcm02 = TRIG_FLOAT_OF_BFP(rm->m[2]);
  const float dcm12 = TRIG_FLOAT_OF_BFP(rm->m[5]);
  const float dcm22 = TRIG_FLOAT_OF_BFP(rm->m[8]);
  const float phi   = atan2f(dcm12, dcm22);
  const float theta = -asinf(dcm02);
  const float psi   = atan2f(dcm01, dcm00);
  e->phi   = ANGLE_BFP_OF_REAL(phi);
  e->theta = ANGLE_BFP_OF_REAL(theta);
  e->psi   = ANGLE_BFP_OF_REAL(psi);
}

void int32_eulers_of_quat(struct Int32Eulers* e, struct Int32Quat* q)
{
  const int32_t qx2  = INT_MULT_RSHIFT(q->qx, q->qx, INT32_QUAT_FRAC);
  const int32_t qy2  = INT_MULT_RSHIFT(q->qy, q->qy, INT32_QUAT_FRAC);
  const int32_t qz2  = INT_MULT_RSHIFT(q->qz, q->qz, INT32_QUAT_FRAC);
  const int32_t qiqx = INT_MULT_RSHIFT(q->qi, q->qx, INT32_QUAT_FRAC);
  const int32_t qiqy = INT_MULT_RSHIFT(q->qi, q->qy, INT32_QUAT_FRAC);
  const int32_t qiqz = INT_MULT_RSHIFT(q->qi, q->qz, INT32_QUAT_FRAC);
  const int32_t qxqy = INT_MULT_RSHIFT(q->qx, q->qy, INT32_QUAT_FRAC);
  const int32_t qxqz = INT_MULT_RSHIFT(q->qx, q->qz, INT32_QUAT_FRAC);
  const int32_t qyqz = INT_MULT_RSHIFT(q->qy, q->qz, INT32_QUAT_FRAC);
  const int32_t one = TRIG_BFP_OF_REAL(1);
  const int32_t two = TRIG_BFP_OF_REAL(2);

  /* dcm00 = 1.0 - 2.*(  qy2 +  qz2 ); */
  const int32_t idcm00 =  one - INT_MULT_RSHIFT(two, (qy2 + qz2),
                          INT32_TRIG_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC);
  /* dcm01 =       2.*( qxqy + qiqz ); */
  const int32_t idcm01 = INT_MULT_RSHIFT(two, (qxqy + qiqz),
                                         INT32_TRIG_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC);
  /* dcm02 =       2.*( qxqz - qiqy ); */
  const int32_t idcm02 = INT_MULT_RSHIFT(two, (qxqz - qiqy),
                                         INT32_TRIG_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC);
  /* dcm12 =       2.*( qyqz + qiqx ); */
  const int32_t idcm12 = INT_MULT_RSHIFT(two, (qyqz + qiqx),
                                         INT32_TRIG_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC);
  /* dcm22 = 1.0 - 2.*(  qx2 +  qy2 ); */
  const int32_t idcm22 = one - INT_MULT_RSHIFT(two, (qx2 + qy2),
                         INT32_TRIG_FRAC + INT32_QUAT_FRAC - INT32_TRIG_FRAC);
  const float dcm00 = (float)idcm00 / (1 << INT32_TRIG_FRAC);
  const float dcm01 = (float)idcm01 / (1 << INT32_TRIG_FRAC);
  const float dcm02 = (float)idcm02 / (1 << INT32_TRIG_FRAC);
  const float dcm12 = (float)idcm12 / (1 << INT32_TRIG_FRAC);
  const float dcm22 = (float)idcm22 / (1 << INT32_TRIG_FRAC);

  const float phi   = atan2f(dcm12, dcm22);
  const float theta = -asinf(dcm02);
  const float psi   = atan2f(dcm01, dcm00);
  e->phi   = ANGLE_BFP_OF_REAL(phi);
  e->theta = ANGLE_BFP_OF_REAL(theta);
  e->psi   = ANGLE_BFP_OF_REAL(psi);
}


/*
 *
 * Rotational speeds
 *
 */

void int32_rates_of_eulers_dot_321(struct Int32Rates* r, struct Int32Eulers* e, struct Int32Eulers* ed)
{
  int32_t sphi;
  PPRZ_ITRIG_SIN(sphi, e->phi);
  int32_t cphi;
  PPRZ_ITRIG_COS(cphi, e->phi);
  int32_t stheta;
  PPRZ_ITRIG_SIN(stheta, e->theta);
  int32_t ctheta;
  PPRZ_ITRIG_COS(ctheta, e->theta);

  int32_t cphi_ctheta = INT_MULT_RSHIFT(cphi,   ctheta, INT32_TRIG_FRAC);
  int32_t sphi_ctheta = INT_MULT_RSHIFT(sphi,   ctheta, INT32_TRIG_FRAC);

  r->p = - INT_MULT_RSHIFT(stheta, ed->psi, INT32_TRIG_FRAC) + ed->phi;
  r->q = INT_MULT_RSHIFT(sphi_ctheta, ed->psi, INT32_TRIG_FRAC) + INT_MULT_RSHIFT(cphi, ed->theta, INT32_TRIG_FRAC);
  r->r = INT_MULT_RSHIFT(cphi_ctheta, ed->psi, INT32_TRIG_FRAC) - INT_MULT_RSHIFT(sphi, ed->theta, INT32_TRIG_FRAC);
}

void int32_eulers_dot_321_of_rates(struct Int32Eulers* ed, struct Int32Eulers* e, struct Int32Rates* r)
{
  int32_t sphi;
  PPRZ_ITRIG_SIN(sphi, e->phi);
  int32_t cphi;
  PPRZ_ITRIG_COS(cphi, e->phi);
  int32_t stheta;
  PPRZ_ITRIG_SIN(stheta, e->theta);
  int64_t ctheta;
  PPRZ_ITRIG_COS(ctheta, e->theta);

  if (ctheta != 0) {
    int64_t cphi_stheta = INT_MULT_RSHIFT(cphi, stheta, INT32_TRIG_FRAC);
    int64_t sphi_stheta = INT_MULT_RSHIFT(sphi, stheta, INT32_TRIG_FRAC);

    ed->phi = r->p + (int32_t)((sphi_stheta * (int64_t)r->q) / ctheta) + (int32_t)((cphi_stheta * (int64_t)r->r) / ctheta);
    ed->theta = INT_MULT_RSHIFT(cphi, r->q, INT32_TRIG_FRAC) - INT_MULT_RSHIFT(sphi, r->r, INT32_TRIG_FRAC);
    ed->psi = (int32_t)(((int64_t)sphi * (int64_t)r->q) / ctheta) + (int32_t)(((int64_t)cphi * (int64_t)r->r) / ctheta);
  }
  /* FIXME: What do you wanna do when you hit the singularity ? */
  /* probably not return an uninitialized variable, or ?        */
  else {
    INT_EULERS_ZERO(*ed);
  }
}
