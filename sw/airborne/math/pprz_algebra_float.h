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
 * @file pprz_algebra_float.h
 * @brief Paparazzi floating point algebra.
 *
 * @addtogroup math_algebra
 * @{
 * @defgroup math_algebra_float Float Algebra
 * @{
 */

#ifndef PPRZ_ALGEBRA_FLOAT_H
#define PPRZ_ALGEBRA_FLOAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pprz_algebra.h"
#include "message_pragmas.h"

#include <math.h>
#include <float.h> // for FLT_MIN

/* this seems to be missing for some arch */
#ifndef M_SQRT2
#define M_SQRT2         1.41421356237309504880
#endif

struct FloatVect2 {
  float x;
  float y;
};

struct FloatVect3 {
  float x;
  float y;
  float z;
};

/**
 * @brief Roation quaternion
 */
struct FloatQuat {
  float qi;
  float qx;
  float qy;
  float qz;
};

struct FloatMat33 {
  float m[3 * 3];
};

/**
 * @brief rotation matrix
 */
struct FloatRMat {
  float m[3 * 3];
};

/**
 * @brief euler angles
 * @details Units: radians */
struct FloatEulers {
  float phi; ///< in radians
  float theta; ///< in radians
  float psi; ///< in radians
};

/**
 * @brief angular rates
 * @details Units: rad/s */
struct FloatRates {
  float p; ///< in rad/s
  float q; ///< in rad/s
  float r; ///< in rad/s
};

#define FLOAT_ANGLE_NORMALIZE(_a) {             \
    while (_a >  M_PI) _a -= (2.*M_PI);             \
    while (_a < -M_PI) _a += (2.*M_PI);             \
  }

/*
 * Returns the real part of the log of v in base of n
 */
static inline float float_log_n(float v, float n)
{
  if (fabsf(v) < 1e-4) { // avoid inf
    return - 1.0E+30;
  }
  if (fabsf(n) < 1e-4) { // avoid nan
    return 0;
  }
  return logf(fabsf(v)) / logf(n);
}

//
//
// Vector algebra
//
//


/*
 * Dimension 2 Vectors
 */

#define FLOAT_VECT2_ZERO(_v) VECT2_ASSIGN(_v, 0., 0.)

/* macros also usable if _v is not a FloatVect2, but a different struct with x,y members */
#define FLOAT_VECT2_NORM(_v) sqrtf(VECT2_NORM2(_v))

static inline float float_vect2_norm2(struct FloatVect2 *v)
{
  return v->x * v->x + v->y * v->y;
}

static inline float float_vect2_norm(struct FloatVect2 *v)
{
  return sqrtf(float_vect2_norm2(v));
}

/** normalize 2D vector in place */
static inline void float_vect2_normalize(struct FloatVect2 *v)
{
  const float n = float_vect2_norm(v);
  if (n > 1e-4f) {
    v->x /= n;
    v->y /= n;
  }
}

#define FLOAT_VECT2_NORMALIZE(_v) float_vect2_normalize(&(_v))


/*
 * Dimension 3 Vectors
 */

#define FLOAT_VECT3_ZERO(_v) VECT3_ASSIGN(_v, 0., 0., 0.)

/* macros also usable if _v is not a FloatVect3, but a different struct with x,y,z members */
#define FLOAT_VECT3_NORM(_v) sqrtf(VECT3_NORM2(_v))

static inline float float_vect3_norm2(struct FloatVect3 *v)
{
  return v->x * v->x + v->y * v->y + v->z * v->z;
}

static inline float float_vect3_norm(struct FloatVect3 *v)
{
  return sqrtf(float_vect3_norm2(v));
}

/** normalize 3D vector in place */
static inline void float_vect3_normalize(struct FloatVect3 *v)
{
  const float n = float_vect3_norm(v);
  if (n > 1e-4f) {
    v->x /= n;
    v->y /= n;
    v->z /= n;
  }
}

#define FLOAT_VECT3_NORMALIZE(_v) float_vect3_normalize(&(_v))



#define FLOAT_RATES_ZERO(_r) {          \
    RATES_ASSIGN(_r, 0., 0., 0.);       \
  }

#define FLOAT_RATES_NORM(_v) (sqrtf((_v).p*(_v).p + (_v).q*(_v).q + (_v).r*(_v).r))

#define FLOAT_RATES_LIN_CMB(_ro, _r1, _s1, _r2, _s2) {          \
    _ro.p = _s1 * _r1.p + _s2 * _r2.p;                  \
    _ro.q = _s1 * _r1.q + _s2 * _r2.q;                  \
    _ro.r = _s1 * _r1.r + _s2 * _r2.r;                  \
  }


extern void float_vect3_integrate_fi(struct FloatVect3 *vec, struct FloatVect3 *dv,
                                     float dt);

extern void float_rates_integrate_fi(struct FloatRates *r, struct FloatRates *dr,
                                     float dt);

extern void float_rates_of_euler_dot(struct FloatRates *r, struct FloatEulers *e,
                                     struct FloatEulers *edot);

/* defines for backwards compatibility */
#define FLOAT_VECT3_INTEGRATE_FI(_vo, _dv, _dt) WARNING("FLOAT_VECT3_INTEGRATE_FI macro is deprecated, use the lower case function instead") float_vect3_integrate_fi(&(_vo), &(_dv), _dt)
#define FLOAT_RATES_INTEGRATE_FI(_ra, _racc, _dt) WARNING("FLOAT_RATES_INTEGRATE_FI macro is deprecated, use the lower case function instead") float_rates_integrate_fi(&(_ra), &(_racc), _dt)
#define FLOAT_RATES_OF_EULER_DOT(_ra, _e, _ed)  WARNING("FLOAT_RATES_OF_EULER_DOT macro is deprecated, use the lower case function instead") float_rates_of_euler_dot(&(_ra), &(_e), &(_ed))


/*
 * 3x3 matrices
 */
#define FLOAT_MAT33_ZERO(_m) {                      \
    MAT33_ELMT(_m, 0, 0) = 0.;                      \
    MAT33_ELMT(_m, 0, 1) = 0.;                      \
    MAT33_ELMT(_m, 0, 2) = 0.;                      \
    MAT33_ELMT(_m, 1, 0) = 0.;                      \
    MAT33_ELMT(_m, 1, 1) = 0.;                      \
    MAT33_ELMT(_m, 1, 2) = 0.;                      \
    MAT33_ELMT(_m, 2, 0) = 0.;                      \
    MAT33_ELMT(_m, 2, 1) = 0.;                      \
    MAT33_ELMT(_m, 2, 2) = 0.;                      \
  }

#define FLOAT_MAT33_DIAG(_m, _d00, _d11, _d22) {    \
    MAT33_ELMT(_m, 0, 0) = _d00;                    \
    MAT33_ELMT(_m, 0, 1) = 0.;                      \
    MAT33_ELMT(_m, 0, 2) = 0.;                      \
    MAT33_ELMT(_m, 1, 0) = 0.;                      \
    MAT33_ELMT(_m, 1, 1) = _d11;                    \
    MAT33_ELMT(_m, 1, 2) = 0.;                      \
    MAT33_ELMT(_m, 2, 0) = 0.;                      \
    MAT33_ELMT(_m, 2, 1) = 0.;                      \
    MAT33_ELMT(_m, 2, 2) = _d22;                    \
  }


//
//
// Rotation Matrices
//
//


/** initialises a rotation matrix to identity */
static inline void float_rmat_identity(struct FloatRMat *rm)
{
  FLOAT_MAT33_DIAG(*rm, 1., 1., 1.);
}

/** Inverse/transpose of a rotation matrix.
 * m_b2a = inv(_m_a2b) = transp(_m_a2b)
 */
extern void float_rmat_inv(struct FloatRMat *m_b2a, struct FloatRMat *m_a2b);

/** Composition (multiplication) of two rotation matrices.
 * m_a2c = m_a2b comp m_b2c , aka  m_a2c = m_b2c * m_a2b
 */
extern void float_rmat_comp(struct FloatRMat *m_a2c, struct FloatRMat *m_a2b,
                            struct FloatRMat *m_b2c);

/** Composition (multiplication) of two rotation matrices.
 * m_a2b = m_a2c comp_inv m_b2c , aka  m_a2b = inv(_m_b2c) * m_a2c
 */
extern void float_rmat_comp_inv(struct FloatRMat *m_a2b, struct FloatRMat *m_a2c,
                                struct FloatRMat *m_b2c);

/// Norm of a rotation matrix.
extern float float_rmat_norm(struct FloatRMat *rm);

/** rotate 3D vector by rotation matrix.
 * vb = m_a2b * va
 */
extern void float_rmat_vmult(struct FloatVect3 *vb, struct FloatRMat *m_a2b,
                             struct FloatVect3 *va);

/** rotate 3D vector by transposed rotation matrix.
 * vb = m_b2a^T * va
 */
extern void float_rmat_transp_vmult(struct FloatVect3 *vb, struct FloatRMat *m_b2a,
                                    struct FloatVect3 *va);

/** rotate angle by rotation matrix.
 * rb = m_a2b * ra
 */
extern void float_rmat_mult(struct FloatEulers *rb, struct FloatRMat *m_a2b,
                            struct FloatEulers *ra);

/** rotate angle by transposed rotation matrix.
 * rb = m_b2a^T * ra
 */
extern void float_rmat_transp_mult(struct FloatEulers *rb, struct FloatRMat *m_b2a,
                                   struct FloatEulers *ra);

/** rotate anglular rates by rotation matrix.
 * rb = m_a2b * ra
 */
extern void float_rmat_ratemult(struct FloatRates *rb, struct FloatRMat *m_a2b,
                                struct FloatRates *ra);

/** rotate anglular rates by transposed rotation matrix.
 * rb = m_b2a^T * ra
 */
extern void float_rmat_transp_ratemult(struct FloatRates *rb, struct FloatRMat *m_b2a,
                                       struct FloatRates *ra);

/** initialises a rotation matrix from unit vector axis and angle */
extern void float_rmat_of_axis_angle(struct FloatRMat *rm, struct FloatVect3 *uv, float angle);

/** Rotation matrix from 321 Euler angles (float).
 * The Euler angles are interpreted as zy'x'' (intrinsic) rotation.
 * First rotate around z with psi, then around the new y' with theta,
 * then around new x'' with phi.
 * This is the same as a xyz (extrinsic) rotation,
 * rotating around the fixed x, then y then z axis.
 * - psi range: -pi < psi <= pi
 * - theta range: -pi/2 <= theta <= pi/2
 * - phi range: -pi < phi <= pi
 * @param[out] rm pointer to rotation matrix
 * @param[in]  e pointer to Euler angles
 */
extern void float_rmat_of_eulers_321(struct FloatRMat *rm, struct FloatEulers *e);
extern void float_rmat_of_eulers_312(struct FloatRMat *rm, struct FloatEulers *e);
#define float_rmat_of_eulers float_rmat_of_eulers_321

extern void float_rmat_of_quat(struct FloatRMat *rm, struct FloatQuat *q);
/** in place first order integration of a rotation matrix */
extern void float_rmat_integrate_fi(struct FloatRMat *rm, struct FloatRates *omega, float dt);
extern float float_rmat_reorthogonalize(struct FloatRMat *rm);

/* defines for backwards compatibility */
#define FLOAT_RMAT_INV(_m_b2a, _m_a2b) WARNING("FLOAT_RMAT_INV macro is deprecated, use the lower case function instead") float_rmat_inv(&(_m_b2a), &(_m_a2b))
#define FLOAT_RMAT_NORM(_m) WARNING("FLOAT_RMAT_NORM macro is deprecated, use the lower case function instead") float_rmat_norm(&(_m))
#define FLOAT_RMAT_COMP(_m_a2c, _m_a2b, _m_b2c) WARNING("FLOAT_RMAT_COMP macro is deprecated, use the lower case function instead") float_rmat_comp(&(_m_a2c), &(_m_a2b), &(_m_b2c))
#define FLOAT_RMAT_COMP_INV(_m_a2b, _m_a2c, _m_b2c) WARNING("FLOAT_RMAT_COMP_INV macro is deprecated, use the lower case function instead") float_rmat_comp_inv(&(_m_a2b), &(_m_a2c), &(_m_b2c))
#define FLOAT_RMAT_VMULT(_vb, _m_a2b, _va) WARNING("FLOAT_RMAT_VMULT macro is deprecated, use the lower case function instead") float_rmat_vmult(&(_vb), &(_m_a2b), &(_va))
#define FLOAT_RMAT_TRANSP_VMULT(_vb, _m_b2a, _va) WARNING("FLOAT_RMAT_TRANSP_VMULT macro is deprecated, use the lower case function instead") float_rmat_transp_vmult(&(_vb), &(_m_b2a), &(_va))
#define FLOAT_RMAT_RATEMULT(_rb, _m_a2b, _ra) WARNING("FLOAT_RMAT_RATEMULT macro is deprecated, use the lower case function instead") float_rmat_ratemult(&(_rb), &(_m_a2b), &(_ra))
#define FLOAT_RMAT_TRANSP_RATEMULT(_rb, _m_b2a, _ra) WARNING("FLOAT_RMAT_TRANSP_RATEMULT macro is deprecated, use the lower case function instead") float_rmat_ratemult(&(_rb), &(_m_b2a), &(_ra))
#define FLOAT_RMAT_OF_AXIS_ANGLE(_rm, _uv, _an) WARNING("FLOAT_RMAT_OF_AXIS_ANGLE macro is deprecated, use the lower case function instead") float_rmat_of_axis_angle(&(_rm), &(_uv), _an)
#define FLOAT_RMAT_OF_EULERS(_rm, _e)     WARNING("FLOAT_RMAT_OF_EULERS macro is deprecated, use the lower case function instead") float_rmat_of_eulers_321(&(_rm), &(_e))
#define FLOAT_RMAT_OF_EULERS_321(_rm, _e) WARNING("FLOAT_RMAT_OF_EULERS_321 macro is deprecated, use the lower case function instead") float_rmat_of_eulers_321(&(_rm), &(_e))
#define FLOAT_RMAT_OF_EULERS_312(_rm, _e) WARNING("FLOAT_RMAT_OF_EULERS_312 macro is deprecated, use the lower case function instead") float_rmat_of_eulers_312(&(_rm), &(_e))
#define FLOAT_RMAT_OF_QUAT(_rm, _q)       WARNING("FLOAT_RMAT_OF_QUAT macro is deprecated, use the lower case function instead") float_rmat_of_quat(&(_rm), &(_q))
#define FLOAT_RMAT_INTEGRATE_FI(_rm, _omega, _dt) WARNING("FLOAT_RMAT_INTEGRATE_FI macro is deprecated, use the lower case function instead") float_rmat_integrate_fi(&(_rm), &(_omega), &(_dt))



//
//
// Quaternion algebras
//
//

/** initialises a quaternion to identity */
static inline void float_quat_identity(struct FloatQuat *q)
{
  q->qi = 1.0;
  q->qx = 0;
  q->qy = 0;
  q->qz = 0;
}

#define FLOAT_QUAT_NORM2(_q) (SQUARE((_q).qi) + SQUARE((_q).qx) + SQUARE((_q).qy) + SQUARE((_q).qz))

static inline float float_quat_norm(struct FloatQuat *q)
{
  return sqrtf(SQUARE(q->qi) + SQUARE(q->qx) +  SQUARE(q->qy) + SQUARE(q->qz));
}

static inline void float_quat_normalize(struct FloatQuat *q)
{
  float qnorm = float_quat_norm(q);
  if (qnorm > FLT_MIN) {
    q->qi = q->qi / qnorm;
    q->qx = q->qx / qnorm;
    q->qy = q->qy / qnorm;
    q->qz = q->qz / qnorm;
  }
}

static inline void float_quat_invert(struct FloatQuat *qo, struct FloatQuat *qi)
{
  QUAT_INVERT(*qo, *qi);
}

static inline void float_quat_wrap_shortest(struct FloatQuat *q)
{
  if (q->qi < 0.) {
    QUAT_EXPLEMENTARY(*q, *q);
  }
}

#define FLOAT_QUAT_EXTRACT(_vo, _qi) QUAT_EXTRACT_Q(_vo, _qi)


/** Composition (multiplication) of two quaternions.
 * a2c = a2b comp b2c , aka  a2c = a2b * b2c
 */
extern void float_quat_comp(struct FloatQuat *a2c, struct FloatQuat *a2b, struct FloatQuat *b2c);

/** Composition (multiplication) of two quaternions.
 * a2b = a2c comp_inv b2c , aka  a2b = a2c * inv(b2c)
 */
extern void float_quat_comp_inv(struct FloatQuat *a2b, struct FloatQuat *a2c, struct FloatQuat *b2c);

/** Composition (multiplication) of two quaternions.
 * b2c = a2b inv_comp a2c , aka  b2c = inv(_a2b) * a2c
 */
extern void float_quat_inv_comp(struct FloatQuat *b2c, struct FloatQuat *a2b, struct FloatQuat *a2c);

/** Composition (multiplication) of two quaternions with normalization.
 * a2c = a2b comp b2c , aka  a2c = a2b * b2c
 */
extern void float_quat_comp_norm_shortest(struct FloatQuat *a2c, struct FloatQuat *a2b, struct FloatQuat *b2c);

/** Composition (multiplication) of two quaternions with normalization.
 * a2b = a2c comp_inv b2c , aka  a2b = a2c * inv(b2c)
 */
extern void float_quat_comp_inv_norm_shortest(struct FloatQuat *a2b, struct FloatQuat *a2c, struct FloatQuat *b2c);

/** Composition (multiplication) of two quaternions with normalization.
 * b2c = a2b inv_comp a2c , aka  b2c = inv(_a2b) * a2c
 */
extern void float_quat_inv_comp_norm_shortest(struct FloatQuat *b2c, struct FloatQuat *a2b, struct FloatQuat *a2c);

/** Quaternion derivative from rotational velocity.
 * qd = -0.5*omega(r) * q
 * or equally:
 * qd = 0.5 * q * omega(r)
 */
extern void float_quat_derivative(struct FloatQuat *qd, struct FloatRates *r, struct FloatQuat *q);

/** Quaternion derivative from rotational velocity with Lagrange multiplier.
 * qd = -0.5*omega(r) * q
 * or equally:
 * qd = 0.5 * q * omega(r)
 */
extern void float_quat_derivative_lagrange(struct FloatQuat *qd, struct FloatRates *r, struct FloatQuat *q);

/** Delta rotation quaternion with constant angular rates.
 */
extern void float_quat_differential(struct FloatQuat *q_out, struct FloatRates *w, float dt);

/** in place first order quaternion integration with constant rotational velocity */
extern void float_quat_integrate_fi(struct FloatQuat *q, struct FloatRates *omega, float dt);

/** in place quaternion integration with constant rotational velocity */
extern void float_quat_integrate(struct FloatQuat *q, struct FloatRates *omega, float dt);

/** rotate 3D vector by quaternion.
 * vb = q_a2b * va * q_a2b^-1
 */
extern void float_quat_vmult(struct FloatVect3 *v_out, struct FloatQuat *q, const struct FloatVect3 *v_in);

/// Quaternion from Euler angles.
extern void float_quat_of_eulers(struct FloatQuat *q, struct FloatEulers *e);
extern void float_quat_of_eulers_zxy(struct FloatQuat *q, struct FloatEulers *e);
extern void float_quat_of_eulers_yxz(struct FloatQuat *q, struct FloatEulers *e);

/** Quaternion from unit vector and angle.
 * Output quaternion is not normalized.
 * It will be a unit quaternion only if the input vector is also unitary.
 */
extern void float_quat_of_axis_angle(struct FloatQuat *q, const struct FloatVect3 *uv, float angle);

/** Quaternion from orientation vector.
 * Length/norm of the vector is the angle.
 */
extern void float_quat_of_orientation_vect(struct FloatQuat *q, const struct FloatVect3 *ov);

/// Quaternion from rotation matrix.
extern void float_quat_of_rmat(struct FloatQuat *q, struct FloatRMat *rm);

/// Tilt twist decomposition of quaternion
extern void float_quat_tilt_twist(struct FloatQuat *tilt, struct FloatQuat *twist, struct FloatQuat *quat);


/* defines for backwards compatibility */
#define FLOAT_QUAT_ZERO(_q) WARNING("FLOAT_QUAT_ZERO macro is deprecated, use the lower case function instead") float_quat_identity(&(_q))
#define FLOAT_QUAT_INVERT(_qo, _qi) WARNING("FLOAT_QUAT_INVERT macro is deprecated, use the lower case function instead") float_quat_invert(&(_qo), &(_qi))
#define FLOAT_QUAT_WRAP_SHORTEST(_q) WARNING("FLOAT_QUAT_WRAP_SHORTEST macro is deprecated, use the lower case function instead") float_quat_wrap_shortest(&(_q))
#define FLOAT_QUAT_NORM(_q) WARNING("FLOAT_QUAT_NORM macro is deprecated, use the lower case function instead") float_quat_norm(&(_q))
#define FLOAT_QUAT_NORMALIZE(_q) WARNING("FLOAT_QUAT_NORMALIZE macro is deprecated, use the lower case function instead") float_quat_normalize(&(_q))
#define FLOAT_QUAT_COMP(_a2c, _a2b, _b2c) WARNING("FLOAT_QUAT_COMP macro is deprecated, use the lower case function instead") float_quat_comp(&(_a2c), &(_a2b), &(_b2c))
#define FLOAT_QUAT_MULT(_a2c, _a2b, _b2c) WARNING("FLOAT_QUAT_MULT macro is deprecated, use the lower case function instead") float_quat_comp(&(_a2c), &(_a2b), &(_b2c))
#define FLOAT_QUAT_INV_COMP(_b2c, _a2b, _a2c) WARNING("FLOAT_QUAT_INV_COMP macro is deprecated, use the lower case function instead") float_quat_inv_comp(&(_b2c), &(_a2b), &(_a2c))
#define FLOAT_QUAT_COMP_INV(_a2b, _a2c, _b2c) WARNING("FLOAT_QUAT_COMP_INV macro is deprecated, use the lower case function instead") float_quat_comp_inv(&(_a2b), &(_a2c), &(_b2c))
#define FLOAT_QUAT_COMP_NORM_SHORTEST(_a2c, _a2b, _b2c) WARNING("FLOAT_QUAT_COMP_NORM_SHORTEST macro is deprecated, use the lower case function instead") float_quat_comp_norm_shortest(&(_a2c), &(_a2b), &(_b2c))
#define FLOAT_QUAT_COMP_INV_NORM_SHORTEST(_a2b, _a2c, _b2c) WARNING("FLOAT_QUAT_COMP_INV_NORM_SHORTEST macro is deprecated, use the lower case function instead") float_quat_comp_inv_norm_shortest(&(_a2b), &(_a2c), &(_b2c))
#define FLOAT_QUAT_INV_COMP_NORM_SHORTEST(_b2c, _a2b, _a2c) WARNING("FLOAT_QUAT_INV_COMP_NORM_SHORTEST macro is deprecated, use the lower case function instead") float_quat_inv_comp_norm_shortest(&(_b2c), &(_a2b), &(_a2c))
#define FLOAT_QUAT_DIFFERENTIAL(q_out, w, dt) WARNING("FLOAT_QUAT_DIFFERENTIAL macro is deprecated, use the lower case function instead") float_quat_differential(&(q_out), &(w), dt)
#define FLOAT_QUAT_INTEGRATE(_q, _omega, _dt) WARNING("FLOAT_QUAT_INTEGRATE macro is deprecated, use the lower case function instead") float_quat_integrate(&(_q), &(_omega), _dt)
#define FLOAT_QUAT_VMULT(v_out, q, v_in) WARNING("FLOAT_QUAT_VMULT macro is deprecated, use the lower case function instead") float_quat_vmult(&(v_out), &(q), &(v_in))
#define FLOAT_QUAT_DERIVATIVE(_qd, _r, _q) WARNING("FLOAT_QUAT_DERIVATIVE macro is deprecated, use the lower case function instead") float_quat_derivative(&(_qd), &(_r), &(_q))
#define FLOAT_QUAT_DERIVATIVE_LAGRANGE(_qd, _r, _q) WARNING("FLOAT_QUAT_DERIVATIVE_LAGRANGE macro is deprecated, use the lower case function instead") float_quat_derivative_lagrange(&(_qd), &(_r), &(_q))
#define FLOAT_QUAT_OF_EULERS(_q, _e) WARNING("FLOAT_QUAT_OF_EULERS macro is deprecated, use the lower case function instead") float_quat_of_eulers(&(_q), &(_e))
#define FLOAT_QUAT_OF_AXIS_ANGLE(_q, _uv, _an) WARNING("FLOAT_QUAT_OF_AXIS_ANGLE macro is deprecated, use the lower case function instead") float_quat_of_axis_angle(&(_q), &(_uv), _an)
#define FLOAT_QUAT_OF_ORIENTATION_VECT(_q, _ov) WARNING("FLOAT_QUAT_OF_ORIENTATION_VECT macro is deprecated, use the lower case function instead") float_quat_of_orientation_vect(&(_q), &(_ov))
#define FLOAT_QUAT_OF_RMAT(_q, _r) WARNING("FLOAT_QUAT_OF_RMAT macro is deprecated, use the lower case function instead") float_quat_of_rmat(&(_q), &(_r))



//
//
// Euler angles
//
//

#define FLOAT_EULERS_ZERO(_e) EULERS_ASSIGN(_e, 0., 0., 0.);

static inline float float_eulers_norm(struct FloatEulers *e)
{
  return sqrtf(SQUARE(e->phi) + SQUARE(e->theta) + SQUARE(e->psi));
}
extern void float_eulers_of_rmat(struct FloatEulers *e, struct FloatRMat *rm);
extern void float_eulers_of_quat(struct FloatEulers *e, struct FloatQuat *q);
extern void float_eulers_of_quat_zxy(struct FloatEulers *e, struct FloatQuat *q);
extern void float_eulers_of_quat_yxz(struct FloatEulers *e, struct FloatQuat *q);

/* defines for backwards compatibility */
#define FLOAT_EULERS_OF_RMAT(_e, _rm) WARNING("FLOAT_EULERS_OF_RMAT macro is deprecated, use the lower case function instead") float_eulers_of_rmat(&(_e), &(_rm))
#define FLOAT_EULERS_OF_QUAT(_e, _q) WARNING("FLOAT_EULERS_OF_QUAT macro is deprecated, use the lower case function instead") float_eulers_of_quat(&(_e), &(_q))
#define FLOAT_EULERS_NORM(_e) WARNING("FLOAT_EULERS_NORM macro is deprecated, use the lower case function instead") float_eulers_norm(&(_e))

//
//
// Generic vector algebra
//
//

/** a = 0 */
static inline void float_vect_zero(float *a, const int n)
{
  int i;
  for (i = 0; i < n; i++) { a[i] = 0.; }
}

/** a = b */
static inline void float_vect_copy(float *a, const float *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { a[i] = b[i]; }
}

/** o = a + b */
static inline void float_vect_sum(float *o, const float *a, const float *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] + b[i]; }
}

/** o = a - b */
static inline void float_vect_diff(float *o, const float *a, const float *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] - b[i]; }
}

/** o = a * b (element wise) */
static inline void float_vect_mul(float *o, const float *a, const float *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] * b[i]; }
}

/** a += b */
static inline void float_vect_add(float *a, const float *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { a[i] += b[i]; }
}

/** a -= b */
static inline void float_vect_sub(float *a, const float *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { a[i] -= b[i]; }
}

/** o = a * s */
static inline void float_vect_smul(float *o, const float *a, const float s, const int n)
{
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] * s; }
}

/** o = a / s */
static inline void float_vect_sdiv(float *o, const float *a, const float s, const int n)
{
  int i;
  if (fabs(s) > 1e-5) {
    for (i = 0; i < n; i++) { o[i] = a[i] / s; }
  }
}

/** ||a|| */
static inline float float_vect_norm(const float *a, const int n)
{
  int i;
  float sum = 0;
  for (i = 0; i < n; i++) { sum += a[i] * a[i]; }
  return sqrtf(sum);
}

/** a *= s */
static inline void float_vect_scale(float *a, const float s, const int n)
{
  int i;
  for (i = 0; i < n; i++) { a[i] *= s; }
}

/** a.b */
static inline float float_vect_dot_product(const float *a, const float *b, const int n)
{
  int i;
  float dot = 0.f;
  for (i = 0; i < n; i++) { dot += a[i] * b[i]; }
  return dot;
}

//
//
// Generic matrix algebra
//
//

/** Make a pointer to a matrix of _rows lines */
#define MAKE_MATRIX_PTR(_ptr, _mat, _rows) \
  float * _ptr[_rows]; \
  { \
    int __i; \
    for (__i = 0; __i < _rows; __i++) { _ptr[__i] = &_mat[__i][0]; } \
  }

extern void float_mat_invert(float **o, float **mat, int n);
extern void float_mat_exp(float **a, float **o, int n);
extern float float_mat_norm_li(float **o, int m, int n);

/** a = 0 */
static inline void float_mat_zero(float **a, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) { a[i][j] = 0.; }
  }
}

/** a = b */
static inline void float_mat_copy(float **a, float **b, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) { a[i][j] = b[i][j]; }
  }
}



/** o = a + b */
static inline void float_mat_sum(float **o, float **a, float **b, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) { o[i][j] = a[i][j] + b[i][j]; }
  }
}

/** o = a - b */
static inline void float_mat_diff(float **o, float **a, float **b, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) { o[i][j] = a[i][j] - b[i][j]; }
  }
}

/** transpose square matrix */
static inline void float_mat_transpose_square(float **a, int n)
{
  int i, j;
  for (i = 0; i < n; i++) {
    for (j = 0; j < i; j++) {
      float t = a[i][j];
      a[i][j] = a[j][i];
      a[j][i] = t;
    }
  }
}


/** transpose non-square matrix */
static inline void float_mat_transpose(float **o, float **a, int n, int m)
{
  int i, j;
  for (i = 0; i < n; i++) {
    for (j = 0; j < m; j++) {
      o[j][i] = a[i][j];
    }
  }
}

/** o = a * b
 *
 * a: [m x n]
 * b: [n x l]
 * o: [m x l]
 */
static inline void float_mat_mul(float **o, float **a, float **b, int m, int n, int l)
{
  int i, j, k;
  for (i = 0; i < m; i++) {
    for (j = 0; j < l; j++) {
      o[i][j] = 0.;
      for (k = 0; k < n; k++) {
        o[i][j] += a[i][k] * b[k][j];
      }
    }
  }
}

/** o = a * b'
 *
 * a: [m x n]
 * b: [l x n]
 * o: [m x l]
 */
static inline void float_mat_mul_transpose(float **o, float **a, float **b, int m, int n, int l)
{
  int i, j, k;
  for (i = 0; i < m; i++) {
    for (j = 0; j < l; j++) {
      o[i][j] = 0.;
      for (k = 0; k < n; k++) {
        o[i][j] += a[i][k] * b[j][k];
      }
    }
  }
}

/** o = a * b
 *
 * a: [m x n]
 * b: [n x l]
 * o: [m x l]
 *
 * Multiply two matrices with eachother.
 * By using a temporary array to store result. The resulting matrix can be stored in one
 * of the input matrices when this function is used, which is useful for consecutive multiplications
 * (e.g. when doing matrix exponentiation), at the cost of some copy overhead.
 */
static inline void float_mat_mul_copy(float **o, float **a, float **b, int m, int n, int l)
{
  float temp[m][l];
  int i, j, k;
  for (i = 0; i < m; i++) {
    for (j = 0; j < l; j++) {
      temp[i][j] = 0.;
      for (k = 0; k < n; k++) {
        temp[i][j] += a[i][k] * b[k][j];
      }
    }
  }
  MAKE_MATRIX_PTR(_o,  o,  m);
  MAKE_MATRIX_PTR(_temp,  temp,  m);
  float_mat_copy(_o, _temp, m, l);
}


/** o = a * b
 *
 * a: [m x n]
 * b: [n x 1]
 * o: [m x 1]
 */
static inline void float_mat_vect_mul(float *o, float **a, float *b, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    o[i] = 0;
    for (j = 0; j < n; j++) {
      o[i] += a[i][j] * b[j];
    }
  }
}

/** a *= k, where k is a scalar value
 *
 * a: [m x n]
 * k: [1 x 1]
 */
static inline void float_mat_scale(float **a, float k, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) {
      a[i][j] *= k;
    }
  }
}

/** a += k*b, where k is a scalar value
 *
 * a: [m x n]
 * b: [m x n]
 * k: [1 x 1]
 */
static inline void float_mat_sum_scaled(float **a, float **b, float k, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) {
      a[i][j] += k * b[i][j];
    }
  }
}


/** matrix minor
 *
 * a: [m x n]
 * o: [I(d,d)     0     ]
 *    [  0    a(d,m:d,n)]
 */
static inline void float_mat_minor(float **o, float **a, int m, int n, int d)
{
  int i, j;
  float_mat_zero(o, m, n);
  for (i = 0; i < d; i++) { o[i][i] = 1.0; }
  for (i = d; i < m; i++) {
    for (j = d; j < n; j++) {
      o[i][j] = a[i][j];
    }
  }
}

/** o = I - v v^T */
static inline void float_mat_vmul(float **o, float *v, int n)
{
  int i, j;
  for (i = 0; i < n; i++) {
    for (j = 0; j < n; j++) {
      o[i][j] = -2. *  v[i] * v[j];
    }
  }
  for (i = 0; i < n; i++) {
    o[i][i] += 1.;
  }
}

/** o = c-th column of matrix a[m x n] */
static inline void float_mat_col(float *o, float **a, int m, int c)
{
  int i;
  for (i = 0; i < m; i++) {
    o[i] = a[i][c];
  }
}

/** Make an n x n identity matrix (for matrix passed as array) */
static inline void float_mat_diagonal_scal(float **o, float v, int n)
{
  int i, j;
  for (i = 0 ; i < n; i++) {
    for (j = 0 ; j < n; j++) {
      if (i == j) {
        o[i][j] = v;
      } else {
        o[i][j] = 0.0;
      }
    }
  }
}

extern bool float_mat_inv_2d(float inv_out[4], float mat_in[4]);
extern void float_mat2_mult(struct FloatVect2 *vect_out, float mat[4], struct FloatVect2 vect_in);
extern bool float_mat_inv_4d(float invOut[16], float mat_in[16]);

extern void float_vect3_bound_in_2d(struct FloatVect3 *vect3, float bound);
extern void float_vect3_bound_in_3d(struct FloatVect3 *vect3, float bound);
extern void float_vect3_scale_in_2d(struct FloatVect3 *vect3, float norm_des);
extern void float_vect2_bound_in_2d(struct FloatVect2 *vect2, float bound);
extern void float_vect2_scale_in_2d(struct FloatVect2 *vect2, float norm_des);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_ALGEBRA_FLOAT_H */
/** @}*/
/** @}*/
