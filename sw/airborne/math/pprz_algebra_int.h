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
 * @file pprz_algebra_int.h
 * @brief Paparazzi fixed point algebra.
 *
 * @addtogroup math_algebra
 * @{
 * @addtogroup math_algebra_int Fixed Point Algebra
 * @{
 */

#ifndef PPRZ_ALGEBRA_INT_H
#define PPRZ_ALGEBRA_INT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include "math/pprz_algebra.h"
#include "math/pprz_trig_int.h"
#include <stdlib.h>


struct Uint8Vect3 {
  uint8_t x;
  uint8_t y;
  uint8_t z;
};

struct Int8Vect3 {
  int8_t x;
  int8_t y;
  int8_t z;
};

struct Uint16Vect3 {
  uint16_t x;
  uint16_t y;
  uint16_t z;
};

struct Int16Vect3 {
  int16_t x;
  int16_t y;
  int16_t z;
};

#define INT32_POS_FRAC 8
#define INT32_POS_OF_CM 2.56
#define INT32_POS_OF_CM_NUM 64
#define INT32_POS_OF_CM_DEN 25

#define INT32_SPEED_FRAC 19
#define INT32_SPEED_OF_CM_S 5242.88
#define INT32_SPEED_OF_CM_S_NUM 41943
#define INT32_SPEED_OF_CM_S_DEN 8

#define INT32_ACCEL_FRAC 10
#define INT32_MAG_FRAC 11

#define INT32_PERCENTAGE_FRAC 10

struct Int32Vect2 {
  int32_t x;
  int32_t y;
};

struct Int32Vect3 {
  int32_t x;
  int32_t y;
  int32_t z;
};

/* Rotation quaternions                         */
#define INT32_QUAT_FRAC 15
/**
 * @brief Rotation quaternion
 * @details Units: BFP with #INT32_QUAT_FRAC */
struct Int32Quat {
  int32_t qi;
  int32_t qx;
  int32_t qy;
  int32_t qz;
};


struct Int64Quat {
  int64_t qi;
  int64_t qx;
  int64_t qy;
  int64_t qz;
};


/* Euler angles                                 */
#define INT32_ANGLE_FRAC 12
#define INT32_RATE_FRAC 12
#define INT32_ANGLE_PI_4   (int32_t)ANGLE_BFP_OF_REAL(   0.7853981633974483096156608458198757)
#define INT32_ANGLE_PI_2   (int32_t)ANGLE_BFP_OF_REAL(   1.5707963267948966192313216916397514)
#define INT32_ANGLE_PI     (int32_t)ANGLE_BFP_OF_REAL(   3.1415926535897932384626433832795029)
#define INT32_ANGLE_2_PI   (int32_t)ANGLE_BFP_OF_REAL(2.*3.1415926535897932384626433832795029)

#define INT32_RAD_OF_DEG(_deg) (int32_t)(((int64_t)(_deg) * 14964008)/857374503)
#define INT32_DEG_OF_RAD(_rad) (int32_t)(((int64_t)(_rad) * 857374503)/14964008)

#define INT32_ANGLE_NORMALIZE(_a) {             \
    while ((_a) > INT32_ANGLE_PI)  (_a) -= INT32_ANGLE_2_PI;    \
    while ((_a) < -INT32_ANGLE_PI) (_a) += INT32_ANGLE_2_PI;    \
  }

#define INT32_COURSE_NORMALIZE(_a) {                \
    while ((_a) < 0) (_a) += INT32_ANGLE_2_PI;                  \
    while ((_a) >= INT32_ANGLE_2_PI)  (_a) -= INT32_ANGLE_2_PI; \
  }


struct Int16Eulers {
  int16_t phi;
  int16_t theta;
  int16_t psi;
};

/**
 * @brief euler angles
 * @details Units: rad in BFP with #INT32_ANGLE_FRAC */
struct Int32Eulers {
  int32_t phi;   ///< in rad with #INT32_ANGLE_FRAC
  int32_t theta; ///< in rad with #INT32_ANGLE_FRAC
  int32_t psi;   ///< in rad with #INT32_ANGLE_FRAC
};


/* Rotation matrix. */
#define INT32_TRIG_FRAC 14

/**
 * @brief rotation matrix
 * @details Units: rad in BFP with #INT32_TRIG_FRAC */
struct Int32RMat {
  int32_t m[3 * 3];
};

/* 3x3 matrix                                    */
struct Int32Mat33 {
  int32_t m[3 * 3];
};

/* Rotational speed                              */
struct Int16Rates {
  int16_t p;
  int16_t q;
  int16_t r;
};

/* Rotational speed                              */
/**
 * @brief angular rates
 * @details Units: rad/s in BFP with #INT32_RATE_FRAC */
struct Int32Rates {
  int32_t p; ///< in rad/s with #INT32_RATE_FRAC
  int32_t q; ///< in rad/s with #INT32_RATE_FRAC
  int32_t r; ///< in rad/s with #INT32_RATE_FRAC
};

struct Int64Rates {
  int64_t p;
  int64_t q;
  int64_t r;
};


struct Int64Vect2 {
  int64_t x;
  int64_t y;
};

struct Int64Vect3 {
  int64_t x;
  int64_t y;
  int64_t z;
};


// Real (floating point) ->  Binary Fixed Point  (int)
#define LBFP_OF_REAL(_vr, _frac)   ((_vr)*(1LL<<(_frac)))
#define BFP_OF_REAL(_vr, _frac)    ((_vr)*(1<<(_frac)))
#define FLOAT_OF_BFP(_vbfp, _frac) ((float)(_vbfp)/(1<<(_frac)))
#define DOUBLE_OF_BFP(_vbfp, _frac) ((double)(_vbfp)/(1<<(_frac)))
#define RATE_BFP_OF_REAL(_af)   BFP_OF_REAL((_af), INT32_RATE_FRAC)
#define RATE_FLOAT_OF_BFP(_ai)  FLOAT_OF_BFP((_ai), INT32_RATE_FRAC)
#define ANGLE_BFP_OF_REAL(_af)  BFP_OF_REAL((_af), INT32_ANGLE_FRAC)
#define ANGLE_FLOAT_OF_BFP(_ai) FLOAT_OF_BFP((_ai), INT32_ANGLE_FRAC)
#define QUAT1_BFP_OF_REAL(_qf)  BFP_OF_REAL((_qf), INT32_QUAT_FRAC)
#define QUAT1_FLOAT_OF_BFP(_qi) FLOAT_OF_BFP((_qi), INT32_QUAT_FRAC)
#define TRIG_BFP_OF_REAL(_tf)   BFP_OF_REAL((_tf), INT32_TRIG_FRAC)
#define TRIG_FLOAT_OF_BFP(_ti)  FLOAT_OF_BFP((_ti),INT32_TRIG_FRAC)
#define POS_BFP_OF_REAL(_af)    BFP_OF_REAL((_af), INT32_POS_FRAC)
#define POS_FLOAT_OF_BFP(_ai)   FLOAT_OF_BFP((_ai), INT32_POS_FRAC)
#define SPEED_BFP_OF_REAL(_af)  BFP_OF_REAL((_af), INT32_SPEED_FRAC)
#define SPEED_FLOAT_OF_BFP(_ai) FLOAT_OF_BFP((_ai), INT32_SPEED_FRAC)
#define ACCEL_BFP_OF_REAL(_af)  BFP_OF_REAL((_af), INT32_ACCEL_FRAC)
#define ACCEL_FLOAT_OF_BFP(_ai) FLOAT_OF_BFP((_ai), INT32_ACCEL_FRAC)
#define MAG_BFP_OF_REAL(_af)    BFP_OF_REAL((_af), INT32_MAG_FRAC)
#define MAG_FLOAT_OF_BFP(_ai)   FLOAT_OF_BFP((_ai), INT32_MAG_FRAC)

#define INT_MULT_RSHIFT(_a, _b, _r) (((_a)*(_b))>>(_r))


extern uint32_t int32_sqrt(uint32_t in);
extern uint32_t int32_gcd(uint32_t a, uint32_t b);
#define INT32_SQRT(_out,_in) WARNING("INT32_SQRT macro is deprecated, use the lower case function instead") { _out = int32_sqrt(_in); }


/*
 * Dimension 2 Vectors
 */

#define INT_VECT2_ZERO(_v) VECT2_ASSIGN(_v, 0, 0)

/* macros also usable if _v is not a Int32Vect2, but a different struct with x,y members */
#define INT32_VECT2_NORM(_v) int32_sqrt(VECT2_NORM2(_v))

/** return squared norm of 2D vector */
static inline uint32_t int32_vect2_norm2(struct Int32Vect2 *v)
{
  return v->x * v->x + v->y * v->y;
}

/** return norm of 2D vector */
static inline uint32_t int32_vect2_norm(struct Int32Vect2 *v)
{
  return int32_sqrt(int32_vect2_norm2(v));
}

/** normalize 2D vector inplace */
static inline void int32_vect2_normalize(struct Int32Vect2 *v, uint8_t frac)
{
  const uint32_t n = int32_vect2_norm(v);
  if (n > 0) {
    const int32_t f = BFP_OF_REAL((1.), frac);
    v->x = v->x * f / (int32_t)n;
    v->y = v->y * f / (int32_t)n;
  }
}

#define INT32_VECT2_NORMALIZE(_v,_frac) WARNING("INT32_VECT2_NORMALIZE macro is deprecated, use the lower case function instead") int32_vect2_normalize(&(_v), _frac)


#define INT32_VECT2_RSHIFT(_o, _i, _r) { \
    (_o).x = ((_i).x >> (_r)); \
    (_o).y = ((_i).y >> (_r)); \
  }

#define INT32_VECT2_LSHIFT(_o, _i, _l) { \
    (_o).x = ((_i).x << (_l)); \
    (_o).y = ((_i).y << (_l)); \
  }

#define INT32_VECT2_SCALE_2(_a, _b, _num, _den) {   \
    (_a).x = ((_b).x * (_num)) / (_den);        \
    (_a).y = ((_b).y * (_num)) / (_den);        \
  }

/*
 * Dimension 3 Vectors
 */

#define INT_VECT3_ZERO(_v) VECT3_ASSIGN(_v, 0, 0, 0)
#define INT32_VECT3_ZERO(_v) VECT3_ASSIGN(_v, 0, 0, 0)

#define INT32_VECT3_SCALE_2(_a, _b, _num, _den) {   \
    (_a).x = ((_b).x * (_num)) / (_den);        \
    (_a).y = ((_b).y * (_num)) / (_den);        \
    (_a).z = ((_b).z * (_num)) / (_den);        \
  }

#define INT32_VECT3_NORM(_v) int32_sqrt(VECT3_NORM2(_v))

#define INT32_VECT3_RSHIFT(_o, _i, _r) { \
    (_o).x = ((_i).x >> (_r));       \
    (_o).y = ((_i).y >> (_r));       \
    (_o).z = ((_i).z >> (_r));       \
  }

#define INT32_VECT3_LSHIFT(_o, _i, _l) { \
    (_o).x = ((_i).x << (_l));       \
    (_o).y = ((_i).y << (_l));       \
    (_o).z = ((_i).z << (_l));       \
  }



/*
 * 3x3 Matrices
 */
#define INT32_MAT33_ZERO(_m) {                      \
    MAT33_ELMT((_m), 0, 0) = 0;                     \
    MAT33_ELMT((_m), 0, 1) = 0;                     \
    MAT33_ELMT((_m), 0, 2) = 0;                     \
    MAT33_ELMT((_m), 1, 0) = 0;                     \
    MAT33_ELMT((_m), 1, 1) = 0;                     \
    MAT33_ELMT((_m), 1, 2) = 0;                     \
    MAT33_ELMT((_m), 2, 0) = 0;                     \
    MAT33_ELMT((_m), 2, 1) = 0;                     \
    MAT33_ELMT((_m), 2, 2) = 0;                     \
  }

#define INT32_MAT33_DIAG(_m, _d00, _d11, _d22) {    \
    MAT33_ELMT((_m), 0, 0) = (_d00);                \
    MAT33_ELMT((_m), 0, 1) = 0;                     \
    MAT33_ELMT((_m), 0, 2) = 0;                     \
    MAT33_ELMT((_m), 1, 0) = 0;                     \
    MAT33_ELMT((_m), 1, 1) = (_d11);                \
    MAT33_ELMT((_m), 1, 2) = 0;                     \
    MAT33_ELMT((_m), 2, 0) = 0;                     \
    MAT33_ELMT((_m), 2, 1) = 0;                     \
    MAT33_ELMT((_m), 2, 2) = (_d22);                \
  }



/*
 * Rotation matrices
 */

/** initialises a rotation matrix to identity */
static inline void int32_rmat_identity(struct Int32RMat *rm)
{
  INT32_MAT33_DIAG(*rm, TRIG_BFP_OF_REAL(1.), TRIG_BFP_OF_REAL(1.), TRIG_BFP_OF_REAL(1.));
}

/** Composition (multiplication) of two rotation matrices.
 * m_a2c = m_a2b comp m_b2c , aka  m_a2c = m_b2c * m_a2b
 */
extern void int32_rmat_comp(struct Int32RMat *m_a2c, const struct Int32RMat *m_a2b,
                            const struct Int32RMat *m_b2c);

/** Composition (multiplication) of two rotation matrices.
 * m_a2b = m_a2c comp_inv m_b2c , aka  m_a2b = inv(_m_b2c) * m_a2c
 */
extern void int32_rmat_comp_inv(struct Int32RMat *m_a2b, const struct Int32RMat *m_a2c,
                                const struct Int32RMat *m_b2c);

/** rotate 3D vector by rotation matrix.
 * vb = m_a2b * va
 */
extern void int32_rmat_vmult(struct Int32Vect3 *vb, struct Int32RMat *m_a2b,
                             struct Int32Vect3 *va);

/** rotate 3D vector by transposed rotation matrix.
 * vb = m_b2a^T * va
 */
extern void int32_rmat_transp_vmult(struct Int32Vect3 *vb, struct Int32RMat *m_b2a,
                                    struct Int32Vect3 *va);

/** rotate anglular rates by rotation matrix.
 * rb = m_a2b * ra
 */
extern void int32_rmat_ratemult(struct Int32Rates *rb, struct Int32RMat *m_a2b,
                                struct Int32Rates *ra);

/** rotate anglular rates by transposed rotation matrix.
 * rb = m_b2a^T * ra
 */
extern void int32_rmat_transp_ratemult(struct Int32Rates *rb, struct Int32RMat *m_b2a,
                                       struct Int32Rates *ra);

/// Convert unit quaternion to rotation matrix.
extern void int32_rmat_of_quat(struct Int32RMat *rm, struct Int32Quat *q);

/** Rotation matrix from 321 Euler angles (int).
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
extern void int32_rmat_of_eulers_321(struct Int32RMat *rm, struct Int32Eulers *e);

/// Rotation matrix from 312 Euler angles.
extern void int32_rmat_of_eulers_312(struct Int32RMat *rm, struct Int32Eulers *e);

/// Rotation matrix from Euler angles.
#define int32_rmat_of_eulers int32_rmat_of_eulers_321

/* defines for backwards compatibility */
#define INT32_RMAT_COMP(_m_a2c, _m_a2b, _m_b2c) WARNING("INT32_RMAT_COMP macro is deprecated, use the lower case function instead") int32_rmat_comp(&(_m_a2c), &(_m_a2b), &(_m_b2c))
#define INT32_RMAT_COMP_INV(_m_a2b, _m_a2c, _m_b2c) WARNING("INT32_RMAT_COMP_INV macro is deprecated, use the lower case function instead") int32_rmat_comp_inv(&(_m_a2b), &(_m_a2c), &(_m_b2c))
#define INT32_RMAT_VMULT(_vb, _m_a2b, _va) WARNING("INT32_RMAT_VMULT macro is deprecated, use the lower case function instead") int32_rmat_vmult(&(_vb), &(_m_a2b), &(_va))
#define INT32_RMAT_TRANSP_VMULT(_vb, _m_b2a, _va) WARNING("INT32_RMAT_TRANSP_VMULT macro is deprecated, use the lower case function instead") int32_rmat_transp_vmult(&(_vb), &(_m_b2a), &(_va))
#define INT32_RMAT_RATEMULT(_rb, _m_a2b, _ra) WARNING("INT32_RMAT_RATEMULT macro is deprecated, use the lower case function instead") int32_rmat_ratemult(&(_rb), &(_m_a2b), &(_ra))
#define INT32_RMAT_TRANSP_RATEMULT(_rb, _m_b2a, _ra) WARNING("INT32_RMAT_TRANSP_RATEMULT macro is deprecated, use the lower case function instead") int32_rmat_ratemult(&(_rb), &(_m_b2a), &(_ra))

#define INT32_RMAT_OF_QUAT(_rm, _q) WARNING("INT32_RMAT_OF_QUAT macro is deprecated, use the lower case function instead") int32_rmat_of_quat(&(_rm), &(_q))
#define INT32_RMAT_OF_EULERS(_rm, _e) WARNING("INT32_RMAT_OF_EULERS macro is deprecated, use the lower case function instead") int32_rmat_of_eulers_321(&(_rm), &(_e))
#define INT32_RMAT_OF_EULERS_321(_rm, _e) WARNING("INT32_RMAT_OF_EULERS_321 macro is deprecated, use the lower case function instead") int32_rmat_of_eulers_321(&(_rm), &(_e))
#define INT32_RMAT_OF_EULERS_312(_rm, _e) WARNING("INT32_RMAT_OF_EULERS_312 macro is deprecated, use the lower case function instead") int32_rmat_of_eulers_312(&(_rm), &(_e))


/*
 *
 * Quaternions
 *
 */

/** initialises a quaternion to identity */
static inline void int32_quat_identity(struct Int32Quat *q)
{
  q->qi = QUAT1_BFP_OF_REAL(1);
  q->qx = 0;
  q->qy = 0;
  q->qz = 0;
}

/** Norm of a quaternion.
 */
static inline uint32_t int32_quat_norm(struct Int32Quat *q)
{
  uint32_t n2 = q->qi * q->qi + q->qx * q->qx + q->qy * q->qy + q->qz * q->qz;
  return int32_sqrt(n2);
}

static inline void int32_quat_wrap_shortest(struct Int32Quat *q)
{
  if (q->qi < 0) {
    QUAT_EXPLEMENTARY(*q, *q);
  }
}

/** normalize a quaternion inplace */
static inline void int32_quat_normalize(struct Int32Quat *q)
{
  int32_t n = int32_quat_norm(q);
  if (n > 0) {
    q->qi = q->qi * QUAT1_BFP_OF_REAL(1) / n;
    q->qx = q->qx * QUAT1_BFP_OF_REAL(1) / n;
    q->qy = q->qy * QUAT1_BFP_OF_REAL(1) / n;
    q->qz = q->qz * QUAT1_BFP_OF_REAL(1) / n;
  }
}

/** Composition (multiplication) of two quaternions.
 * a2c = a2b comp b2c , aka  a2c = a2b * b2c
 */
extern void int32_quat_comp(struct Int32Quat *a2c, struct Int32Quat *a2b, struct Int32Quat *b2c);

/** Composition (multiplication) of two quaternions.
 * a2b = a2c comp_inv b2c , aka  a2b = a2c * inv(b2c)
 */
extern void int32_quat_comp_inv(struct Int32Quat *a2b, struct Int32Quat *a2c, struct Int32Quat *b2c);

/** Composition (multiplication) of two quaternions.
 * b2c = a2b inv_comp a2c , aka  b2c = inv(_a2b) * a2c
 */
extern void int32_quat_inv_comp(struct Int32Quat *b2c, struct Int32Quat *a2b, struct Int32Quat *a2c);

/** Composition (multiplication) of two quaternions with normalization.
 * a2c = a2b comp b2c , aka  a2c = a2b * b2c
 */
extern void int32_quat_comp_norm_shortest(struct Int32Quat *a2c, struct Int32Quat *a2b, struct Int32Quat *b2c);

/** Composition (multiplication) of two quaternions with normalization.
 * a2b = a2c comp_inv b2c , aka  a2b = a2c * inv(b2c)
 */
extern void int32_quat_comp_inv_norm_shortest(struct Int32Quat *a2b, struct Int32Quat *a2c, struct Int32Quat *b2c);

/** Composition (multiplication) of two quaternions with normalization.
 * b2c = a2b inv_comp a2c , aka  b2c = inv(_a2b) * a2c
 */
extern void int32_quat_inv_comp_norm_shortest(struct Int32Quat *b2c, struct Int32Quat *a2b, struct Int32Quat *a2c);

/** Quaternion derivative from rotational velocity.
 * qd = -0.5*omega(r) * q
 * or equally:
 * qd = 0.5 * q * omega(r)
 */
extern void int32_quat_derivative(struct Int32Quat *qd, const struct Int32Rates *r, struct Int32Quat *q);

/** in place quaternion first order integration with constant rotational velocity. */
extern void int32_quat_integrate_fi(struct Int32Quat *q, struct Int64Quat *hr, struct Int32Rates *omega, int freq);

/** rotate 3D vector by quaternion.
 * vb = q_a2b * va * q_a2b^-1
 * Doesn't support inplace rotation, meaning v_out mustn't be a pointer to same struct as v_in.
 */
extern void int32_quat_vmult(struct Int32Vect3 *v_out, struct Int32Quat *q, struct Int32Vect3 *v_in);

/// Quaternion from Euler angles.
extern void int32_quat_of_eulers(struct Int32Quat *q, struct Int32Eulers *e);

/** Quaternion from unit vector and angle.
 * Output quaternion is not normalized.
 * The output resolution depends on the resolution of the resolution of the unit vector.
 * If the unit vector has no fractional part (ex: [0, 0, 1]), the quaternion is unitary.
 */
extern void int32_quat_of_axis_angle(struct Int32Quat *q, struct Int32Vect3 *uv, int32_t angle);

/// Quaternion from rotation matrix.
extern void int32_quat_of_rmat(struct Int32Quat *q, struct Int32RMat *r);

/* defines for backwards compatibility */
#define INT32_QUAT_ZERO(_q) WARNING("INT32_QUAT_ZERO macro is deprecated, use the lower case function instead") int32_quat_identity(&(_q))
#define INT32_QUAT_NORM(n, q) WARNING("INT32_QUAT_NORM macro is deprecated, use the lower case function instead") { n = int32_quat_norm(&(q)); }
#define INT32_QUAT_WRAP_SHORTEST(q) WARNING("INT32_QUAT_WRAP_SHORTEST macro is deprecated, use the lower case function instead") int32_quat_wrap_shortest(&(q))
#define INT32_QUAT_NORMALIZE(q) WARNING("INT32_QUAT_NORMALIZE macro is deprecated, use the lower case function instead") int32_quat_normalize(&(q))
#define INT32_QUAT_COMP(_a2c, _a2b, _b2c) WARNING("INT32_QUAT_COMP macro is deprecated, use the lower case function instead") int32_quat_comp(&(_a2c), &(_a2b), &(_b2c))
#define INT32_QUAT_COMP_INV(_a2b, _a2c, _b2c) WARNING("INT32_QUAT_COMP_INV macro is deprecated, use the lower case function instead") int32_quat_comp_inv(&(_a2b), &(_a2c), &(_b2c))
#define INT32_QUAT_INV_COMP(_b2c, _a2b, _a2c) WARNING("INT32_QUAT_INV_COMP macro is deprecated, use the lower case function instead") int32_quat_inv_comp(&(_b2c), &(_a2b), &(_a2c))
#define INT32_QUAT_COMP_NORM_SHORTEST(_a2c, _a2b, _b2c) WARNING("INT32_QUAT_COMP_NORM_SHORTEST macro is deprecated, use the lower case function instead") int32_quat_comp_norm_shortest(&(_a2c), &(_a2b), &(_b2c))
#define INT32_QUAT_INV_COMP_NORM_SHORTEST(_b2c, _a2b, _a2c) WARNING("INT32_QUAT_INV_COMP_NORM_SHORTEST macro is deprecated, use the lower case function instead") int32_quat_inv_comp_norm_shortest(&(_b2c), &(_a2b), &(_a2c))
#define INT32_QUAT_DERIVATIVE(_qd, _r, _q) WARNING("INT32_QUAT_DERIVATIVE macro is deprecated, use the lower case function instead") int32_quat_derivative(&(_qd), &(_r), &(_q))
#define INT32_QUAT_INTEGRATE_FI(_q, _hr, _omega, _f) WARNING("INT32_QUAT_INTEGRATE_FI macro is deprecated, use the lower case function instead") int32_quat_integrate_fi(&(_q), &(_hr), &(_omega), _f)
#define INT32_QUAT_VMULT(v_out, q, v_in) WARNING("INT32_QUAT_VMULT macro is deprecated, use the lower case function instead") int32_quat_vmult(&(v_out), &(q), &(v_in))
#define INT32_QUAT_OF_EULERS(_q, _e) WARNING("INT32_QUAT_OF_EULERS macro is deprecated, use the lower case function instead") int32_quat_of_eulers(&(_q), &(_e))
#define INT32_QUAT_OF_AXIS_ANGLE(_q, _uv, _an) WARNING("INT32_QUAT_OF_AXIS_ANGLE macro is deprecated, use the lower case function instead") int32_quat_of_axis_angle(&(_q), &(_uv), _an)
#define INT32_QUAT_OF_RMAT(_q, _r) WARNING("INT32_QUAT_OF_RMAT macro is deprecated, use the lower case function instead") int32_quat_of_rmat(&(_q), &(_r))


/*
 *
 * Euler angles
 *
 */

#define INT_EULERS_ZERO(_e) EULERS_ASSIGN(_e, 0, 0, 0)

extern void int32_eulers_of_rmat(struct Int32Eulers *e, struct Int32RMat *rm);
extern void int32_eulers_of_quat(struct Int32Eulers *e, struct Int32Quat *q);

/* defines for backwards compatibility */
#define INT32_EULERS_OF_RMAT(_e, _rm) WARNING("INT32_EULERS_OF_RMAT macro is deprecated, use the lower case function instead") int32_eulers_of_rmat(&(_e), &(_rm))
#define INT32_EULERS_OF_QUAT(_e, _q) WARNING("INT32_EULERS_OF_QUAT macro is deprecated, use the lower case function instead") int32_eulers_of_quat(&(_e), &(_q))

#define INT32_EULERS_LSHIFT(_o, _i, _r) {  \
    (_o).phi   = ((_i).phi   << (_r));     \
    (_o).theta = ((_i).theta << (_r));     \
    (_o).psi   = ((_i).psi   << (_r));     \
  }

#define INT32_EULERS_RSHIFT(_o, _i, _r) {  \
    (_o).phi   = ((_i).phi   >> (_r));     \
    (_o).theta = ((_i).theta >> (_r));     \
    (_o).psi   = ((_i).psi   >> (_r));     \
  }


/*
 * Rotational speeds
 */

#define INT_RATES_ZERO(_e) RATES_ASSIGN(_e, 0, 0, 0)

#define INT_RATES_RSHIFT(_o, _i, _r) {   \
    (_o).p = ((_i).p >> (_r));       \
    (_o).q = ((_i).q >> (_r));       \
    (_o).r = ((_i).r >> (_r));       \
  }

#define INT_RATES_LSHIFT(_o, _i, _r) {   \
    (_o).p = ((_i).p << (_r));       \
    (_o).q = ((_i).q << (_r));       \
    (_o).r = ((_i).r << (_r));       \
  }


extern void int32_rates_of_eulers_dot_321(struct Int32Rates *r, struct Int32Eulers *e, struct Int32Eulers *ed);
extern void int32_eulers_dot_321_of_rates(struct Int32Eulers *ed, struct Int32Eulers *e, struct Int32Rates *r);

#define int32_eulers_dot_of_rates int32_eulers_dot_321_of_rates

/* defines for backwards compatibility */
#define INT32_RATES_OF_EULERS_DOT_321(_r, _e, _ed) WARNING("INT32_RATES_OF_EULERS_DOT_321 macro is deprecated, use the lower case function instead") int32_rates_of_eulers_dot_321(&(_r), &(_e), &(_ed))
#define INT32_RATES_OF_EULERS_DOT(_r, _e, _ed)     WARNING("INT32_RATES_OF_EULERS_DOT macro is deprecated, use the lower case function instead") int32_rates_of_eulers_dot_321(&(_r), &(_e), &(_ed))
#define INT32_EULERS_DOT_321_OF_RATES(_ed, _e, _r) WARNING("INT32_EULERS_DOT_321_OF_RATES macro is deprecated, use the lower case function instead") int32_eulers_dot_321_of_rates(&(_ed), &(_e), &(_r))
#define INT32_EULERS_DOT_OF_RATES(_ed, _e, _r)     WARNING("INT32_EULERS_DOT_OF_RATES macro is deprecated, use the lower case function instead") int32_eulers_dot_321_of_rates(&(_ed), &(_e), &(_r))


//
//
// Generic int32_t vector algebra
//
//

/** a = 0 */
static inline void int32_vect_zero(int32_t *a, const int n)
{
  int i;
  for (i = 0; i < n; i++) { a[i] = 0.; }
}

/** a = 0 */
static inline void int16_vect_zero(int16_t *a, const int n)
{
  int i;
  for (i = 0; i < n; i++) { a[i] = 0.; }
}

/** a = v * ones(n,1) */
static inline void int32_vect_set_value(int32_t *a, const int32_t v, const int n)
{
  int i;
  for(i = 0 ; i < n; i++) { a[i] = v; }
}


/** a = b */
static inline void int32_vect_copy(int32_t *a, const int32_t *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { a[i] = b[i]; }
}

/** o = a + b */
static inline void int32_vect_sum(int32_t *o, const int32_t *a, const int32_t *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] + b[i]; }
}

/** o = a - b */
static inline void int32_vect_diff(int32_t *o, const int32_t *a, const int32_t *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] - b[i]; }
}

/** o = a * b (element wise) */
static inline void int32_vect_mul(int32_t *o, const int32_t *a, const int32_t *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] * b[i]; }
}

/** a += b */
static inline void int32_vect_add(int32_t *a, const int32_t *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { a[i] += b[i]; }
}

/** a -= b */
static inline void int32_vect_sub(int32_t *a, const int32_t *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { a[i] -= b[i]; }
}

/** o = a * s */
static inline void int32_vect_smul(int32_t *o, const int32_t *a, const int32_t s, const int n)
{
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] * s; }
}

/** Find value s in array a. Returns 1 if found, 0 if not found.
 * If the value is found loc = index of found value in array, else loc = -1 
 */
static inline bool int32_vect_find(const int32_t *a, const int32_t s, int *loc, const int n) 
{
  int i;
  for (i = 0; i < n; i++) {
    if (a[i] == s) {
      *loc = i;
      return true;
    }
  }
  *loc = -1;
  return false;
}

//
//
// Generic matrix algebra
//
//

/** o = a * b
 *
 * a: [m x n]
 * b: [n x l]
 * o: [m x l]
 */
static inline void int32_mat_mul(int32_t **o, int32_t **a, int32_t **b, int m, int n, int l)
{
  int i, j, k;
  for (i = 0; i < m; i++) {
    for (j = 0; j < l; j++) {
      o[i][j] = 0;
      for (k = 0; k < n; k++) {
        o[i][j] += a[i][k] * b[k][j];
      }
    }
  }
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_ALGEBRA_INT_H */
/** @}*/
/** @}*/
