/*
 * Copyright (C) 2008-2011 The Paparazzi Team
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
 *
 */

/**
 * @file pprz_algebra_int.h
 *   @brief Paparazzi fixed point algebra.
 *
 *   This is the more detailed description of this file.
 *
 */

#ifndef PPRZ_ALGEBRA_INT_H
#define PPRZ_ALGEBRA_INT_H


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
  int32_t m[3*3];
};

/* 3x3 matrix                                    */
struct Int32Mat33 {
  int32_t m[3*3];
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
/*
 * Dimension 2 Vectors
 */

#define INT_VECT2_ZERO(_v) VECT2_ASSIGN(_v, 0, 0)

#define INT_VECT2_ASSIGN(_a, _x, _y) VECT2_ASSIGN(_a, _x, _y)

#define INT32_VECT2_NORM(n, v) {            \
    int32_t n2 = (v).x*(v).x + (v).y*(v).y; \
    INT32_SQRT(n, n2);                  \
  }

#define INT32_VECT2_NORMALIZE(_v,_frac) {				\
    int32_t n;								\
    INT32_VECT2_NORM(n, _v);						\
    INT32_VECT2_SCALE_2(_v, _v, BFP_OF_REAL((1.),_frac) , n);		\
  }


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

#define INT_VECT3_ASSIGN(_a, _x, _y, _z) VECT3_ASSIGN(_a, _x, _y, _z)
#define INT32_VECT3_ASSIGN(_a, _x, _y, _z) VECT3_ASSIGN(_a, _x, _y, _z)

#define INT32_VECT3_COPY(_o, _i) VECT3_COPY(_o, _i)

#define INT32_VECT3_SUM(_c, _a, _b) VECT3_SUM(_c, _a, _b)

#define INT32_VECT3_DIFF(_c, _a, _b) VECT3_DIFF(_c, _a, _b)

#define INT32_VECT3_ADD(_a, _b) VECT3_ADD(_a, _b)

#define INT32_VECT3_SCALE_2(_a, _b, _num, _den) {   \
    (_a).x = ((_b).x * (_num)) / (_den);        \
    (_a).y = ((_b).y * (_num)) / (_den);        \
    (_a).z = ((_b).z * (_num)) / (_den);        \
  }

#define INT32_VECT3_SDIV(_a, _b, _s) VECT3_SDIV(_a, _b, _s)


#define INT32_VECT3_NORM(n, v) {                \
    int32_t n2 = (v).x*(v).x + (v).y*(v).y + (v).z*(v).z;   \
    INT32_SQRT(n, n2);                  \
  }

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

#define INT32_VECT3_CROSS_PRODUCT(_vo, _v1, _v2) {          \
    (_vo).x = (_v1).y*(_v2).z - (_v1).z*(_v2).y;            \
    (_vo).y = (_v1).z*(_v2).x - (_v1).x*(_v2).z;            \
    (_vo).z = (_v1).x*(_v2).y - (_v1).y*(_v2).x;            \
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

#define INT32_MAT33_DIAG(_m, _d00, _d11, _d22) {            \
    MAT33_ELMT((_m), 0, 0) = (_d00);                    \
    MAT33_ELMT((_m), 0, 1) = 0;                     \
    MAT33_ELMT((_m), 0, 2) = 0;                     \
    MAT33_ELMT((_m), 1, 0) = 0;                     \
    MAT33_ELMT((_m), 1, 1) = (_d11);                    \
    MAT33_ELMT((_m), 1, 2) = 0;                     \
    MAT33_ELMT((_m), 2, 0) = 0;                     \
    MAT33_ELMT((_m), 2, 1) = 0;                     \
    MAT33_ELMT((_m), 2, 2) = (_d22);                    \
  }


#define INT32_MAT33_VECT3_MUL(_o, _m, _v, _f) {         \
    (_o).x = ((_m).m[0]*(_v).x + (_m).m[1]*(_v).y + (_m).m[2]*(_v).z)>>(_f);    \
    (_o).y = ((_m).m[3]*(_v).x + (_m).m[4]*(_v).y + (_m).m[5]*(_v).z)>>(_f);    \
    (_o).z = ((_m).m[6]*(_v).x + (_m).m[7]*(_v).y + (_m).m[8]*(_v).z)>>(_f);    \
  }

/*
 * Rotation matrices
 */

#define INT32_RMAT_ZERO(_rm)                        \
  INT32_MAT33_DIAG(_rm, TRIG_BFP_OF_REAL( 1.), TRIG_BFP_OF_REAL( 1.), TRIG_BFP_OF_REAL( 1.))

/* _m_a2c = _m_a2b comp _m_b2c , aka  _m_a2c = _m_b2c * _m_a2b */
#define INT32_RMAT_COMP(_m_a2c, _m_a2b, _m_b2c) {           \
    (_m_a2c).m[0] = ((_m_b2c).m[0]*(_m_a2b).m[0] + (_m_b2c).m[1]*(_m_a2b).m[3] + (_m_b2c).m[2]*(_m_a2b).m[6])>>INT32_TRIG_FRAC; \
    (_m_a2c).m[1] = ((_m_b2c).m[0]*(_m_a2b).m[1] + (_m_b2c).m[1]*(_m_a2b).m[4] + (_m_b2c).m[2]*(_m_a2b).m[7])>>INT32_TRIG_FRAC; \
    (_m_a2c).m[2] = ((_m_b2c).m[0]*(_m_a2b).m[2] + (_m_b2c).m[1]*(_m_a2b).m[5] + (_m_b2c).m[2]*(_m_a2b).m[8])>>INT32_TRIG_FRAC; \
    (_m_a2c).m[3] = ((_m_b2c).m[3]*(_m_a2b).m[0] + (_m_b2c).m[4]*(_m_a2b).m[3] + (_m_b2c).m[5]*(_m_a2b).m[6])>>INT32_TRIG_FRAC; \
    (_m_a2c).m[4] = ((_m_b2c).m[3]*(_m_a2b).m[1] + (_m_b2c).m[4]*(_m_a2b).m[4] + (_m_b2c).m[5]*(_m_a2b).m[7])>>INT32_TRIG_FRAC; \
    (_m_a2c).m[5] = ((_m_b2c).m[3]*(_m_a2b).m[2] + (_m_b2c).m[4]*(_m_a2b).m[5] + (_m_b2c).m[5]*(_m_a2b).m[8])>>INT32_TRIG_FRAC; \
    (_m_a2c).m[6] = ((_m_b2c).m[6]*(_m_a2b).m[0] + (_m_b2c).m[7]*(_m_a2b).m[3] + (_m_b2c).m[8]*(_m_a2b).m[6])>>INT32_TRIG_FRAC; \
    (_m_a2c).m[7] = ((_m_b2c).m[6]*(_m_a2b).m[1] + (_m_b2c).m[7]*(_m_a2b).m[4] + (_m_b2c).m[8]*(_m_a2b).m[7])>>INT32_TRIG_FRAC; \
    (_m_a2c).m[8] = ((_m_b2c).m[6]*(_m_a2b).m[2] + (_m_b2c).m[7]*(_m_a2b).m[5] + (_m_b2c).m[8]*(_m_a2b).m[8])>>INT32_TRIG_FRAC; \
  }

/* _m_a2b = _m_a2c comp_inv _m_b2c , aka  _m_a2b = inv(_m_b2c) * _m_a2c */
#define INT32_RMAT_COMP_INV(_m_a2b, _m_a2c, _m_b2c) {                                                     \
    (_m_a2b).m[0] = ((_m_b2c).m[0]*(_m_a2c).m[0] + (_m_b2c).m[3]*(_m_a2c).m[3] + (_m_b2c).m[6]*(_m_a2c).m[6])>>INT32_TRIG_FRAC; \
    (_m_a2b).m[1] = ((_m_b2c).m[0]*(_m_a2c).m[1] + (_m_b2c).m[3]*(_m_a2c).m[4] + (_m_b2c).m[6]*(_m_a2c).m[7])>>INT32_TRIG_FRAC; \
    (_m_a2b).m[2] = ((_m_b2c).m[0]*(_m_a2c).m[2] + (_m_b2c).m[3]*(_m_a2c).m[5] + (_m_b2c).m[6]*(_m_a2c).m[8])>>INT32_TRIG_FRAC; \
    (_m_a2b).m[3] = ((_m_b2c).m[1]*(_m_a2c).m[0] + (_m_b2c).m[4]*(_m_a2c).m[3] + (_m_b2c).m[7]*(_m_a2c).m[6])>>INT32_TRIG_FRAC; \
    (_m_a2b).m[4] = ((_m_b2c).m[1]*(_m_a2c).m[1] + (_m_b2c).m[4]*(_m_a2c).m[4] + (_m_b2c).m[7]*(_m_a2c).m[7])>>INT32_TRIG_FRAC; \
    (_m_a2b).m[5] = ((_m_b2c).m[1]*(_m_a2c).m[2] + (_m_b2c).m[4]*(_m_a2c).m[5] + (_m_b2c).m[7]*(_m_a2c).m[8])>>INT32_TRIG_FRAC; \
    (_m_a2b).m[6] = ((_m_b2c).m[2]*(_m_a2c).m[0] + (_m_b2c).m[5]*(_m_a2c).m[3] + (_m_b2c).m[8]*(_m_a2c).m[6])>>INT32_TRIG_FRAC; \
    (_m_a2b).m[7] = ((_m_b2c).m[2]*(_m_a2c).m[1] + (_m_b2c).m[5]*(_m_a2c).m[4] + (_m_b2c).m[8]*(_m_a2c).m[7])>>INT32_TRIG_FRAC; \
    (_m_a2b).m[8] = ((_m_b2c).m[2]*(_m_a2c).m[2] + (_m_b2c).m[5]*(_m_a2c).m[5] + (_m_b2c).m[8]*(_m_a2c).m[8])>>INT32_TRIG_FRAC; \
  }

/* _vb = _m_a2b * _va */
#define INT32_RMAT_VMULT(_vb, _m_a2b, _va) {                                                 \
    (_vb).x = ( (_m_a2b).m[0]*(_va).x + (_m_a2b).m[1]*(_va).y + (_m_a2b).m[2]*(_va).z)>>INT32_TRIG_FRAC; \
    (_vb).y = ( (_m_a2b).m[3]*(_va).x + (_m_a2b).m[4]*(_va).y + (_m_a2b).m[5]*(_va).z)>>INT32_TRIG_FRAC; \
    (_vb).z = ( (_m_a2b).m[6]*(_va).x + (_m_a2b).m[7]*(_va).y + (_m_a2b).m[8]*(_va).z)>>INT32_TRIG_FRAC; \
  }

#define INT32_RMAT_TRANSP_VMULT(_vb, _m_b2a, _va) {                                      \
    (_vb).x = ( (_m_b2a).m[0]*(_va).x + (_m_b2a).m[3]*(_va).y + (_m_b2a).m[6]*(_va).z)>>INT32_TRIG_FRAC; \
    (_vb).y = ( (_m_b2a).m[1]*(_va).x + (_m_b2a).m[4]*(_va).y + (_m_b2a).m[7]*(_va).z)>>INT32_TRIG_FRAC; \
    (_vb).z = ( (_m_b2a).m[2]*(_va).x + (_m_b2a).m[5]*(_va).y + (_m_b2a).m[8]*(_va).z)>>INT32_TRIG_FRAC; \
  }

#define INT32_RMAT_RATEMULT(_vb, _m_a2b, _va) {                  \
    (_vb).p = ( (_m_a2b).m[0]*(_va).p + (_m_a2b).m[1]*(_va).q + (_m_a2b).m[2]*(_va).r)>>INT32_TRIG_FRAC; \
    (_vb).q = ( (_m_a2b).m[3]*(_va).p + (_m_a2b).m[4]*(_va).q + (_m_a2b).m[5]*(_va).r)>>INT32_TRIG_FRAC; \
    (_vb).r = ( (_m_a2b).m[6]*(_va).p + (_m_a2b).m[7]*(_va).q + (_m_a2b).m[8]*(_va).r)>>INT32_TRIG_FRAC; \
  }

#define INT32_RMAT_TRANSP_RATEMULT(_vb, _m_b2a, _va) {                                       \
    (_vb).p = ( (_m_b2a).m[0]*(_va).p + (_m_b2a).m[3]*(_va).q + (_m_b2a).m[6]*(_va).r)>>INT32_TRIG_FRAC; \
    (_vb).q = ( (_m_b2a).m[1]*(_va).p + (_m_b2a).m[4]*(_va).q + (_m_b2a).m[7]*(_va).r)>>INT32_TRIG_FRAC; \
    (_vb).r = ( (_m_b2a).m[2]*(_va).p + (_m_b2a).m[5]*(_va).q + (_m_b2a).m[8]*(_va).r)>>INT32_TRIG_FRAC; \
  }


/*
 * http://www.mathworks.com/access/helpdesk_r13/help/toolbox/aeroblks/quaternionstodirectioncosinematrix.html
 */
#define INT32_RMAT_OF_QUAT(_rm, _q) {                                   \
    const int32_t _2qi2_m1  = INT_MULT_RSHIFT((_q).qi,(_q).qi, INT32_QUAT_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC-1)-TRIG_BFP_OF_REAL( 1); \
    (_rm).m[0]      = INT_MULT_RSHIFT((_q).qx,(_q).qx, INT32_QUAT_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC-1);  \
    (_rm).m[4]      = INT_MULT_RSHIFT((_q).qy,(_q).qy, INT32_QUAT_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC-1);  \
    (_rm).m[8]      = INT_MULT_RSHIFT((_q).qz,(_q).qz, INT32_QUAT_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC-1);  \
                                                            \
    const int32_t _2qiqx   = INT_MULT_RSHIFT((_q).qi,(_q).qx, INT32_QUAT_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC-1);   \
    const int32_t _2qiqy   = INT_MULT_RSHIFT((_q).qi,(_q).qy, INT32_QUAT_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC-1);   \
    const int32_t _2qiqz   = INT_MULT_RSHIFT((_q).qi,(_q).qz, INT32_QUAT_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC-1);   \
    (_rm).m[1]      = INT_MULT_RSHIFT((_q).qx,(_q).qy, INT32_QUAT_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC-1);  \
    (_rm).m[2]      = INT_MULT_RSHIFT((_q).qx,(_q).qz, INT32_QUAT_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC-1);  \
    (_rm).m[5]      = INT_MULT_RSHIFT((_q).qy,(_q).qz, INT32_QUAT_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC-1);  \
                            \
    (_rm).m[0] += _2qi2_m1;             \
    (_rm).m[3] = (_rm).m[1]-_2qiqz;         \
    (_rm).m[6] = (_rm).m[2]+_2qiqy;         \
    (_rm).m[7] = (_rm).m[5]-_2qiqx;         \
    (_rm).m[4] += _2qi2_m1;             \
    (_rm).m[1] += _2qiqz;               \
    (_rm).m[2] -= _2qiqy;               \
    (_rm).m[5] += _2qiqx;               \
    (_rm).m[8] += _2qi2_m1;             \
  }


/*
 * http://www.mathworks.com/access/helpdesk_r13/help/toolbox/aeroblks/euleranglestodirectioncosinematrix.html
 */

#define INT32_RMAT_OF_EULERS(_rm, _e) INT32_RMAT_OF_EULERS_321(_rm, _e)

#define INT32_RMAT_OF_EULERS_321(_rm, _e) {             \
                                    \
    int32_t sphi;                           \
    PPRZ_ITRIG_SIN(sphi, (_e).phi);                 \
    int32_t cphi;                           \
    PPRZ_ITRIG_COS(cphi, (_e).phi);                 \
    int32_t stheta;                         \
    PPRZ_ITRIG_SIN(stheta, (_e).theta);                 \
    int32_t ctheta;                         \
    PPRZ_ITRIG_COS(ctheta, (_e).theta);                 \
    int32_t spsi;                           \
    PPRZ_ITRIG_SIN(spsi, (_e).psi);                 \
    int32_t cpsi;                           \
    PPRZ_ITRIG_COS(cpsi, (_e).psi);                 \
                                        \
    int32_t ctheta_cpsi = INT_MULT_RSHIFT(ctheta, cpsi,   INT32_TRIG_FRAC); \
    int32_t ctheta_spsi = INT_MULT_RSHIFT(ctheta, spsi,   INT32_TRIG_FRAC); \
    int32_t cphi_spsi   = INT_MULT_RSHIFT(cphi,   spsi,   INT32_TRIG_FRAC); \
    int32_t cphi_cpsi   = INT_MULT_RSHIFT(cphi,   cpsi,   INT32_TRIG_FRAC); \
    int32_t cphi_ctheta = INT_MULT_RSHIFT(cphi,   ctheta, INT32_TRIG_FRAC); \
    int32_t cphi_stheta = INT_MULT_RSHIFT(cphi,   stheta, INT32_TRIG_FRAC); \
    int32_t sphi_ctheta = INT_MULT_RSHIFT(sphi,   ctheta, INT32_TRIG_FRAC); \
    int32_t sphi_stheta = INT_MULT_RSHIFT(sphi,   stheta, INT32_TRIG_FRAC); \
    int32_t sphi_spsi   = INT_MULT_RSHIFT(sphi,   spsi,   INT32_TRIG_FRAC); \
    int32_t sphi_cpsi   = INT_MULT_RSHIFT(sphi,   cpsi,   INT32_TRIG_FRAC); \
                                        \
    int32_t sphi_stheta_cpsi = INT_MULT_RSHIFT(sphi_stheta, cpsi, INT32_TRIG_FRAC); \
    int32_t sphi_stheta_spsi = INT_MULT_RSHIFT(sphi_stheta, spsi, INT32_TRIG_FRAC); \
    int32_t cphi_stheta_cpsi = INT_MULT_RSHIFT(cphi_stheta, cpsi, INT32_TRIG_FRAC); \
    int32_t cphi_stheta_spsi = INT_MULT_RSHIFT(cphi_stheta, spsi, INT32_TRIG_FRAC); \
                                        \
    RMAT_ELMT(_rm, 0, 0) = ctheta_cpsi;                 \
    RMAT_ELMT(_rm, 0, 1) = ctheta_spsi;                 \
    RMAT_ELMT(_rm, 0, 2) = -stheta;                 \
    RMAT_ELMT(_rm, 1, 0) = sphi_stheta_cpsi - cphi_spsi;        \
    RMAT_ELMT(_rm, 1, 1) = sphi_stheta_spsi + cphi_cpsi;        \
    RMAT_ELMT(_rm, 1, 2) = sphi_ctheta;                 \
    RMAT_ELMT(_rm, 2, 0) = cphi_stheta_cpsi + sphi_spsi;        \
    RMAT_ELMT(_rm, 2, 1) = cphi_stheta_spsi - sphi_cpsi;        \
    RMAT_ELMT(_rm, 2, 2) = cphi_ctheta;                 \
                                        \
  }


#define INT32_RMAT_OF_EULERS_312(_rm, _e) {             \
                                        \
    int32_t sphi;                           \
    PPRZ_ITRIG_SIN(sphi, (_e).phi);                 \
    int32_t cphi;                           \
    PPRZ_ITRIG_COS(cphi, (_e).phi);                 \
    int32_t stheta;                         \
    PPRZ_ITRIG_SIN(stheta, (_e).theta);                 \
    int32_t ctheta;                         \
    PPRZ_ITRIG_COS(ctheta, (_e).theta);                 \
    int32_t spsi;                           \
    PPRZ_ITRIG_SIN(spsi, (_e).psi);                 \
    int32_t cpsi;                           \
    PPRZ_ITRIG_COS(cpsi, (_e).psi);                 \
                                        \
                                        \
    int32_t stheta_spsi = INT_MULT_RSHIFT(stheta, spsi,   INT32_TRIG_FRAC); \
    int32_t stheta_cpsi = INT_MULT_RSHIFT(stheta, cpsi,   INT32_TRIG_FRAC); \
    int32_t ctheta_spsi = INT_MULT_RSHIFT(ctheta, spsi,   INT32_TRIG_FRAC); \
    int32_t ctheta_cpsi = INT_MULT_RSHIFT(ctheta, cpsi,   INT32_TRIG_FRAC); \
    int32_t cphi_stheta = INT_MULT_RSHIFT(cphi,   stheta, INT32_TRIG_FRAC); \
    int32_t cphi_ctheta = INT_MULT_RSHIFT(cphi,   ctheta, INT32_TRIG_FRAC); \
    int32_t cphi_spsi   = INT_MULT_RSHIFT(cphi,   spsi,   INT32_TRIG_FRAC); \
    int32_t cphi_cpsi   = INT_MULT_RSHIFT(cphi,   cpsi,   INT32_TRIG_FRAC); \
    int32_t sphi_stheta = INT_MULT_RSHIFT(sphi,   stheta, INT32_TRIG_FRAC); \
    int32_t sphi_ctheta = INT_MULT_RSHIFT(sphi,   ctheta, INT32_TRIG_FRAC); \
                                        \
    int32_t sphi_stheta_spsi = INT_MULT_RSHIFT(sphi_stheta, spsi, INT32_TRIG_FRAC); \
    int32_t sphi_stheta_cpsi = INT_MULT_RSHIFT(sphi_stheta, cpsi, INT32_TRIG_FRAC); \
    int32_t sphi_ctheta_spsi = INT_MULT_RSHIFT(sphi_ctheta, spsi, INT32_TRIG_FRAC); \
    int32_t sphi_ctheta_cpsi = INT_MULT_RSHIFT(sphi_ctheta, cpsi, INT32_TRIG_FRAC); \
                                        \
    RMAT_ELMT(_rm, 0, 0) =  ctheta_cpsi - sphi_stheta_spsi;     \
    RMAT_ELMT(_rm, 0, 1) =  ctheta_spsi + sphi_stheta_cpsi;     \
    RMAT_ELMT(_rm, 0, 2) = -cphi_stheta;                \
    RMAT_ELMT(_rm, 1, 0) = -cphi_spsi;                  \
    RMAT_ELMT(_rm, 1, 1) =  cphi_cpsi;                  \
    RMAT_ELMT(_rm, 1, 2) =  sphi;                   \
    RMAT_ELMT(_rm, 2, 0) =  stheta_cpsi + sphi_ctheta_spsi;     \
    RMAT_ELMT(_rm, 2, 1) =  stheta_spsi - sphi_ctheta_cpsi;     \
    RMAT_ELMT(_rm, 2, 2) =  cphi_ctheta;                \
                                    \
  }


/*
 * Quaternions
 */

#define INT32_QUAT_ZERO(_q) {                       \
    (_q).qi = QUAT1_BFP_OF_REAL(1);                 \
    (_q).qx = 0;                            \
    (_q).qy = 0;                            \
    (_q).qz = 0;                            \
  }

#define INT32_QUAT_INVERT(_qo, _qi) QUAT_INVERT(_qo, _qi)

#define INT32_QUAT_NORM(n, q) {                                 \
    int32_t n2 = (q).qi*(q).qi + (q).qx*(q).qx + (q).qy*(q).qy + (q).qz*(q).qz; \
    INT32_SQRT(n, n2);                          \
  }

#define INT32_QUAT_WRAP_SHORTEST(q) {                   \
    if ((q).qi < 0)                         \
      QUAT_EXPLEMENTARY(q,q);                       \
  }

#define INT32_QUAT_NORMALIZE(q) {                   \
    int32_t n;                                      \
    INT32_QUAT_NORM(n, q);                          \
    if (n > 0) {                                    \
      (q).qi = (q).qi * QUAT1_BFP_OF_REAL(1) / n;   \
      (q).qx = (q).qx * QUAT1_BFP_OF_REAL(1) / n;   \
      (q).qy = (q).qy * QUAT1_BFP_OF_REAL(1) / n;   \
      (q).qz = (q).qz * QUAT1_BFP_OF_REAL(1) / n;   \
    }                                               \
  }

/* _a2c = _a2b comp _b2c , aka  _a2c = _b2c * _a2b */
#define INT32_QUAT_COMP(_a2c, _a2b, _b2c) {             \
    (_a2c).qi = ((_a2b).qi*(_b2c).qi - (_a2b).qx*(_b2c).qx - (_a2b).qy*(_b2c).qy - (_a2b).qz*(_b2c).qz)>>INT32_QUAT_FRAC; \
    (_a2c).qx = ((_a2b).qi*(_b2c).qx + (_a2b).qx*(_b2c).qi + (_a2b).qy*(_b2c).qz - (_a2b).qz*(_b2c).qy)>>INT32_QUAT_FRAC; \
    (_a2c).qy = ((_a2b).qi*(_b2c).qy - (_a2b).qx*(_b2c).qz + (_a2b).qy*(_b2c).qi + (_a2b).qz*(_b2c).qx)>>INT32_QUAT_FRAC; \
    (_a2c).qz = ((_a2b).qi*(_b2c).qz + (_a2b).qx*(_b2c).qy - (_a2b).qy*(_b2c).qx + (_a2b).qz*(_b2c).qi)>>INT32_QUAT_FRAC; \
  }

/* _a2b = _a2b comp_inv _b2c , aka  _a2b = inv(_b2c) * _a2c */
#define INT32_QUAT_COMP_INV(_a2b, _a2c, _b2c) {             \
    (_a2b).qi = ( (_a2c).qi*(_b2c).qi + (_a2c).qx*(_b2c).qx + (_a2c).qy*(_b2c).qy + (_a2c).qz*(_b2c).qz)>>INT32_QUAT_FRAC; \
    (_a2b).qx = (-(_a2c).qi*(_b2c).qx + (_a2c).qx*(_b2c).qi - (_a2c).qy*(_b2c).qz + (_a2c).qz*(_b2c).qy)>>INT32_QUAT_FRAC; \
    (_a2b).qy = (-(_a2c).qi*(_b2c).qy + (_a2c).qx*(_b2c).qz + (_a2c).qy*(_b2c).qi - (_a2c).qz*(_b2c).qx)>>INT32_QUAT_FRAC; \
    (_a2b).qz = (-(_a2c).qi*(_b2c).qz - (_a2c).qx*(_b2c).qy + (_a2c).qy*(_b2c).qx + (_a2c).qz*(_b2c).qi)>>INT32_QUAT_FRAC; \
  }

/* _b2c = _a2b inv_comp _a2c , aka  _b2c = _a2c * inv(_a2b) */
#define INT32_QUAT_INV_COMP(_b2c, _a2b, _a2c) {             \
    (_b2c).qi = ((_a2b).qi*(_a2c).qi + (_a2b).qx*(_a2c).qx + (_a2b).qy*(_a2c).qy + (_a2b).qz*(_a2c).qz)>>INT32_QUAT_FRAC; \
    (_b2c).qx = ((_a2b).qi*(_a2c).qx - (_a2b).qx*(_a2c).qi - (_a2b).qy*(_a2c).qz + (_a2b).qz*(_a2c).qy)>>INT32_QUAT_FRAC; \
    (_b2c).qy = ((_a2b).qi*(_a2c).qy + (_a2b).qx*(_a2c).qz - (_a2b).qy*(_a2c).qi - (_a2b).qz*(_a2c).qx)>>INT32_QUAT_FRAC; \
    (_b2c).qz = ((_a2b).qi*(_a2c).qz - (_a2b).qx*(_a2c).qy + (_a2b).qy*(_a2c).qx - (_a2b).qz*(_a2c).qi)>>INT32_QUAT_FRAC; \
  }

/* _b2c = _a2b inv_comp _a2c , aka  _b2c = inv(_a2b) * _a2c */
#define INT32_QUAT_INV_COMP_NORM_SHORTEST(_b2c, _a2b, _a2c) {   \
    INT32_QUAT_INV_COMP(_b2c, _a2b, _a2c);                      \
    INT32_QUAT_WRAP_SHORTEST(_b2c);                             \
    INT32_QUAT_NORMALIZE(_b2c);                                 \
  }

/* _a2c = _a2b comp _b2c , aka  _a2c = _a2b * _b2c */
#define INT32_QUAT_COMP_NORM_SHORTEST(_a2c, _a2b, _b2c) {       \
    INT32_QUAT_COMP(_a2c, _a2b, _b2c);                  \
    INT32_QUAT_WRAP_SHORTEST(_a2c);                 \
    INT32_QUAT_NORMALIZE(_a2c);                     \
  }


/* _qd = -0.5*omega(_r) * _q  */
// mult with 0.5 is done by shifting one more bit to the right
#define INT32_QUAT_DERIVATIVE(_qd, _r, _q) {                \
    (_qd).qi = (-( (_r).p*(_q).qx + (_r).q*(_q).qy + (_r).r*(_q).qz))>>(INT32_RATE_FRAC+1); \
    (_qd).qx = (-(-(_r).p*(_q).qi - (_r).r*(_q).qy + (_r).q*(_q).qz))>>(INT32_RATE_FRAC+1); \
    (_qd).qy = (-(-(_r).q*(_q).qi + (_r).r*(_q).qx - (_r).p*(_q).qz))>>(INT32_RATE_FRAC+1); \
    (_qd).qz = (-(-(_r).r*(_q).qi - (_r).q*(_q).qx + (_r).p*(_q).qy))>>(INT32_RATE_FRAC+1); \
  }

/** in place quaternion first order integration with constant rotational velocity. */
#define INT32_QUAT_INTEGRATE_FI(_q, _hr, _omega, _f) {              \
    _hr.qi += - ((int64_t) _omega.p)*_q.qx - ((int64_t) _omega.q)*_q.qy - ((int64_t) _omega.r)*_q.qz;    \
    _hr.qx +=   ((int64_t) _omega.p)*_q.qi + ((int64_t) _omega.r)*_q.qy - ((int64_t) _omega.q)*_q.qz;    \
    _hr.qy +=   ((int64_t) _omega.q)*_q.qi - ((int64_t) _omega.r)*_q.qx + ((int64_t) _omega.p)*_q.qz;    \
    _hr.qz +=   ((int64_t) _omega.r)*_q.qi + ((int64_t) _omega.q)*_q.qx - ((int64_t) _omega.p)*_q.qy;    \
                                                                    \
    lldiv_t _div = lldiv(_hr.qi, ((1<<INT32_RATE_FRAC)*_f*2));      \
    _q.qi+= (int32_t) _div.quot;                                    \
    _hr.qi = _div.rem;                                              \
                                                                    \
    _div = lldiv(_hr.qx, ((1<<INT32_RATE_FRAC)*_f*2));              \
    _q.qx+= (int32_t) _div.quot;                                    \
    _hr.qx = _div.rem;                                              \
                                                                    \
    _div = lldiv(_hr.qy, ((1<<INT32_RATE_FRAC)*_f*2));              \
    _q.qy+= (int32_t) _div.quot;                                    \
    _hr.qy = _div.rem;                                              \
                                                                    \
    _div = lldiv(_hr.qz, ((1<<INT32_RATE_FRAC)*_f*2));              \
    _q.qz+= (int32_t) _div.quot;                                    \
    _hr.qz = _div.rem;                                              \
                                                                    \
  }


#define INT32_QUAT_VMULT(v_out, q, v_in) {              \
    const int32_t _2qi2_m1 = (((q).qi*(q).qi)>>(INT32_QUAT_FRAC-1)) - QUAT1_BFP_OF_REAL( 1);    \
    const int32_t _2qx2    =  ((q).qx*(q).qx)>>(INT32_QUAT_FRAC-1);     \
    const int32_t _2qy2    =  ((q).qy*(q).qy)>>(INT32_QUAT_FRAC-1);     \
    const int32_t _2qz2    =  ((q).qz*(q).qz)>>(INT32_QUAT_FRAC-1);     \
    const int32_t _2qiqx   =  ((q).qi*(q).qx)>>(INT32_QUAT_FRAC-1);     \
    const int32_t _2qiqy   =  ((q).qi*(q).qy)>>(INT32_QUAT_FRAC-1);     \
    const int32_t _2qiqz   =  ((q).qi*(q).qz)>>(INT32_QUAT_FRAC-1);     \
    const int32_t m01 = (((q).qx*(q).qy)>>(INT32_QUAT_FRAC-1)) + _2qiqz;                \
    const int32_t m02 = (((q).qx*(q).qz)>>(INT32_QUAT_FRAC-1)) - _2qiqy;                \
    const int32_t m12 = (((q).qy*(q).qz)>>(INT32_QUAT_FRAC-1)) + _2qiqx;                \
    (v_out).x = (_2qi2_m1*(v_in).x + _2qx2 * (v_in).x + m01 * (v_in).y +  m02 * (v_in).z)>>INT32_QUAT_FRAC; \
    (v_out).y = (_2qi2_m1*(v_in).y + m01 * (v_in).x -2*_2qiqz*(v_in).x + _2qy2 * (v_in).y + m12 * (v_in).z)>>INT32_QUAT_FRAC; \
    (v_out).z = (_2qi2_m1*(v_in).z + m02 * (v_in).x +2*_2qiqy*(v_in).x+ m12 * (v_in).y -2*_2qiqx*(v_in).y+ _2qz2 * (v_in).z)>>INT32_QUAT_FRAC; \
  }



/*
 * http://www.mathworks.com/access/helpdesk_r13/help/toolbox/aeroblks/euleranglestoquaternions.html
 */
#define INT32_QUAT_OF_EULERS(_q, _e) {                  \
    const int32_t phi2   = (_e).phi   / 2;              \
    const int32_t theta2 = (_e).theta / 2;              \
    const int32_t psi2   = (_e).psi   / 2;              \
                                        \
    int32_t s_phi2;                         \
    PPRZ_ITRIG_SIN(s_phi2, phi2);                   \
    int32_t c_phi2;                         \
    PPRZ_ITRIG_COS(c_phi2, phi2);                   \
    int32_t s_theta2;                           \
    PPRZ_ITRIG_SIN(s_theta2, theta2);                   \
    int32_t c_theta2;                           \
    PPRZ_ITRIG_COS(c_theta2, theta2);                   \
    int32_t s_psi2;                         \
    PPRZ_ITRIG_SIN(s_psi2, psi2);                   \
    int32_t c_psi2;                         \
    PPRZ_ITRIG_COS(c_psi2, psi2);                   \
                                    \
    int32_t c_th_c_ps = INT_MULT_RSHIFT(c_theta2, c_psi2, INT32_TRIG_FRAC); \
    int32_t c_th_s_ps = INT_MULT_RSHIFT(c_theta2, s_psi2, INT32_TRIG_FRAC); \
    int32_t s_th_s_ps = INT_MULT_RSHIFT(s_theta2, s_psi2, INT32_TRIG_FRAC); \
    int32_t s_th_c_ps = INT_MULT_RSHIFT(s_theta2, c_psi2, INT32_TRIG_FRAC); \
                                    \
    (_q).qi = INT_MULT_RSHIFT( c_phi2, c_th_c_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC) + \
              INT_MULT_RSHIFT( s_phi2, s_th_s_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC);  \
    (_q).qx = INT_MULT_RSHIFT(-c_phi2, s_th_s_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC) + \
              INT_MULT_RSHIFT( s_phi2, c_th_c_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC);  \
    (_q).qy = INT_MULT_RSHIFT( c_phi2, s_th_c_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC) + \
              INT_MULT_RSHIFT( s_phi2, c_th_s_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC);  \
    (_q).qz = INT_MULT_RSHIFT( c_phi2, c_th_s_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC) + \
              INT_MULT_RSHIFT(-s_phi2, s_th_c_ps, INT32_TRIG_FRAC + INT32_TRIG_FRAC - INT32_QUAT_FRAC);  \
  }

#define INT32_QUAT_OF_AXIS_ANGLE(_q, _uv, _an) {        \
    int32_t san2;                                           \
    PPRZ_ITRIG_SIN(san2, (_an/2));                  \
    int32_t can2;                                           \
    PPRZ_ITRIG_COS(can2, (_an/2));                  \
    _q.qi = can2;                   \
    _q.qx = san2 * _uv.x;                   \
    _q.qy = san2 * _uv.y;                   \
    _q.qz = san2 * _uv.z;                   \
  }



#define INT32_QUAT_OF_RMAT(_q, _r) {                                    \
    const int32_t tr = RMAT_TRACE(_r);                                  \
    if (tr > 0) {                                                       \
      const int32_t two_qi_two = TRIG_BFP_OF_REAL(1.) + tr;             \
      int32_t two_qi;                                                   \
      INT32_SQRT(two_qi, (two_qi_two<<INT32_TRIG_FRAC));                \
      two_qi = two_qi << (INT32_QUAT_FRAC - INT32_TRIG_FRAC);           \
      (_q).qi = two_qi / 2;                                             \
      (_q).qx = ((RMAT_ELMT(_r, 1, 2) - RMAT_ELMT(_r, 2, 1)) <<         \
                 (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1)) \
        / two_qi;                                                       \
      (_q).qy = ((RMAT_ELMT(_r, 2, 0) - RMAT_ELMT(_r, 0, 2)) <<         \
                 (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1)) \
        / two_qi;                                                       \
      (_q).qz = ((RMAT_ELMT(_r, 0, 1) - RMAT_ELMT(_r, 1, 0)) <<         \
                 (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1)) \
        / two_qi;                                                       \
    }                                                                   \
    else {                                                              \
      if (RMAT_ELMT(_r, 0, 0) > RMAT_ELMT(_r, 1, 1) &&                  \
          RMAT_ELMT(_r, 0, 0) > RMAT_ELMT(_r, 2, 2)) {                  \
        const int32_t two_qx_two = RMAT_ELMT(_r, 0, 0) - RMAT_ELMT(_r, 1, 1) \
          - RMAT_ELMT(_r, 2, 2) + TRIG_BFP_OF_REAL(1.);                 \
        int32_t two_qx;                                                 \
        INT32_SQRT(two_qx, (two_qx_two<<INT32_TRIG_FRAC));              \
        two_qx = two_qx << (INT32_QUAT_FRAC - INT32_TRIG_FRAC);         \
        (_q).qi = ((RMAT_ELMT(_r, 1, 2) - RMAT_ELMT(_r, 2, 1)) <<       \
                   (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1)) \
          / two_qx;                                                     \
        (_q).qx = two_qx / 2;                                           \
        (_q).qy = ((RMAT_ELMT(_r, 0, 1) + RMAT_ELMT(_r, 1, 0)) <<       \
                   (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1)) \
          / two_qx;                                                     \
        (_q).qz = ((RMAT_ELMT(_r, 2, 0) + RMAT_ELMT(_r, 0, 2)) <<       \
                   (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1)) \
          / two_qx;                                                     \
      }                                                                 \
      else if (RMAT_ELMT(_r, 1, 1) > RMAT_ELMT(_r, 2, 2)) {             \
        const int32_t two_qy_two = RMAT_ELMT(_r, 1, 1) - RMAT_ELMT(_r, 0, 0) \
          - RMAT_ELMT(_r, 2, 2) + TRIG_BFP_OF_REAL(1.);                 \
        int32_t two_qy;                                                 \
        INT32_SQRT(two_qy, (two_qy_two<<INT32_TRIG_FRAC));              \
        two_qy = two_qy << (INT32_QUAT_FRAC - INT32_TRIG_FRAC);         \
        (_q).qi = ((RMAT_ELMT(_r, 2, 0) - RMAT_ELMT(_r, 0, 2)) <<       \
                   (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1)) \
          / two_qy;                                                     \
        (_q).qx = ((RMAT_ELMT(_r, 0, 1) + RMAT_ELMT(_r, 1, 0)) <<       \
                   (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1)) \
          / two_qy;                                                     \
        (_q).qy = two_qy / 2;                                           \
        (_q).qz = ((RMAT_ELMT(_r, 1, 2) + RMAT_ELMT(_r, 2, 1)) <<       \
                   (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1)) \
          / two_qy;                                                     \
      }                                                                 \
      else {                                                            \
        const int32_t two_qz_two = RMAT_ELMT(_r, 2, 2) - RMAT_ELMT(_r, 0, 0) \
          - RMAT_ELMT(_r, 1, 1) + TRIG_BFP_OF_REAL(1.);                 \
        int32_t two_qz;                                                 \
        INT32_SQRT(two_qz, (two_qz_two<<INT32_TRIG_FRAC));              \
        two_qz = two_qz << (INT32_QUAT_FRAC - INT32_TRIG_FRAC);         \
        (_q).qi = ((RMAT_ELMT(_r, 0, 1) - RMAT_ELMT(_r, 1, 0)) <<       \
                   (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1)) \
          / two_qz;                                                     \
        (_q).qx = ((RMAT_ELMT(_r, 2, 0) + RMAT_ELMT(_r, 0, 2)) <<       \
                   (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1)) \
          / two_qz;                                                     \
        (_q).qy = ((RMAT_ELMT(_r, 1, 2) + RMAT_ELMT(_r, 2, 1)) <<       \
                   (INT32_QUAT_FRAC - INT32_TRIG_FRAC + INT32_QUAT_FRAC - 1)) \
          / two_qz;                                                     \
        (_q).qz = two_qz / 2;                                           \
      }                                                                 \
    }                                                                   \
  }


/*
 * Euler angles
 */

#define INT_EULERS_ZERO(_e) EULERS_ASSIGN(_e, 0, 0, 0)


#define INT32_EULERS_OF_RMAT(_e, _rm) {                 \
                                        \
    const float dcm00 = TRIG_FLOAT_OF_BFP((_rm).m[0]);          \
    const float dcm01 = TRIG_FLOAT_OF_BFP((_rm).m[1]);          \
    const float dcm02 = TRIG_FLOAT_OF_BFP((_rm).m[2]);          \
    const float dcm12 = TRIG_FLOAT_OF_BFP((_rm).m[5]);          \
    const float dcm22 = TRIG_FLOAT_OF_BFP((_rm).m[8]);          \
    const float phi   = atan2f( dcm12, dcm22 );             \
    const float theta = -asinf( dcm02 );                \
    const float psi   = atan2f( dcm01, dcm00 );             \
    (_e).phi   = ANGLE_BFP_OF_REAL(phi);                \
    (_e).theta = ANGLE_BFP_OF_REAL(theta);              \
    (_e).psi   = ANGLE_BFP_OF_REAL(psi);                \
                                        \
  }


#define INT32_EULERS_OF_QUAT(_e, _q) {                  \
                                    \
    const int32_t qx2  = INT_MULT_RSHIFT((_q).qx,(_q).qx, INT32_QUAT_FRAC); \
    const int32_t qy2  = INT_MULT_RSHIFT((_q).qy,(_q).qy, INT32_QUAT_FRAC); \
    const int32_t qz2  = INT_MULT_RSHIFT((_q).qz,(_q).qz, INT32_QUAT_FRAC); \
    const int32_t qiqx = INT_MULT_RSHIFT((_q).qi,(_q).qx, INT32_QUAT_FRAC); \
    const int32_t qiqy = INT_MULT_RSHIFT((_q).qi,(_q).qy, INT32_QUAT_FRAC); \
    const int32_t qiqz = INT_MULT_RSHIFT((_q).qi,(_q).qz, INT32_QUAT_FRAC); \
    const int32_t qxqy = INT_MULT_RSHIFT((_q).qx,(_q).qy, INT32_QUAT_FRAC); \
    const int32_t qxqz = INT_MULT_RSHIFT((_q).qx,(_q).qz, INT32_QUAT_FRAC); \
    const int32_t qyqz = INT_MULT_RSHIFT((_q).qy,(_q).qz, INT32_QUAT_FRAC); \
    const int32_t one = TRIG_BFP_OF_REAL( 1);               \
    const int32_t two = TRIG_BFP_OF_REAL( 2);               \
                                    \
    /* dcm00 = 1.0 - 2.*(  qy2 +  qz2 ); */             \
    const int32_t idcm00 =  one - INT_MULT_RSHIFT( two, (qy2+qz2),  \
                           INT32_TRIG_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC); \
    /* dcm01 =       2.*( qxqy + qiqz ); */             \
    const int32_t idcm01 = INT_MULT_RSHIFT( two, (qxqy+qiqz),       \
                        INT32_TRIG_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC); \
    /* dcm02 =       2.*( qxqz - qiqy ); */             \
    const int32_t idcm02 = INT_MULT_RSHIFT( two, (qxqz-qiqy),       \
                        INT32_TRIG_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC); \
    /* dcm12 =       2.*( qyqz + qiqx ); */             \
    const int32_t idcm12 = INT_MULT_RSHIFT( two, (qyqz+qiqx),       \
                        INT32_TRIG_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC); \
    /* dcm22 = 1.0 - 2.*(  qx2 +  qy2 ); */             \
    const int32_t idcm22 = one - INT_MULT_RSHIFT( two, (qx2+qy2),   \
                          INT32_TRIG_FRAC+INT32_QUAT_FRAC-INT32_TRIG_FRAC); \
    const float dcm00 = (float)idcm00/(1<<INT32_TRIG_FRAC);     \
    const float dcm01 = (float)idcm01/(1<<INT32_TRIG_FRAC);     \
    const float dcm02 = (float)idcm02/(1<<INT32_TRIG_FRAC);     \
    const float dcm12 = (float)idcm12/(1<<INT32_TRIG_FRAC);     \
    const float dcm22 = (float)idcm22/(1<<INT32_TRIG_FRAC);     \
                                    \
    const float phi   = atan2f( dcm12, dcm22 );             \
    const float theta = -asinf( dcm02 );                \
    const float psi   = atan2f( dcm01, dcm00 );             \
    (_e).phi   = ANGLE_BFP_OF_REAL(phi);                \
    (_e).theta = ANGLE_BFP_OF_REAL(theta);              \
    (_e).psi   = ANGLE_BFP_OF_REAL(psi);                \
                                        \
  }

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

#define INT_RATES_ADD_SCALED_VECT(_ro, _v, _s) {    \
    _ro.p += _v.x * _s;                 \
    _ro.q += _v.y * _s;                 \
    _ro.r += _v.z * _s;                 \
  }

#define INT_RATES_SDIV(_ro, _s, _ri) {          \
    _ro.p = _ri.p / _s;                 \
    _ro.q = _ri.q / _s;                 \
    _ro.r = _ri.r / _s;                 \
  }

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



#define INT32_RATES_OF_EULERS_DOT_321(_r, _e, _ed) {            \
                                    \
    int32_t sphi;                           \
    PPRZ_ITRIG_SIN(sphi, (_e).phi);                 \
    int32_t cphi;                           \
    PPRZ_ITRIG_COS(cphi, (_e).phi);                 \
    int32_t stheta;                         \
    PPRZ_ITRIG_SIN(stheta, (_e).theta);                 \
    int32_t ctheta;                         \
    PPRZ_ITRIG_COS(ctheta, (_e).theta);                 \
                                    \
    int32_t cphi_ctheta = INT_MULT_RSHIFT(cphi,   ctheta, INT32_TRIG_FRAC); \
    int32_t sphi_ctheta = INT_MULT_RSHIFT(sphi,   ctheta, INT32_TRIG_FRAC); \
                                    \
    (_r).p = - INT_MULT_RSHIFT(stheta, (_ed).psi, INT32_TRIG_FRAC) + (_ed).phi; \
    (_r).q = INT_MULT_RSHIFT(sphi_ctheta, (_ed).psi, INT32_TRIG_FRAC) + INT_MULT_RSHIFT(cphi, (_ed).theta, INT32_TRIG_FRAC); \
    (_r).r = INT_MULT_RSHIFT(cphi_ctheta, (_ed).psi, INT32_TRIG_FRAC) - INT_MULT_RSHIFT(sphi, (_ed).theta, INT32_TRIG_FRAC); \
                                    \
  }

#define INT32_RATES_OF_EULERS_DOT(_r, _e, _ed) INT32_RATES_OF_EULERS_DOT_321(_r, _e, _ed)

#define INT32_EULERS_DOT_321_OF_RATES(_ed, _e, _r) {            \
                                        \
    int32_t sphi;                           \
    PPRZ_ITRIG_SIN(sphi, (_e).phi);                 \
    int32_t cphi;                           \
    PPRZ_ITRIG_COS(cphi, (_e).phi);                 \
    int32_t stheta;                         \
    PPRZ_ITRIG_SIN(stheta, (_e).theta);                 \
    int64_t ctheta;                         \
    PPRZ_ITRIG_COS(ctheta, (_e).theta);                 \
                                        \
    if (ctheta != 0) {                          \
      int64_t cphi_stheta = INT_MULT_RSHIFT(cphi, stheta, INT32_TRIG_FRAC); \
      int64_t sphi_stheta = INT_MULT_RSHIFT(sphi, stheta, INT32_TRIG_FRAC); \
                                        \
      (_ed).phi = (_r).p + (int32_t)((sphi_stheta * (int64_t)(_r).q) / ctheta) + (int32_t)((cphi_stheta * (int64_t)(_r).r) / ctheta);  \
      (_ed).theta = INT_MULT_RSHIFT(cphi, (_r).q, INT32_TRIG_FRAC) - INT_MULT_RSHIFT(sphi, (_r).r, INT32_TRIG_FRAC);  \
      (_ed).psi = (int32_t)(((int64_t)sphi * (int64_t)(_r).q) / ctheta) + (int32_t)(((int64_t)cphi * (int64_t)(_r).r) / ctheta);  \
    }                                   \
    /* FIXME: What do you wanna do when you hit the singularity ? */    \
    /* probably not return an uninitialized variable, or ?        */    \
    else {                              \
      INT_EULERS_ZERO(_ed);                     \
    }                                   \
  }

#define INT32_EULERS_DOT_OF_RATES(_ed, _e, _r) INT32_EULERS_DOT_321_OF_RATES(_ed, _e, _r)

/*
 *
 */
#define INT32_SQRT_MAX_ITER 40
#define INT32_SQRT(_out,_in) {                                  \
    if ((_in) == 0)                                             \
      (_out) = 0;                                               \
    else {                                                      \
      uint32_t s1, s2;                                          \
      uint8_t iter = 0;                                         \
      s2 = _in;                                                 \
      do {                                                      \
        s1 = s2;                                                \
        s2 = (_in) / s1;                                        \
        s2 += s1;                                               \
        s2 /= 2;                                                \
        iter++;                                                 \
      }                                                         \
      while( ( (s1-s2) > 1) && (iter < INT32_SQRT_MAX_ITER));   \
      (_out) = s2;                                              \
    }                                                           \
  }


/* http://jet.ro/files/The_neglected_art_of_Fixed_Point_arithmetic_20060913.pdf */
/* http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm */

#define R_FRAC 14

#define INT32_ATAN2(_a, _y, _x) {           \
    const int32_t c1 = INT32_ANGLE_PI_4;        \
    const int32_t c2 = 3 * INT32_ANGLE_PI_4;        \
    const int32_t abs_y = abs(_y) + 1;          \
    int32_t r;                      \
    if ( (_x) >= 0) {                               \
      r = (((_x)-abs_y)<<R_FRAC) / ((_x)+abs_y);    \
      (_a) = c1 - ((c1 * r)>>R_FRAC);               \
    }                           \
    else {                      \
      r = (((_x)+abs_y)<<R_FRAC) / (abs_y-(_x));    \
      (_a) = c2 - ((c1 * r)>>R_FRAC);           \
    }                           \
    if ((_y)<0)                     \
      (_a) = -(_a);                 \
  }


#define INT32_ATAN2_2(_a, _y, _x) {                 \
    const int32_t c1 = INT32_ANGLE_PI_4;                \
    const int32_t c2 = 3 * INT32_ANGLE_PI_4;                \
    const int32_t abs_y = abs(_y) + 1;                  \
    int32_t r;                              \
    if ( (_x) >= 0) {                           \
      r = (((_x)-abs_y)<<R_FRAC) / ((_x)+abs_y);                \
      int32_t r2 = (r * r)>>R_FRAC;                 \
      int32_t tmp1 = ((r2 * (int32_t)ANGLE_BFP_OF_REAL(0.1963))>>INT32_ANGLE_FRAC) - ANGLE_BFP_OF_REAL(0.9817); \
      (_a) = ((tmp1 * r)>>R_FRAC) + c1;                 \
    }                                   \
    else {                              \
      r = (((_x)+abs_y)<<R_FRAC) / (abs_y-(_x));                \
      (_a) = c2 - ((c1 * r)>>R_FRAC);                   \
    }                                   \
    if ((_y)<0)                             \
      (_a) = -(_a);                             \
  }



#endif /* PPRZ_ALGEBRA_INT_H */
