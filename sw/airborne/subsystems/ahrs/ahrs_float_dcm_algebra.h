/*
 * Released under Creative Commons License
 *
 * 2010 The Paparazzi Team
 *
 *
 * Based on Code by Jordi Munoz and William Premerlani, Supported by Chris Anderson (Wired) and Nathan Sindle (SparkFun).
 * Version 1.0 for flat board updated by Doug Weibel and Jose Julio
 *
 */

/**
 * @file subsystems/ahrs/ahrs_float_dcm_algebra.h
 *
 * Algebra helper functions for DCM.
 *
 * @todo get rid of this and use pprz math lib.
 */

#ifndef AHRS_FLOAT_DCM_ALGEBRA_H
#define AHRS_FLOAT_DCM_ALGEBRA_H


static inline float Vector_Dot_Product(float vector1[3], float vector2[3])
{
  return vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2];
}

static inline void Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3])
{
  vectorOut[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  vectorOut[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  vectorOut[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

static inline void Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2)
{
  vectorOut[0] = vectorIn[0] * scale2;
  vectorOut[1] = vectorIn[1] * scale2;
  vectorOut[2] = vectorIn[2] * scale2;
}

static inline void Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3])
{
  vectorOut[0] = vectorIn1[0] + vectorIn2[0];
  vectorOut[1] = vectorIn1[1] + vectorIn2[1];
  vectorOut[2] = vectorIn1[2] + vectorIn2[2];
}

/*
  #define Matrix_Multiply( _m_a2b, _m_b2c, _m_a2c) {      \
  _m_a2c[0] = (_m_b2c[0]*_m_a2b[0] + _m_b2c[1]*_m_a2b[3] + _m_b2c[2]*_m_a2b[6]); \
  _m_a2c[1] = (_m_b2c[0]*_m_a2b[1] + _m_b2c[1]*_m_a2b[4] + _m_b2c[2]*_m_a2b[7]); \
  _m_a2c[2] = (_m_b2c[0]*_m_a2b[2] + _m_b2c[1]*_m_a2b[5] + _m_b2c[2]*_m_a2b[8]); \
  _m_a2c[3] = (_m_b2c[3]*_m_a2b[0] + _m_b2c[4]*_m_a2b[3] + _m_b2c[5]*_m_a2b[6]); \
  _m_a2c[4] = (_m_b2c[3]*_m_a2b[1] + _m_b2c[4]*_m_a2b[4] + _m_b2c[5]*_m_a2b[7]); \
  _m_a2c[5] = (_m_b2c[3]*_m_a2b[2] + _m_b2c[4]*_m_a2b[5] + _m_b2c[5]*_m_a2b[8]); \
  _m_a2c[6] = (_m_b2c[6]*_m_a2b[0] + _m_b2c[7]*_m_a2b[3] + _m_b2c[8]*_m_a2b[6]); \
  _m_a2c[7] = (_m_b2c[6]*_m_a2b[1] + _m_b2c[7]*_m_a2b[4] + _m_b2c[8]*_m_a2b[7]); \
  _m_a2c[8] = (_m_b2c[6]*_m_a2b[2] + _m_b2c[7]*_m_a2b[5] + _m_b2c[8]*_m_a2b[8]); \
  }
*/

static inline void Matrix_Multiply(float a[3][3], float b[3][3], float mat[3][3])
{
  float op[3];
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 3; y++) {
      for (int w = 0; w < 3; w++) {
        op[w] = a[x][w] * b[w][y];
      }
      mat[x][y] = op[0] + op[1] + op[2];
    }
  }
}

#endif // AHRS_FLOAT_DCM_ALGEBRA_H
