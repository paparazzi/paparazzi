/*
 * Copyright (C) 2009-2014 The Paparazzi Team
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
 * @file pprz_simple_matrix.h
 * @brief Simple matrix helper macros.
 *
 */

#ifndef PPRZ_SIMPLE_MATRIX_H
#define PPRZ_SIMPLE_MATRIX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <float.h>  /* for FLT_EPSILON */
#include <math.h>
#ifdef HAVE_STDIO
#include <stdio.h>  /* for printf'ing warnings */
#define warn_message printf
#else
#define warn_message(...) do { } while(0)
#endif

//
// C = A*B   A:(i,k) B:(k,j) C:(i,j)
//
#define MAT_MUL(_i, _k, _j, C, A, B) {          \
    int l,c,m;                                  \
    for (l=0; l<_i; l++)                        \
      for (c=0; c<_j; c++) {                    \
        C[l][c] = 0.;                           \
        for (m=0; m<_k; m++)                    \
          C[l][c] += A[l][m]*B[m][c];           \
      }                                         \
  }

//
// C = A*B'   A:(i,k) B:(j,k) C:(i,j)
//
#define MAT_MUL_T(_i, _k, _j, C, A, B) {        \
    int l,c,m;                                  \
    for (l=0; l<_i; l++)                        \
      for (c=0; c<_j; c++) {                    \
        C[l][c] = 0.;                           \
        for (m=0; m<_k; m++)                    \
          C[l][c] += A[l][m]*B[c][m];           \
      }                                         \
  }


//
// C = A-B
//
#define MAT_SUB(_i, _j, C, A, B) {              \
    int l,c;                                    \
    for (l=0; l<_i; l++)                        \
      for (c=0; c<_j; c++)                      \
        C[l][c] = A[l][c] - B[l][c];            \
  }




//
// invS = 1/det(S) com(S)'
//
#define MAT_INV33(_invS, _S) {                                          \
    const float m00 = _S[1][1]*_S[2][2] - _S[1][2]*_S[2][1];            \
    const float m10 = _S[0][1]*_S[2][2] - _S[0][2]*_S[2][1];            \
    const float m20 = _S[0][1]*_S[1][2] - _S[0][2]*_S[1][1];            \
    const float m01 = _S[1][0]*_S[2][2] - _S[1][2]*_S[2][0];            \
    const float m11 = _S[0][0]*_S[2][2] - _S[0][2]*_S[2][0];            \
    const float m21 = _S[0][0]*_S[1][2] - _S[0][2]*_S[1][0];            \
    const float m02 = _S[1][0]*_S[2][1] - _S[1][1]*_S[2][0];            \
    const float m12 = _S[0][0]*_S[2][1] - _S[0][1]*_S[2][0];            \
    const float m22 = _S[0][0]*_S[1][1] - _S[0][1]*_S[1][0];            \
    float det = _S[0][0]*m00 - _S[1][0]*m10 + _S[2][0]*m20;             \
    if (fabs(det) < FLT_EPSILON) {                                      \
      /* If the determinant is too small then set it to epsilon preserving sign. */ \
      warn_message("warning: %s:%d MAT_INV33 trying to invert non-invertable matrix '%s' and put result in '%s'.\n", __FILE__, __LINE__, #_S, #_invS); \
      det = copysignf(FLT_EPSILON, det);                                \
    }                                                                   \
    _invS[0][0] =  m00 / det;                                           \
    _invS[1][0] = -m01 / det;                                           \
    _invS[2][0] =  m02 / det;                                           \
    _invS[0][1] = -m10 / det;                                           \
    _invS[1][1] =  m11 / det;                                           \
    _invS[2][1] = -m12 / det;                                           \
    _invS[0][2] =  m20 / det;                                           \
    _invS[1][2] = -m21 / det;                                           \
    _invS[2][2] =  m22 / det;                                           \
  }

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_SIMPLE_MATRIX_H */
