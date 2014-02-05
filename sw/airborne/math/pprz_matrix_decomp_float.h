/*
 * Copyright (C) 2014 Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "std.h"
#include "math/pprz_algebra_float.h"
#include <math.h>
#include <string.h>

/** Cholesky decomposition
 *
 * http://rosettacode.org/wiki/Cholesky_decomposition#C
 *
 * @param out pointer to the output array (square n x n matrix, seen as a single dimension array of size n*n)
 * @param in pointer to the input array (square n x n matrix, seen as a single dimension array of size n*n)
 * @param n dimension of the matrix
 */
static inline void pprz_cholesky_float(float * out, const float * in, const int n) {
  int i,j,k;
  float o[n*n];

  for (i = 0; i < n; i++) {
    for (j = 0; j < (i+1); j++) {
      float s = 0;
      for (k = 0; k < j; k++)
        s += o[i * n + k] * o[j * n + k];
      o[i * n + j] = (i == j) ?
        sqrtf(in[i * n + i] - s) :
        (1.0 / o[j * n + j] * (in[i * n + j] - s));
    }
  }
  memcpy(out, o, n*n*sizeof(float));
}

/** QR decomposition
 *
 * using Householder method
 *
 * http://rosettacode.org/wiki/QR_decomposition#C
 *
 * @param Q square orthogonal matrix Q (m x m matrix, seen as a single dimension array of size m*m)
 * @param R upper triangular matrix R (m x n matrix, seen as a single dimension array of size m*n)
 * @param in pointer to the input array (square m x n matrix, seen as a single dimension array of size m*n)
 * @param m number of rows of the input matrix
 * @param n number of column of the input matrix
 */
static inline void pprz_qr_float(float * Q, float * R, const float * in, const int m, const int n)
{
  int i, k;
  float q[m][m*m];
  float z[m*n], z1[m*n], z2[m*m];
  float_mat_copy(z, in, m, n);
  for (k = 0; k < n && k < m - 1; k++) {
    float e[m], x[m], a, b;
    float_mat_minor(z1, z, m, n, k);
    float_mat_col(x, z1, m, n, k);
    a = float_vect_norm(x, m);
    if (in[k*n + k] > 0) a = -a;
    for (i = 0; i < m; i++) {
      e[i] = (i == k) ? 1 : 0;
      e[i] = x[i] + a * e[i];
    }
    b = float_vect_norm(e, m);
    float_vect_sdiv(e, e, b, m);
    float_mat_vmul(q[k], e, m);
    float_mat_mul(z, q[k], z1, m, m, n);
  }
  float_mat_copy(Q, q[0], m, m);
  for (i = 1; i < n && i < m - 1; i++) {
    float_mat_mul(z2, q[i], Q, m, m, m);
    float_mat_copy(Q, z2, m, m);
  }
  float_mat_mul(R, Q, in, m, m, n);
  float_mat_transpose(Q, m);
}

