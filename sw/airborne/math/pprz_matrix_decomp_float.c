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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file pprz_matrix_decomp_float.c
 * @brief Matrix decompositions in floating point.
 *
 */

#include "math/pprz_matrix_decomp_float.h"
#include "math/pprz_algebra_float.h"
#include <math.h>
#include <string.h>

/** Cholesky decomposition
 *
 * http://rosettacode.org/wiki/Cholesky_decomposition#C
 *
 * @param out pointer to the output array [n x n]
 * @param in pointer to the input array [n x n]
 * @param n dimension of the matrix
 */
void pprz_cholesky_float(float **out, float **in, int n)
{
  int i, j, k;
  float _o[n][n];
  MAKE_MATRIX_PTR(o, _o, n);

  float_mat_zero(o, n, n);
  for (i = 0; i < n; i++) {
    for (j = 0; j < (i + 1); j++) {
      float s = 0;
      for (k = 0; k < j; k++) {
        s += o[i][k] * o[j][k];
      }
      o[i][j] = (i == j) ?
                sqrtf(in[i][i] - s) :
                (1.0 / o[j][j] * (in[i][j] - s));
    }
  }
  float_mat_copy(out, o, n, n);
}

/** QR decomposition
 *
 * using Householder method
 *
 * http://rosettacode.org/wiki/QR_decomposition#C
 *
 * @param Q square orthogonal matrix Q [m x m]
 * @param R upper triangular matrix R [m x n]
 * @param in pointer to the input array [m x n]
 * @param m number of rows of the input matrix
 * @param n number of column of the input matrix
 */
void pprz_qr_float(float **Q, float **R, float **in, int m, int n)
{
  int i, k;
  float _q[m][m][m];
  float *q[m][m];
  float _z[m][n], _z1[m][n], _z2[m][m];
  MAKE_MATRIX_PTR(z, _z, m);
  MAKE_MATRIX_PTR(z1, _z1, m);
  MAKE_MATRIX_PTR(z2, _z2, m);
  for (i = 0;  i < m; i++) for (k = 0; k < m; k++) { q[i][k] = &_q[i][k][0]; }
  float_mat_copy(z, in, m, n);
  for (k = 0; k < n && k < m - 1; k++) {
    float e[m], x[m], a, b;
    float_mat_minor(z1, z, m, n, k);
    float_mat_col(x, z1, m, k);
    a = float_vect_norm(x, m);
    if (in[k][k] > 0) { a = -a; }
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

/** Some SVD decomposition utility macros and functions
*/

//Computes sqrt(a*a + b*b) without destructive underflow or overflow
static inline float pythag(float a, float b)
{
  float absa, absb;
  absa = fabsf(a);
  absb = fabsf(b);
  if (absa > absb) {
    return (absa * sqrtf(1.0 + (absb / absa) * (absb / absa)));
  } else if (absb == 0.0) {
    return 0.0;
  } else {
    return (absb * sqrtf(1.0 + (absa / absb) * (absa / absb)));
  }
}


/** SVD decomposition
 *
 * --------------------------------------------------------------------- *
 * Reference:  "Numerical Recipes By W.H. Press, B. P. Flannery,         *
 *              S.A. Teukolsky and W.T. Vetterling, Cambridge            *
 *              University Press, 1986" [BIBLI 08].                      *
 * --------------------------------------------------------------------- *
 *
 * Given a matrix a(m,n), this routine computes its singular value decomposition,
 * A = U · W · Vt. The matrix U replaces a on output. The diagonal matrix of singular
 * values W is output as a vector w(n). The matrix V (not the transpose Vt) is output
 * as v(n,n).
 *
 * @param a input matrix [m x n] and output matrix U [m x n]
 * @param w output diagonal vector of matrix W [n]
 * @param v output square matrix V [n x n]
 * @param m number of rows of input the matrix
 * @param n number of columns of the input matrix
 * @return 0 (false) if convergence failed, 1 (true) if decomposition succed
 */
int pprz_svd_float(float **a, float *w, float **v, int m, int n)
{
  /* Householder reduction to bidiagonal form. */
  int flag, i, its, j, jj, k, l, NM;
  float C, F, H, S, X, Y, Z, tmp;
  float G = 0.0;
  float Scale = 0.0;
  float ANorm = 0.0;
  float rv1[n];

  for (i = 0; i < n; ++i) {
    l = i + 1;
    rv1[i] = Scale * G;
    G = 0.0;
    S = 0.0;
    Scale = 0.0;
    if (i < m) {
      for (k = i; k < m; ++k) {
        Scale = Scale + fabsf(a[k][i]);
      }
      if (Scale != 0.0) {
        for (k = i; k < m; ++k) {
          a[k][i] = a[k][i] / Scale;
          S = S + a[k][i] * a[k][i];
        }
        F = a[i][i];
        G = sqrtf(S);
        if (F > 0.0) {
          G = -G;
        }
        H = F * G - S;
        a[i][i] = F - G;
        if (i != (n - 1)) {
          for (j = l; j < n; ++j) {
            S = 0.0;
            for (k = i; k < m; ++k) {
              S = S + a[k][i] * a[k][j];
            }
            F = S / H;
            for (k = i; k < m; ++k) {
              a[k][j] = a[k][j] + F * a[k][i];
            }
          }
        }
        for (k = i; k < m; ++k) {
          a[k][i] = Scale * a[k][i];
        }
      }
    }

    w[i] = Scale * G;
    G = 0.0;
    S = 0.0;
    Scale = 0.0;
    if ((i < m) && (i != (n - 1))) {
      for (k = l; k < n; ++k) {
        Scale = Scale + fabsf(a[i][k]);
      }
      if (Scale != 0.0) {
        for (k = l; k < n; ++k) {
          a[i][k] = a[i][k] / Scale;
          S = S + a[i][k] * a[i][k];
        }
        F = a[i][l];
        G = sqrtf(S);
        if (F > 0.0) {
          G = -G;
        }
        H = F * G - S;
        a[i][l] = F - G;
        for (k = l; k < n; ++k) {
          rv1[k] = a[i][k] / H;
        }
        if (i != (m - 1)) {
          for (j = l; j < m; ++j) {
            S = 0.0;
            for (k = l; k < n; ++k) {
              S = S + a[j][k] * a[i][k];
            }
            for (k = l; k < n; ++k) {
              a[j][k] = a[j][k] + S * rv1[k];
            }
          }
        }
        for (k = l; k < n; ++k) {
          a[i][k] = Scale * a[i][k];
        }
      }
    }
    tmp = fabsf(w[i]) + fabsf(rv1[i]);
    if (tmp > ANorm) {
      ANorm = tmp;
    }
  }

  /* Accumulation of right-hand transformations. */
  for (i = n - 1; i >= 0; --i) {
    if (i < (n - 1)) {
      if (G != 0.0) {
        for (j = l; j < n; ++j) {
          v[j][i] = (a[i][j] / a[i][l]) / G;
        }
        for (j = l; j < n; ++j) {
          S = 0.0;
          for (k = l; k < n; ++k) {
            S = S + a[i][k] * v[k][j];
          }
          for (k = l; k < n; ++k) {
            v[k][j] = v[k][j] + S * v[k][i];
          }
        }
      }
      for (j = l; j < n; ++j) {
        v[i][j] = 0.0;
        v[j][i] = 0.0;
      }
    }
    v[i][i] = 1.0;
    G = rv1[i];
    l = i;
  }

  /* Accumulation of left-hand transformations. */
  for (i = n - 1; i >= 0; --i) {
    l = i + 1;
    G = w[i];
    if (i < (n - 1)) {
      for (j = l; j < n; ++j) {
        a[i][j] = 0.0;
      }
    }
    if (G != 0.0) {
      G = 1.0 / G;
      if (i != (n - 1)) {
        for (j = l; j < n; ++j) {
          S = 0.0;
          for (k = l; k < m; ++k) {
            S = S + a[k][i] * a[k][j];
          }
          F = (S / a[i][i]) * G;
          for (k = i; k < m; ++k) {
            a[k][j] = a[k][j] + F * a[k][i];
          }
        }
      }
      for (j = i; j < m; ++j) {
        a[j][i] = a[j][i] * G;
      }
    } else {
      for (j = i; j < m; ++j) {
        a[j][i] = 0.0;
      }
    }
    a[i][i] = a[i][i] + 1.0;
  }

  /* Diagonalization of the bidiagonal form.
     Loop over singular values. */
  for (k = (n - 1); k >= 0; --k) {
    /* Loop over allowed iterations. */
    for (its = 1; its <= 30; ++its) {
      /* Test for splitting.
         Note that rv1[0] is always zero. */
      flag = true;
      for (l = k; l >= 0; --l) {
        NM = l - 1;
        if ((fabsf(rv1[l]) + ANorm) == ANorm) {
          flag = false;
          break;
        } else if ((fabsf(w[NM]) + ANorm) == ANorm) {
          break;
        }
      }

      /* Cancellation of rv1[l], if l > 0; */
      if (flag) {
        C = 0.0;
        S = 1.0;
        for (i = l; i <= k; ++i) {
          F = S * rv1[i];
          if ((fabsf(F) + ANorm) != ANorm) {
            G = w[i];
            //H = sqrtf( F * F + G * G );
            H = pythag(F, G);
            w[i] = H;
            H = 1.0 / H;
            C = (G * H);
            S = -(F * H);
            for (j = 0; j < m; ++j) {
              Y = a[j][NM];
              Z = a[j][i];
              a[j][NM] = (Y * C) + (Z * S);
              a[j][i] = -(Y * S) + (Z * C);
            }
          }
        }
      }
      Z = w[k];
      /* Convergence. */
      if (l == k) {
        /* Singular value is made nonnegative. */
        if (Z < 0.0) {
          w[k] = -Z;
          for (j = 0; j < n; ++j) {
            v[j][k] = -v[j][k];
          }
        }
        break;
      }

      if (its >= 30) {
        // No convergence in 30 iterations
        return 0;
      }

      X = w[l];
      NM = k - 1;
      Y = w[NM];
      G = rv1[NM];
      H = rv1[k];
      F = ((Y - Z) * (Y + Z) + (G - H) * (G + H)) / (2.0 * H * Y);
      //G = sqrtf( F * F + 1.0 );
      G = pythag(F, 1.0);
      tmp = G;
      if (F < 0.0) {
        tmp = -tmp;
      }
      F = ((X - Z) * (X + Z) + H * ((Y / (F + tmp)) - H)) / X;

      /* Next QR transformation. */
      C = 1.0;
      S = 1.0;
      for (j = l; j <= NM; ++j) {
        i = j + 1;
        G = rv1[i];
        Y = w[i];
        H = S * G;
        G = C * G;
        //Z = sqrtf( F * F + H * H );
        Z = pythag(F, H);
        rv1[j] = Z;
        C = F / Z;
        S = H / Z;
        F = (X * C) + (G * S);
        G = -(X * S) + (G * C);
        H = Y * S;
        Y = Y * C;
        for (jj = 0; jj < n; ++jj) {
          X = v[jj][j];
          Z = v[jj][i];
          v[jj][j] = (X * C) + (Z * S);
          v[jj][i] = -(X * S) + (Z * C);
        }
        //Z = sqrtf( F * F + H * H );
        Z = pythag(F, H);
        w[j] = Z;

        /* Rotation can be arbitrary if Z = 0. */
        if (Z != 0.0) {
          Z = 1.0 / Z;
          C = F * Z;
          S = H * Z;
        }
        F = (C * G) + (S * Y);
        X = -(S * G) + (C * Y);
        for (jj = 0; jj < m; ++jj) {
          Y = a[jj][j];
          Z = a[jj][i];
          a[jj][j] = (Y * C) + (Z * S);
          a[jj][i] = -(Y * S) + (Z * C);
        }
      }
      rv1[l] = 0.0;
      rv1[k] = F;
      w[k] = X;
    }
  }

  return 1;
}

/** SVD based linear solver
 *
 * Solves A · X = B for a vector X,
 * where A is specified by the arrays u, w, v as returned by pprz_svd_float.
 * m and n are the dimensions of a.
 * b(m) is the input right-hand side.
 * x(n) is the output solution vector.
 * No input quantities are destroyed, so the routine may be called sequentially with different b's.
 *
 * @param x solution of the system ([n x l] matrix)
 * @param u U matrix from SVD decomposition
 * @param w diagonal of the W matrix from the SVD decomposition
 * @param v V matrrix from SVD decomposition
 * @param b right-hand side input matrix from system to solve (column vector [m x l])
 * @param m number of rows of the matrix A
 * @param n number of columns of the matrix A
 * @param l number of columns of the matrix B
 */
void pprz_svd_solve_float(float **x, float **u, float *w, float **v, float **b, int m, int n, int l)
{
  int i, j, jj, k;
  float s;
  float tmp[n];
  for (k = 0; k < l; k++) { //Iterate on all column of b
    for (j = 0; j < n; j++) { //Calculate UTB
      s = 0.0;
      if (w[j] != 0.0) {   //Nonzero result only if wj is nonzero
        for (i = 0; i < m; i++) { s += u[i][j] * b[i][k]; }
        s /= w[j];         //This is the divide by wj
      }
      tmp[j] = s;
    }
    for (j = 0; j < n; j++) { //Matrix multiply by V to get answer
      s = 0.0;
      for (jj = 0; jj < n; jj++) { s += v[j][jj] * tmp[jj]; }
      x[j][k] = s;
    }
  }
}

