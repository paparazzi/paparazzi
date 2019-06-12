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

#include "math/pprz_simple_matrix.h"
#include "math/pprz_matrix_decomp_float.h"
#include "math/pprz_algebra_float.h"
#include <math.h>
#include <string.h>

#if DEBUG_RANSAC
#include "stdio.h"
#define DEBUG_PRINT  printf
#define DEBUG_MAT_PRINT MAT_PRINT
#else
#define DEBUG_PRINT(...)
#define DEBUG_MAT_PRINT(...)
#endif

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
      if (i == j) {
        o[i][j] = sqrtf(in[i][i] - s);
      } else {
        if (o[j][j] != 0) {
          o[i][j] = 1.0 / o[j][j] * (in[i][j] - s);
        } else {
          o[i][j] = 0.0;
        }
      }
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
  float_mat_transpose_square(Q, m);
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
    if (absa == 0) { return 0.0; }
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
            if (H != 0.0) {
              F = S / H;
            } else {
              F = 0.0;
            }
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
        if (H == 0.0) {
          H = 0.00001f;
        }
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
          if (a[i][l] != 0) {
            v[j][i] = (a[i][j] / a[i][l]) / G;
          } else {
            v[j][i] = 0.0;
          }
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
            if (H != 0) {
              H = 1.0 / H;
            } else {
              H = 0;
            }
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
      if (H * Y != 0) {
        F = ((Y - Z) * (Y + Z) + (G - H) * (G + H)) / (2.0 * H * Y);
      } else {
        F = 0;
      }
      //G = sqrtf( F * F + 1.0 );
      G = pythag(F, 1.0);
      tmp = G;
      if (F < 0.0) {
        tmp = -tmp;
      }
      if ((F + tmp) != 0) {
        F = ((X - Z) * (X + Z) + H * ((Y / (F + tmp)) - H)) / X;
      } else {
        F = 0;
      }

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
        if (Z != 0) {
          C = F / Z;
          S = H / Z;
        } else {
          C = 0;
          S = 0;
        }
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


/**
 * Fit a linear model from samples to target values.
 * Effectively a wrapper for the pprz_svd_float and pprz_svd_solve_float functions.
 *
 * @param[in] targets The target values
 * @param[in] samples The samples / feature vectors
 * @param[in] D The dimensionality of the samples
 * @param[in] count The number of samples
 * @param[in] use_bias Whether to use the bias. Please note that params should always be of size D+1, but in case of no bias, the bias value is set to 0.
 * @param[out] parameters* Parameters of the linear fit
 * @param[out] fit_error* Total error of the fit
 */
void fit_linear_model(float *targets, int D, float (*samples)[D], uint16_t count, bool use_bias, float *params,
                      float *fit_error)
{

  // We will solve systems of the form A x = b,
  // where A = [nx(D+1)] matrix with entries [s1, ..., sD, 1] for each sample (1 is the bias)
  // and b = [nx1] vector with the target values.
  // x in the system are the parameters for the linear regression function.

  // local vars for iterating, random numbers:
  int sam, d;
  uint16_t n_samples = count;
  uint8_t D_1 = D + 1;
  // ensure that n_samples is high enough to ensure a result for a single fit:
  n_samples = (n_samples < D_1) ? D_1 : n_samples;
  // n_samples should not be higher than count:
  n_samples = (n_samples < count) ? n_samples : count;

  // initialize matrices and vectors for the full point set problem:
  // this is used for determining inliers
  float _AA[count][D_1];
  MAKE_MATRIX_PTR(AA, _AA, count);
  float _targets_all[count][1];
  MAKE_MATRIX_PTR(targets_all, _targets_all, count);

  for (sam = 0; sam < count; sam++) {
    for (d = 0; d < D; d++) {
      AA[sam][d] = samples[sam][d];
    }
    if (use_bias) {
      AA[sam][D] = 1.0f;
    } else {
      AA[sam][D] = 0.0f;
    }
    targets_all[sam][0] = targets[sam];
  }

  // decompose A in u, w, v with singular value decomposition A = u * w * vT.
  // u replaces A as output:
  float _parameters[D_1][1];
  MAKE_MATRIX_PTR(parameters, _parameters, D_1);
  float w[n_samples], _v[D_1][D_1];
  MAKE_MATRIX_PTR(v, _v, D_1);

  // solve the system:

  pprz_svd_float(AA, w, v, count, D_1);
  pprz_svd_solve_float(parameters, AA, w, v, targets_all, count, D_1, 1);

  // used to determine the error of a set of parameters on the whole set:
  float _bb[count][1];
  MAKE_MATRIX_PTR(bb, _bb, count);
  float _C[count][1];
  MAKE_MATRIX_PTR(C, _C, count);

  // error is determined on the entire set
  // bb = AA * parameters:
  // TODO: error: AA has been replaced in the pprz_svd_float procedure by U!!!
  MAT_MUL(count, D_1, 1, bb, AA, parameters);
  // subtract bu_all: C = 0 in case of perfect fit:
  MAT_SUB(count, 1, C, bb, targets_all);
  *fit_error = 0;
  for (sam = 0; sam < count; sam++) {
    *fit_error += fabsf(C[sam][0]);
  }
  if (count > 0) {
    *fit_error /= count;
  }

  for (d = 0; d < D_1; d++) {
    params[d] = parameters[d][0];
  }
}


/**
 * Fit a linear model from samples to target values with a prior.
 * Effectively a wrapper for the pprz_svd_float and pprz_svd_solve_float functions.
 *
 * @param[in] targets The target values
 * @param[in] samples The samples / feature vectors
 * @param[in] D The dimensionality of the samples
 * @param[in] count The number of samples
 * @param[in] use_bias Whether to use the bias. Please note that params should always be of size D+1, but in case of no bias, the bias value is set to 0.
 * @param[in] priors Prior per dimension. If use_bias, also for the dimension D+1.
 * @param[out] parameters* Parameters of the linear fit
 * @param[out] fit_error* Total error of the fit
 */
void fit_linear_model_prior(float *targets, int D, float (*samples)[D], uint16_t count, bool use_bias, float *priors,
                            float *params, float *fit_error)
{

  // We will solve systems of the form A x = b,
  // where A = [nx(D+1)] matrix with entries [s1, ..., sD, 1] for each sample (1 is the bias)
  // and b = [nx1] vector with the target values.
  // x in the system are the parameters for the linear regression function.

  // local vars for iterating, random numbers:
  int sam, d;
  uint16_t n_samples = count;
  uint8_t D_1 = D + 1;

  if (D_1 != 2) {
    DEBUG_PRINT("not yet implemented!!\n");
    return;
  }

  DEBUG_PRINT("n_samples = %d\n", n_samples);

  // ensure that n_samples is high enough to ensure a result for a single fit:
  n_samples = (n_samples < D_1) ? D_1 : n_samples;
  // n_samples should not be higher than count:
  n_samples = (n_samples < count) ? n_samples : count;
  count = n_samples;

  DEBUG_PRINT("n_samples = %d\n", n_samples);

  // initialize matrices and vectors for the full point set problem:
  // this is used for determining inliers
  float _AA[count][D_1];
  MAKE_MATRIX_PTR(AA, _AA, count);
  float _AAT[D_1][count];
  MAKE_MATRIX_PTR(AAT, _AAT, D_1);
  float _AATAA[D_1][D_1];
  MAKE_MATRIX_PTR(AATAA, _AATAA, D_1);
  float _PRIOR[D_1][D_1];
  MAKE_MATRIX_PTR(PRIOR, _PRIOR, D_1);
  for(d = 0; d < D; d++) {
    PRIOR[d][d] = priors[d];
  }
  if(use_bias) {
    PRIOR[D][D] = priors[D];
  }
  else {
    PRIOR[D][D] = 1.0f;
  }

  float _targets_all[count][1];
  MAKE_MATRIX_PTR(targets_all, _targets_all, count);

  for (sam = 0; sam < count; sam++) {
    for (d = 0; d < D; d++) {
      AA[sam][d] = samples[sam][d];
    }
    if (use_bias) {
      AA[sam][D] = 1.0f;
    } else {
      AA[sam][D] = 0.0f;
    }
    targets_all[sam][0] = targets[sam];
  }

  DEBUG_PRINT("A:\n");
  DEBUG_MAT_PRINT(count, D_1, AA);

  // make the pseudo-inverse matrix:
  float_mat_transpose(AAT, AA, count, D_1);

  DEBUG_PRINT("AT:\n");
  DEBUG_MAT_PRINT(D_1, count, AAT);

  float_mat_mul(AATAA, AAT, AA, D_1, count, D_1);

  DEBUG_PRINT("ATA:\n");
  DEBUG_MAT_PRINT(D_1, D_1, AATAA);

  // TODO: here, AATAA is used as float** - should be ok right?
  // add the prior to AATAA:
  float_mat_sum(AATAA, AATAA, PRIOR, D_1, D_1);

  DEBUG_PRINT("ATA+prior*I:\n");
  DEBUG_MAT_PRINT(D_1, D_1, AATAA);

  float _INV_AATAA[D_1][D_1];
  MAKE_MATRIX_PTR(INV_AATAA, _INV_AATAA, D_1);

  /*
  // 2-dimensional:
  float det = AATAA[0][0] * AATAA[1][1] - AATAA[0][1] * AATAA[1][0];
  if (fabsf(det) < 1e-4) {
    printf("Not invertible\n");
    return; // does this return go well?
  } //not invertible

  INV_AATAA[0][0] =  AATAA[1][1] / det;
  INV_AATAA[0][1] = -AATAA[0][1] / det;
  INV_AATAA[1][0] = -AATAA[1][0] / det;
  INV_AATAA[1][1] =  AATAA[0][0] / det;

  DEBUG_PRINT("INV assuming D_1 = 2:\n");
  DEBUG_MAT_PRINT(D_1, D_1, INV_AATAA);
  */

  // the AATAA matrix is square, so:
  float_mat_invert(INV_AATAA, AATAA, D_1);

  DEBUG_PRINT("GENERIC INV:\n");
  DEBUG_MAT_PRINT(D_1, D_1, INV_AATAA);


  //float_mat_inv_2d(INV_AATAA, _INV_AATAA);
  float _PINV[D_1][count];
  MAKE_MATRIX_PTR(PINV, _PINV, D_1);
  // TODO: what is the difference with float_mat_mul used above? Only that this is a matrix expansion?
  MAT_MUL(D_1, D_1, count, PINV, INV_AATAA, AAT);

  DEBUG_PRINT("PINV:\n");
  DEBUG_MAT_PRINT(D_1, count, PINV);

  float _parameters[D_1][1];
  MAKE_MATRIX_PTR(parameters, _parameters, D_1);
  MAT_MUL(D_1, count, 1, parameters, PINV, targets_all);

  DEBUG_PRINT("parameters:\n");
  DEBUG_MAT_PRINT(D_1, 1, parameters);

  // used to determine the error of a set of parameters on the whole set:
  float _bb[count][1];
  MAKE_MATRIX_PTR(bb, _bb, count);
  float _C[count][1];
  MAKE_MATRIX_PTR(C, _C, count);


  DEBUG_PRINT("A:\n");
  DEBUG_MAT_PRINT(count, D_1, AA);

  // error is determined on the entire set
  // bb = AA * parameters:
  MAT_MUL(count, D_1, 1, bb, AA, parameters);

  // subtract bu_all: C = 0 in case of perfect fit:
  MAT_SUB(count, 1, C, bb, targets_all);

  *fit_error = 0;
  for (sam = 0; sam < count; sam++) {
    *fit_error += fabsf(C[sam][0]);
  }
  *fit_error /= count;

  DEBUG_PRINT("Fit error = %f\n", *fit_error);

  for (d = 0; d < D_1; d++) {
    params[d] = parameters[d][0];
  }
  DEBUG_PRINT("End of the function\n");
}
