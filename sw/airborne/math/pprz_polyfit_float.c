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
 * @file pprz_polyfit_float.c
 * @brief Polynomial regression.
 *
 */

#include "math/pprz_polyfit_float.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_matrix_decomp_float.h"

/** Polynomial regression
 *
 * Polynomial regression is a form of linear regression in which the relationship between
 * the independent variable x and the dependent variable y is modelled as an nth order polynomial.
 *
 * Considering the regression model:
 *  @f[
 *  y_i = c_0 + c_1 x_i + ... + c_p x_i^p + \epsilon_i  (i = 1 ... n)
 *  @f]
 * in matrix form
 *  @f[
 *  y = X.c + \epsilon
 *  @f]
 * where
 *  @f[
 *  X_{ij} = x_i^j (i = 1 ... n; j = 1 ... p)
 *  @f]
 * The vector of estimated polynomial regression coefficients using ordinary least squares estimation is
 *  @f[
 *  c = (X' X)^{-1} X' y
 *  @f]
 *
 * http://en.wikipedia.org/wiki/Polynomial_regression
 * http://fr.wikipedia.org/wiki/R%C3%A9gression_polynomiale
 * http://www.arachnoid.com/sage/polynomial.html
 *
 * @param[in] x pointer to the input array of independent variable X [n]
 * @param[in] y pointer to the input array of dependent variable Y [n]
 * @param[in] n number of input measurments
 * @param[in] p degree of the output polynomial
 * @param[out] c pointer to the output array of polynomial coefficients [p+1]
 */
void pprz_polyfit_float(float *x, float *y, int n, int p, float *c)
{
  int i, j, k;

  // Instead of solving directly (X'X)^-1 X' y
  // let's build the matrices (X'X) and (X'y)
  // Then element ij in (X'X) matrix is sum_{k=0,n-1} x_k^(i+j)
  // and element i in (X'y) vector is sum_{k=0,n-1} x_k^i * y_k
  // Finally we can solve the linear system (X'X).c = (X'y) using SVD decomposition

  // First build a table of element S_i = sum_{k=0,n-1} x_k^i of dimension 2*p+1
  float S[2 * p + 1];
  float_vect_zero(S, 2 * p + 1);
  // and a table of element T_i = sum_{k=0,n-1} x_k^i*y_k of dimension p+1
  // make it a matrix for later use
  float _T[p + 1][1];
  MAKE_MATRIX_PTR(T, _T, p + 1);
  float_mat_zero(T, p + 1, 1);
  S[0] = n; // S_0 is always the number of input measurements
  for (k = 0; k < n; k++) {
    float x_tmp = x[k];
    T[0][0] += y[k];
    for (i = 1; i < 2 * p + 1; i++) {
      S[i] += x_tmp; // add element to S_i
      if (i < p + 1) {
        T[i][0] += x_tmp * y[k];  // add element to T_i if i < p+1
      }
      x_tmp *= x[k]; // multiply x_tmp by current value of x
    }
  }
  // Then build a [p+1 x p+1] matrix corresponding to (X'X) based on the S_i
  // element ij of (X'X) is S_(i+j)
  float _XtX[p + 1][p + 1];
  MAKE_MATRIX_PTR(XtX, _XtX, p + 1);
  for (i = 0; i < p + 1; i++) {
    for (j = 0; j < p + 1; j++) {
      XtX[i][j] = S[i + j];
    }
  }
  // Solve linear system XtX.c = T after performing a SVD decomposition of XtX
  // which is probably a bit overkill but looks really cool
  float w[p + 1], _v[p + 1][p + 1];
  MAKE_MATRIX_PTR(v, _v, p + 1);
  pprz_svd_float(XtX, w, v, p + 1, p + 1);
  float _c[p + 1][1];
  MAKE_MATRIX_PTR(c_tmp, _c, p + 1);
  pprz_svd_solve_float(c_tmp, XtX, w, v, T, p + 1, p + 1, 1);
  // set output vector
  for (i = 0; i < p + 1; i++) {
    c[i] = c_tmp[i][0];
  }

}


