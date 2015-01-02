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
 * @file math/pprz_polyfit_float.h
 * @brief Polynomial regression.
 *
 */

#ifndef PPRZ_POLYFIT_FLOAT_H
#define PPRZ_POLYFIT_FLOAT_H

#ifdef __cplusplus
extern "C" {
#endif

/** Polynomial regression
 *
 * Polynomial regression is a form of linear regression in which the relationship between
 * the independent variable x and the dependent variable y is modelled as an nth order polynomial.
 *
 * Considering the regression model:
 *  @f[
 *  y_i = a_0 + a_1 x_i + ... + a_p x_i^p + \epsilon_i  (i = 1 ... n)
 *  @f]
 * in matrix form
 *  @f[
 *  y = Xa + \epsilon
 *  @f]
 * where
 *  @f[
 *  X_{ij} = x_i^j (i = 1 ... n; j = 1 ... p)
 *  @f]
 * The vector of estimated polynomial regression coefficients using ordinary least squares estimation is
 *  @f[
 *  a = (X' X)^{-1} X' y
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
 * @param[out] c pointer to the output array of polynomial coefficients [p]
 */
void pprz_polyfit_float(float *x, float *y, int n, int p, float *c);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_POLYFIT_FLOAT_H */

