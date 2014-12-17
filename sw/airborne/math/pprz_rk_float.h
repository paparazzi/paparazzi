/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 * @file pprz_rk_float.h
 * @brief Runge-Kutta library (float version)
 *
 */

#ifndef PPRZ_RK_FLOAT_H
#define PPRZ_RK_FLOAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "math/pprz_algebra_float.h"

/** First-Order Runge-Kutta
 *
 * aka RK1, aka the euler method
 *
 * considering x' = f(x,u)
 * with x = x0 the initial state
 * and u the command vector
 *
 * x_new = x + dt * f(x, u)
 * is the integrated state vector x based on model f under command u
 *
 * @param xo output integrated state
 * @param x initial state
 * @param n state dimension
 * @param u command vector
 * @param m command dimension
 * @param f model function
 * @param dt integration step
 */
static inline void runge_kutta_1_float(
  float *xo,
  const float *x, const int n,
  const float *u, const int m,
  void (*f)(float *o, const float *x, const int n, const float *u, const int m),
  const float dt)
{
  float dx[n];

  f(dx, x, n, u, m);
  float_vect_smul(xo, dx, dt, n);
  float_vect_add(xo, x, n);
}

/** Second-Order Runge-Kutta
 *
 * aka RK2, aka the mid-point method
 *
 * considering x' = f(x,u)
 * with x = x0 the initial state
 * and u the command vector
 *
 * mid_point = x + (dt/2)*f(x, u)
 * x_new = x + dt * f(mid_point, u)
 * is the integrated state vector x based on model f under command u
 *
 * @param xo output integrated state
 * @param x initial state
 * @param n state dimension
 * @param u command vector
 * @param m command dimension
 * @param f model function
 * @param dt integration step
 */
static inline void runge_kutta_2_float(
  float *xo,
  const float *x, const int n,
  const float *u, const int m,
  void (*f)(float *o, const float *x, const int n, const float *u, const int m),
  const float dt)
{
  float mid_point[n];

  // mid_point = x + (dt/2)*f(x, u)
  f(mid_point, x, n, u, m);
  float_vect_smul(mid_point, mid_point, dt / 2., n);
  float_vect_add(mid_point, x, n);
  // xo = x + dt * f(mid_point, u)
  f(xo, mid_point, n, u, m);
  float_vect_smul(xo, xo, dt, n);
  float_vect_add(xo, x, n);
}

/** Fourth-Order Runge-Kutta
 *
 * aka RK4, aka 'the' Runge-Kutta
 *
 * considering x' = f(x,u)
 * with x = x0 the initial state
 * and u the command vector
 *
 * k1 = f(x, u)
 * k2 = f(x + dt * (k1 / 2), u)
 * k3 = f(x + dt * (k2 / 2), u)
 * k4 = f(x + dt * k3, u)
 *
 * x_new = x + (dt / 6) * (k1 + 2 * (k2 + k3) + k4)
 * is the integrated state vector x based on model f under command u
 *
 * @param xo output integrated state
 * @param x initial state
 * @param n state dimension
 * @param u command vector
 * @param m command dimension
 * @param f model function
 * @param dt integration step
 */
static inline void runge_kutta_4_float(
  float *xo,
  const float *x, const int n,
  const float *u, const int m,
  void (*f)(float *o, const float *x, const int n, const float *u, const int m),
  const float dt)
{
  float k1[n], k2[n], k3[n], k4[n], ktmp[n];

  // k1 = f(x, u)
  f(k1, x, n, u, m);

  // k2 = f(x + dt * (k1 / 2), u)
  float_vect_smul(ktmp, k1, dt / 2., n);
  float_vect_add(ktmp, x, n);
  f(k2, ktmp, n, u, m);

  // k3 = f(x + dt * (k2 / 2), u)
  float_vect_smul(ktmp, k2, dt / 2., n);
  float_vect_add(ktmp, x, n);
  f(k3, ktmp, n, u, m);

  // k4 = f(x + dt * k3, u)
  float_vect_smul(ktmp, k3, dt, n);
  float_vect_add(ktmp, x, n);
  f(k4, ktmp, n, u, m);

  // xo = x + (dt / 6) * (k1 + 2 * (k2 + k3) + k4)
  float_vect_add(k2, k3, n);
  float_vect_smul(k2, k2, 2., n);
  float_vect_add(k1, k2, n);
  float_vect_add(k1, k4, n);
  float_vect_smul(k1, k1, dt / 6., n);
  float_vect_sum(xo, x, k1, n);
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_RK_FLOAT_H */
