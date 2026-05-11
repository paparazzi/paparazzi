/*
 * Copyright (C) 2023 Mael Feurgard <maelfeurgard@gmail.com>
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


#include <math.h>
#include "std.h"

/**
 * @file modules/guidance/gvf_parametric/gvf_adapted_step.c
 *
 * Dynamic parametric step adaptation for the GVF algorithm
 */

#include "gvf_adapted_step.h"

// Numbers with absolute values smaller than `ADAPTED_STEP_NULL_TOLERANCE` will be considered equal to 0
#ifndef ADAPTED_STEP_NULL_TOLERANCE
  #define ADAPTED_STEP_NULL_TOLERANCE 1e-6
#endif

// Maximum number of steps used in the root finding algorithm
#ifndef ADAPTED_STEP_MAX_ROOTFINDING_STEPS
  #define ADAPTED_STEP_MAX_ROOTFINDING_STEPS 1e6
#endif

/**
 * @brief Horner's method for fast degree 4 polynomial evaluation
 *
 * Cf https://en.wikipedia.org/wiki/Horner's_method
 *
 * @param x : Evaluation point for the polynomial
 * @param a4 : Degree 4 coefficient of the polynomial
 * @param a3 : Degree 3 coefficient of the polynomial
 * @param a2 : Degree 2 coefficient of the polynomial
 * @param a1 : Degree 1 coefficient of the polynomial
 * @param a0 : Degree 0 coefficient of the polynomial
 * @return float : Value of the polynomial evaluated at `x`
 */
static float p4_eval(float x, float a4, float a3, float a2, float a1, float a0)
{
  return a0 + x * (a1 + x * (a2 + x * (a3 + x * (a4))));
}

/**
 * @brief Implementation of Halley's method for degree 4 polynomial root finding
 *
 * Cf https://en.wikipedia.org/wiki/Halley%27s_method
 *
 * @param a4 : Degree 4 coefficient of the polynomial
 * @param a3 : Degree 3 coefficient of the polynomial
 * @param a2 : Degree 2 coefficient of the polynomial
 * @param a1 : Degree 1 coefficient of the polynomial
 * @param a0 : Degree 0 coefficient of the polynomial
 * @param tol : Tolerance; the root is found if the absolute value of the polynomial evaluation is below the tolerance
 * @param init : Initial guess for the root
 * @param max_steps : Maximal number of iterations allowed
 * @return float : The guessed root (or whatever value is left after `max_steps` iteration)
 */
static float p4_halley(float a4, float a3, float a2, float a1, float a0, float tol, float init, int max_steps)
{
  float x = init;
  float p_x = p4_eval(x, a4, a3, a2, a1, a0);

  // Coefficients of the polynomial first derivative
  float a4_d = 0.;
  float a3_d = 4 * a4;
  float a2_d = 3 * a3;
  float a1_d = 2 * a2;
  float a0_d = a1;

  // Coefficients of the polynomial second derivative
  float a4_dd = 0.;
  float a3_dd = 0.;
  float a2_dd = 3 * a3_d;
  float a1_dd = 2 * a2_d;
  float a0_dd = a1_d;

  int step = 0;

  while (ABS(p_x) > tol && step < max_steps) {
    float p_xd = p4_eval(x, a4_d, a3_d, a2_d, a1_d, a0_d);
    float p_xdd = p4_eval(x, a4_dd, a3_dd, a2_dd, a1_dd, a0_dd);
    p_x = p4_eval(x, a4, a3, a2, a1, a0);
    x = x - (2 * p_x * p_xd) / (2 * p_xd * p_xd - p_x * p_xdd);
    step++;
  }

  return x;
}

float step_adaptation(float ds, float f1d, float f2d, float f3d, float f1dd, float f2dd, float f3dd)
{
  float a4 = (f1dd * f1dd + f2dd * f2dd + f3dd * f3dd) / 4.;
  float a3 = (f1d * f1dd + f2d * f2dd + f3d * f3dd);
  float a2 = (f1d * f1d + f2d * f2d + f3d * f3d);
  float a1 = 0.;
  float a0 = - ds * ds;


  if (a4 < ADAPTED_STEP_NULL_TOLERANCE && a2 < ADAPTED_STEP_NULL_TOLERANCE) {
    // fprintf(stderr,"***** Error: order 2 singularity (f' and f'' are null) *****\n");
    // fprintf(stderr, "Falling back to 'natural' parametrization'\n");
    return ds;
  }

  if (a4 < ADAPTED_STEP_NULL_TOLERANCE && a2 > ADAPTED_STEP_NULL_TOLERANCE) {
    // Second derivative is null; this is a degree 2 polynomial
    return ds / sqrtf(a2);
  }

  if (a4 > ADAPTED_STEP_NULL_TOLERANCE && a2 < ADAPTED_STEP_NULL_TOLERANCE) {
    // First derivative is null, but not the second; immediate analytical solution
    float output = sqrtf(ABS(ds)) / sqrtf(sqrtf(a4));
    if (ds > 0) {
      return output;
    } else {
      return - output;
    }
  }


  float init;

  {
    // Compute derivative's (reduced polynomial) discriminant
    // float a4_d = 0.; unused
    float a3_d = 4 * a4;
    float a2_d = 3 * a3;
    float a1_d = 2 * a2;
    // float a0_d = a1; // = 0; unused

    float discr = a2_d * a2_d - 4 * a3_d * a1_d;

    if (discr < 0) {
      // If there are no additional roots, use the 1st order approx as starting point
      init = ds / sqrtf(a2);
    } else {
      // If there are additional roots, use the closest one
      if (ds > 0) {
        // Positive direction
        if (a2_d < 0) {
          // Additional roots are on this side, use the closest to 0 for reference
          init = (-a2_d - sqrtf(discr)) / (4 * a3_d);
        } else {
          // No additional roots
          init = ds / sqrtf(a2);
        }
      } else {
        if (a2_d < 0) {
          // No additional roots
          init = ds / sqrtf(a2);
        } else {
          // Additional roots are on this side, use the closest to 0 for reference
          init = (-a2_d + sqrtf(discr)) / (4 * a3_d);
        }
      }
    }
  }

  float result = p4_halley(a4, a3, a2, a1, a0, ADAPTED_STEP_NULL_TOLERANCE, init, ADAPTED_STEP_MAX_ROOTFINDING_STEPS);

  if (result * ds < 0) {
    result = p4_halley(a4, a3, a2, a1, a0, ADAPTED_STEP_NULL_TOLERANCE, 2 * init, ADAPTED_STEP_MAX_ROOTFINDING_STEPS);
  }

  return result;
}