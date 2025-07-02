/*
 * This file is part of paparazzi
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

#include "math/pprz_circfit_float.h"
#include <math.h>

#ifndef PPRZ_CIRCFIT_EPSILON
#define PPRZ_CIRCFIT_EPSILON 1e-3f
#endif

#ifndef PPRZ_CIRCFIT_ITER_MAX
#define PPRZ_CIRCFIT_ITER_MAX 250
#endif

#ifndef PPRZ_CIRCFIT_NORM_CUTOFF
#define PPRZ_CIRCFIT_NORM_CUTOFF 1e-6f
#endif

enum CircFitStatus_t pprz_circfit_wei_float(struct circle_t *c, const float *x, const float *y, uint16_t n, struct circle_t *g) {

  // Check if initial guess is provided
  if (g != NULL) {
    c->r = g->r;
    c->x = g->x;
    c->y = g->y;
  } else {
    c->x = 0.0f;
    c->y = 0.0f;
    c->r = 1.0f;
  }

  float norm[n];
  float x_prev = 0;
  float y_prev = 0;
  float r_prev = -1;
  uint16_t iteration = 0;
    
  while (fabsf(c->r - r_prev) > PPRZ_CIRCFIT_EPSILON || fabsf(c->x - x_prev) > PPRZ_CIRCFIT_EPSILON || fabsf(c->y - y_prev) > PPRZ_CIRCFIT_EPSILON) {
    
    float sum_norm = 0.0f;
    x_prev = c->x;
    y_prev = c->y;
    r_prev = c->r;

    // Prepare ||x - c||
    for (int i = 0; i < n; i++) {
      norm[i] = sqrtf((x[i] - x_prev) * (x[i] - x_prev) + (y[i] - y_prev) * (y[i] - y_prev));
      sum_norm += norm[i];
    }

    c->r = sum_norm / n;
    
    for (int i = 0; i < n; i++) {
      if (norm[i] < PPRZ_CIRCFIT_NORM_CUTOFF)
      {
        return CIRC_FIT_NORM_ERROR; // Norm error, too small distance
      }
      c->x += x[i] + c->r * (x_prev - x[i]) / norm[i];
      c->y += y[i] + c->r * (y_prev - y[i]) / norm[i];
    }

    c->x /= n;
    c->y /= n;

    iteration++;

    if (iteration >= PPRZ_CIRCFIT_ITER_MAX) {
      return CIRC_FIT_ITERATION_LIMIT; // Reached iteration limit
    }

  }

  return CIRC_FIT_OK; // Circle fit successful

}