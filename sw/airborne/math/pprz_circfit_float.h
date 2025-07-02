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

#ifndef PPRZ_CIRCFIT_FLOAT_H
#define PPRZ_CIRCFIT_FLOAT_H

#include <std.h>

enum CircFitStatus_t {
  CIRC_FIT_OK = 0,                  // Circle fit successful
  CIRC_FIT_ERROR = -1,              // Circle fit failed
  CIRC_FIT_ITERATION_LIMIT = -2,     // Circle fit reached iteration limit
  CIRC_FIT_NORM_ERROR = -3,          // Circle fit norm error
};

struct circle_t {
  float x;  // x coordinate of the circle center
  float y;  // y coordinate of the circle center
  float r;  // radius of the circle
};

/*
  * Fitting Noisy Data to a Circle: A Simple Iterative Maximum Likelihood Approach
  * Wei Li, et al.
  * DOI: 10.1109/icc.2011.5963101
  * 
  * inputs:
  *   c: struct to store the fitted circle
  *   x: x coordinates used during fitting
  *   y: y coordinates used during fitting
  *   n: number of points
  *   g: initial guess for the circle (NULL for no initial guess)
  */
enum CircFitStatus_t pprz_circfit_wei_float(struct circle_t *c, const float *x, const float *y, uint16_t n, struct circle_t *g); 


#endif // PPRZ_CIRCFIT_FLOAT_H