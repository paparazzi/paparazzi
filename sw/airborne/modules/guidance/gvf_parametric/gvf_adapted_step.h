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

/**
 * @file modules/guidance/gvf_parametric/gvf_adapted_step.h
 *
 * Dynamic parametric step adaptation for the GVF algorithm 
 */

#ifndef GVF_ADAPTED_STEP_H
#define GVF_ADAPTED_STEP_H

// Numbers with absolute values smaller than `NULL_TOLERANCE` will be considered equal to 0
#ifndef NULL_TOLERANCE
#define NULL_TOLERANCE 1e-6
#endif

// Maximum number of steps used in the root finding algorithm
#ifndef MAX_STEPS
#define MAX_STEPS 1e6
#endif


/**
 * @brief Compute the adapted parametric step given the wanted geometric distance
 * 
 * Given the 3D parametric curve described by f at p and a wanted geometric distance ds, 
 * computes the parametric step dp such that:
 *    |ds| = || f(p+dp) - f(p) || and (dp)(dp) > 0
 * 
 * To do so, it uses second order Taylor approximation around p to reduce the problem to finding
 * a root of a degree 4 polynomial. This root is found using Halley's method.
 * 
 * We assume the curve non 2-singular (that is both f' and f'' does not vanish).
 * (If it is the case, then the method falls back to returning dp = ds, and print an error to `stderr`).
 * 
 * @param ds Wanted geometric distance
 * @param f1d x value of f'(p)
 * @param f2d y value of f'(p)
 * @param f3d z value of f'(p)
 * @param f1dd x value of f''(p)
 * @param f2dd y value of f''(p)
 * @param f3dd z value of f''(p)
 * @return float The corresponding parametric step to take
 */
float step_adaptation(float ds, float f1d, float f2d, float f3d, float f1dd, float f2dd, float f3dd);

#endif // GVF_ADAPTED_STEP_H