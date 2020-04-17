/*
 * Copyright (C) Anton Naruta && Daniel Hoppener
 * MAVLab Delft University of Technology
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
 */

/**
 * @brief Wrapper for qr solve
 *
 * Possible to use a different solver if needed.
 * Solves a system of the form Ax = b for x.
 *
 * @param m number of rows
 * @param n number of columns
 */
void qr_solve_wrapper(int m, int n, float** A, float* b, float* x);

/**
 * @brief active set algorithm for control allocation
 *
 * Takes the control objective and max and min inputs from pprz and calculates
 * the inputs that will satisfy most of the control objective, subject to the
 * weighting matrices Wv and Wu
 *
 * @param u The control output vector
 * @param v The control objective
 * @param umin The minimum u vector
 * @param umax The maximum u vector
 * @param B The control effectiveness matrix
 * @param n_u Length of u
 * @param n_v Lenght of v
 * @param u_guess Initial value for u
 * @param W_init Initial working set, if known
 * @param Wv Weighting on different control objectives
 * @param Wu Weighting on different controls
 * @param up Preferred control vector
 * @param gamma_sq Preference of satisfying control objective over desired
 * control vector (sqare root of gamma)
 * @param imax Max number of iterations
 *
 * @return Number of iterations, -1 upon failure
 */
int wls_alloc(float* u, float* v, float* umin, float* umax, float** B,
              float* u_guess, float* W_init, float* Wv, float* Wu,
              float* ud, float gamma, int imax);
