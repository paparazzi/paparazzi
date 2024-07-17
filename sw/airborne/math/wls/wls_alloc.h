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
 * @brief active set algorithm for control allocation
 *
 * Takes the control objective and max and min inputs from pprz and calculates
 * the inputs that will satisfy most of the control objective, subject to the
 * weighting matrices Wv and Wu
 *
 * The dimension of the input vectors u and v are defined at compilation time
 * and must be large enough for all the considered cases.
 *
 * @param u The control output vector
 * @param v The control objective vector
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
 * @param n_u Length of u (the number of actuators)
 * @param n_v Lenght of v (the number of control objectives)
 *
 * @return Number of iterations: (imax+1) means it ran out of iterations
 */

#ifndef WLS_ALLOC_HEADER
#define WLS_ALLOC_HEADER

#ifndef TEST_ALLOC
#include "generated/airframe.h"
#endif

#ifndef WLS_N_U_MAX
#define WLS_N_U_MAX 6
#endif

#ifndef WLS_N_V_MAX
#define WLS_N_V_MAX 4
#endif
struct WLS_t{
  int nu;                    // number of actuators
  int nv;                    // number of controlled axes
  float gamma_sq;            // weighting factor WLS
  float v[WLS_N_V_MAX];      // Pseudo Control Vector
  float u[WLS_N_U_MAX];      // Allocation of Controls
  float Wv[WLS_N_V_MAX];     // Weighting on different control objectives
  float Wu[WLS_N_U_MAX];     // Weighting on different actuators
  float u_pref[WLS_N_U_MAX]; // Preferred control vector
  float u_min[WLS_N_U_MAX];  // Minimum control vector
  float u_max[WLS_N_U_MAX];  // Maximum control vector
  float PC;                  // Primary cost
  float SC;                  // Secondary cost
  int   iter;                // Number of iterations
};

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
extern void send_wls_v(char *name, struct WLS_t* WLS_p, struct transport_tx *trans, struct link_device *dev);
extern void send_wls_u(char *name, struct WLS_t* WLS_p, struct transport_tx *trans, struct link_device *dev);
#endif


extern void wls_alloc(struct WLS_t* WLS_p, float **B, float *u_guess, float *W_init, int imax);


#endif