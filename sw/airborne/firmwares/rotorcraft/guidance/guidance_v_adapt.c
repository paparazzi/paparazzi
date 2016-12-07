/*
 * Copyright (C) 2009-2013 The Paparazzi Team
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

/** @file firmwares/rotorcraft/guidance/guidance_v_adapt.c
 *  Adaptation block of the vertical guidance.
 *
 *  This is a dimension one kalman filter estimating
 *  the ratio of vertical acceleration over thrust command ( ~ inverse of the mass )
 *  needed by the invert dynamic model to produce a nominal command.
 */

#include "firmwares/rotorcraft/guidance/guidance_v_adapt.h"
#include "paparazzi.h"
#include "math/pprz_algebra_int.h"
#include "generated/airframe.h"


/** Initial hover throttle as factor of MAX_PPRZ.
 *  Should be a value between #GUIDANCE_V_ADAPT_MIN_HOVER_THROTTLE and
 *  #GUIDANCE_V_ADAPT_MAX_HOVER_THROTTLE.
 *  If the nominal hover throttle is defined use it otherwise it is better
 *  to start with low thrust and let it rise as the adaptive filter finds
 *  the vehicle needs more thrust.
 */
#ifndef GUIDANCE_V_ADAPT_INITIAL_HOVER_THROTTLE
#ifdef GUIDANCE_V_NOMINAL_HOVER_THROTTLE
#define GUIDANCE_V_ADAPT_INITIAL_HOVER_THROTTLE GUIDANCE_V_NOMINAL_HOVER_THROTTLE
#else
#define GUIDANCE_V_ADAPT_INITIAL_HOVER_THROTTLE 0.3
#endif
#endif
PRINT_CONFIG_VAR(GUIDANCE_V_ADAPT_INITIAL_HOVER_THROTTLE)

/** Minimum hover throttle as factor of MAX_PPRZ.
 *  With the default of 0.2 the nominal hover throttle will
 *  never go lower than 20%.
 */
#ifndef GUIDANCE_V_ADAPT_MIN_HOVER_THROTTLE
#define GUIDANCE_V_ADAPT_MIN_HOVER_THROTTLE 0.2
#endif
PRINT_CONFIG_VAR(GUIDANCE_V_ADAPT_MIN_HOVER_THROTTLE)

/** Maximum hover throttle as factor of MAX_PPRZ.
 *  With the default of 0.75 the nominal hover throttle will
 *  never go over 75% of max throttle.
 */
#ifndef GUIDANCE_V_ADAPT_MAX_HOVER_THROTTLE
#define GUIDANCE_V_ADAPT_MAX_HOVER_THROTTLE 0.75
#endif
PRINT_CONFIG_VAR(GUIDANCE_V_ADAPT_MAX_HOVER_THROTTLE)

/** Adapt noise factor.
 *  Smaller values will make the filter to adapt faster.
 *  Bigger values (slower adaptation) make the filter more robust to external pertubations.
 *  Factor should always be >0
 */
#ifndef GUIDANCE_V_ADAPT_NOISE_FACTOR
#define GUIDANCE_V_ADAPT_NOISE_FACTOR 1.0
#endif


/** Filter is not fed if accel values are more than +/- MAX_ACCEL.
 *  MAX_ACCEL is a positive value in m/s^2
 */
#ifndef GUIDANCE_V_ADAPT_MAX_ACCEL
#define GUIDANCE_V_ADAPT_MAX_ACCEL 4.0
#endif

/** Filter is not fed if command values are out of a % of 0/MAX_PPRZ.
 *  MAX_CMD and MIN_CMD must be between 0 and 1 with MIN_CMD < MAX_CMD
 */
#ifndef GUIDANCE_V_ADAPT_MAX_CMD
#define GUIDANCE_V_ADAPT_MAX_CMD 0.9
#endif
#ifndef GUIDANCE_V_ADAPT_MIN_CMD
#define GUIDANCE_V_ADAPT_MIN_CMD 0.1
#endif



int32_t gv_adapt_X;
int32_t gv_adapt_P;
int32_t gv_adapt_Xmeas;

/* System  noises */
#ifndef GV_ADAPT_SYS_NOISE_F
#define GV_ADAPT_SYS_NOISE_F 0.00005
#endif
#define GV_ADAPT_SYS_NOISE  BFP_OF_REAL(GV_ADAPT_SYS_NOISE_F, GV_ADAPT_P_FRAC)

/* Measuremement noises */
#define GV_ADAPT_MEAS_NOISE_HOVER_F (50.0*GUIDANCE_V_ADAPT_NOISE_FACTOR)
#define GV_ADAPT_MEAS_NOISE_HOVER BFP_OF_REAL(GV_ADAPT_MEAS_NOISE_HOVER_F, GV_ADAPT_P_FRAC)
#define GV_ADAPT_MEAS_NOISE_OF_ZD (100.0*GUIDANCE_V_ADAPT_NOISE_FACTOR)

/* Initial Covariance    */
#define GV_ADAPT_P0_F 0.1
static const int32_t gv_adapt_P0 = BFP_OF_REAL(GV_ADAPT_P0_F, GV_ADAPT_P_FRAC);
static const int32_t gv_adapt_X0 = BFP_OF_REAL(9.81, GV_ADAPT_X_FRAC) /
                                   (GUIDANCE_V_ADAPT_INITIAL_HOVER_THROTTLE *MAX_PPRZ);

void gv_adapt_init(void)
{
  gv_adapt_X = gv_adapt_X0;
  gv_adapt_P = gv_adapt_P0;
}

#define K_FRAC 12

/** Adaptation function.
 * @param zdd_meas        vert accel measurement in m/s^2 with #INT32_ACCEL_FRAC
 * @param thrust_applied  controller input [0 : MAX_PPRZ]
 * @param zd_ref          vertical speed reference in m/s with #INT32_SPEED_FRAC
 */
void gv_adapt_run(int32_t zdd_meas, int32_t thrust_applied, int32_t zd_ref)
{

  static const int32_t gv_adapt_min_cmd = GUIDANCE_V_ADAPT_MIN_CMD * MAX_PPRZ;
  static const int32_t gv_adapt_max_cmd = GUIDANCE_V_ADAPT_MAX_CMD * MAX_PPRZ;
  static const int32_t gv_adapt_max_accel = ACCEL_BFP_OF_REAL(GUIDANCE_V_ADAPT_MAX_ACCEL);

  /* Update only if accel and commands are in a valid range */
  /* This also ensures we don't divide by zero */
  if (thrust_applied < gv_adapt_min_cmd || thrust_applied > gv_adapt_max_cmd
      || zdd_meas < -gv_adapt_max_accel || zdd_meas > gv_adapt_max_accel) {
    return;
  }

  /* We don't propagate state, it's constant !       */
  /* We propagate our covariance                     */
  gv_adapt_P =  gv_adapt_P + GV_ADAPT_SYS_NOISE;

  /* Compute our measurement. If zdd_meas is in the range +/-5g, meas is less than 30 bits */
  const int32_t g_m_zdd = ((int32_t)BFP_OF_REAL(9.81,
                           INT32_ACCEL_FRAC) - zdd_meas) << (GV_ADAPT_X_FRAC - INT32_ACCEL_FRAC);
  if (g_m_zdd > 0) {
    gv_adapt_Xmeas = (g_m_zdd + (thrust_applied >> 1)) / thrust_applied;
  } else {
    gv_adapt_Xmeas = (g_m_zdd - (thrust_applied >> 1)) / thrust_applied;
  }

  /* Compute a residual */
  int32_t residual = gv_adapt_Xmeas - gv_adapt_X;

  /* Covariance Error  E = P + R  */
  int32_t ref = zd_ref >> (INT32_SPEED_FRAC - GV_ADAPT_P_FRAC);
  if (zd_ref < 0) { ref = -ref; }
  int32_t E = gv_adapt_P + GV_ADAPT_MEAS_NOISE_HOVER + ref * GV_ADAPT_MEAS_NOISE_OF_ZD;

  /* Kalman gain  K = P / (P + R) = P / E  */
  int32_t K = (gv_adapt_P << K_FRAC) / E;

  /* Update Covariance  Pnew = P - K * P   */
  gv_adapt_P = gv_adapt_P - ((K * gv_adapt_P) >> K_FRAC);
  /* Don't let covariance climb over initial value */
  if (gv_adapt_P > gv_adapt_P0) {
    gv_adapt_P = gv_adapt_P0;
  }

  /* Update State */
  gv_adapt_X = gv_adapt_X + ((((int64_t)K * residual)) >> K_FRAC);

  /* Output bounds.
   * Don't let it climb over a value that would
   * give less than #GUIDANCE_V_ADAPT_MIN_HOVER_THROTTLE % throttle
   * or more than #GUIDANCE_V_ADAPT_MAX_HOVER_THROTTLE % throttle.
   */
  static const int32_t max_out = BFP_OF_REAL(9.81, GV_ADAPT_X_FRAC) /
                                 (GUIDANCE_V_ADAPT_MIN_HOVER_THROTTLE * MAX_PPRZ);
  static const int32_t min_out = BFP_OF_REAL(9.81, GV_ADAPT_X_FRAC) /
                                 (GUIDANCE_V_ADAPT_MAX_HOVER_THROTTLE * MAX_PPRZ);
  Bound(gv_adapt_X, min_out, max_out);
}
