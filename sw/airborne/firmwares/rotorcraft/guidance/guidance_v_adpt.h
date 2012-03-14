/*
 * Copyright (C) 2009-2010 The Paparazzi Team
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

/** @file guidance_v_adapt.h
 *  Adaptation bloc of the vertical guidance.
 *
 *  This is a dimension one kalman filter estimating
 *  the ratio of vertical acceleration over thrust command ( ~ invert of the mass )
 *  needed by the invert dynamic model to produce a nominal command
 */

#ifndef GUIDANCE_V_ADPT
#define GUIDANCE_V_ADPT

extern int32_t gv_adapt_X;
extern int32_t gv_adapt_P;
extern int32_t gv_adapt_Xmeas;


#ifdef GUIDANCE_V_C

/** Supervision default bounds
 *  In case Asctec controllers are used without supervision
 * */
#ifndef SUPERVISION_MIN_MOTOR
#define SUPERVISION_MIN_MOTOR 1
#endif
#ifndef SUPERVISION_MAX_MOTOR
#define SUPERVISION_MAX_MOTOR 200
#endif

/** State of the estimator
 *  fixed point representation with #GV_ADAPT_X_FRAC
 *  Q13.18
 */
int32_t gv_adapt_X;
#define GV_ADAPT_X_FRAC 18

/** Covariance
 *  fixed point representation with #GV_ADAPT_P_FRAC
 *  Q13.18
 */
int32_t gv_adapt_P;
#define GV_ADAPT_P_FRAC 18

/** Measurement */
int32_t gv_adapt_Xmeas;


/* Initial State and Covariance    */
#define GV_ADAPT_X0_F 0.15
#define GV_ADAPT_X0 BFP_OF_REAL(GV_ADAPT_X0_F, GV_ADAPT_X_FRAC)
#define GV_ADAPT_P0_F 0.5
#define GV_ADAPT_P0 BFP_OF_REAL(GV_ADAPT_P0_F, GV_ADAPT_P_FRAC)

/* System  noises */
#define GV_ADAPT_SYS_NOISE_F 0.00005
#define GV_ADAPT_SYS_NOISE  BFP_OF_REAL(GV_ADAPT_SYS_NOISE_F, GV_ADAPT_P_FRAC)

/** Adapt noise factor
 * Smaller values will make the filter to adapter faster
 * Bigger values (slower adaptation) make the filter more robust to external perturbations
 * Factor should always be >0
 */
#ifndef GUIDANCE_V_ADAPT_NOISE_FACTOR
#define GUIDANCE_V_ADAPT_NOISE_FACTOR 1.0
#endif

/* Measuremement noises */
#define GV_ADAPT_MEAS_NOISE_HOVER_F (8.0*GUIDANCE_V_ADAPT_NOISE_FACTOR)
#define GV_ADAPT_MEAS_NOISE_HOVER BFP_OF_REAL(GV_ADAPT_MEAS_NOISE_HOVER_F, GV_ADAPT_P_FRAC)
#define GV_ADAPT_MEAS_NOISE_OF_ZD (20.0*GUIDANCE_V_ADAPT_NOISE_FACTOR)

/** Filter is not fed if accel values are more than +/- MAX_ACCEL
 * MAX_ACCEL is a positive value in m/s^2
 */
#ifndef GUIDANCE_V_ADAPT_MAX_ACCEL
#define GUIDANCE_V_ADAPT_MAX_ACCEL 4.0
#endif
#define GV_ADAPT_MAX_ACCEL ACCEL_BFP_OF_REAL(GUIDANCE_V_ADAPT_MAX_ACCEL)

/** Filter is not fed if command values are out of a % of MIN/MAX_SUPERVISION
 * MAX_CMD and MIN_CMD must be between 0 and 1 with MIN_CMD < MAX_CMD
 */
#ifndef GUIDANCE_V_ADAPT_MAX_CMD
#define GUIDANCE_V_ADAPT_MAX_CMD 0.9
#endif
#ifndef GUIDANCE_V_ADAPT_MIN_CMD
#define GUIDANCE_V_ADAPT_MIN_CMD 0.1
#endif

#define GV_ADAPT_MAX_CMD ((int32_t)(SUPERVISION_MIN_MOTOR + (GUIDANCE_V_ADAPT_MAX_CMD * (SUPERVISION_MAX_MOTOR - SUPERVISION_MIN_MOTOR))))
#define GV_ADAPT_MIN_CMD ((int32_t)(SUPERVISION_MIN_MOTOR + (GUIDANCE_V_ADAPT_MIN_CMD * (SUPERVISION_MAX_MOTOR - SUPERVISION_MIN_MOTOR))))


static inline void gv_adapt_init(void) {
  gv_adapt_X = GV_ADAPT_X0;
  gv_adapt_P = GV_ADAPT_P0;
}

/** Adaptation function
 * zdd_meas : INT32_ACCEL_FRAC
 * thrust_applied : controller input [SUPERVISION_MIN_MOTOR, SUPERVISION_MAX_MOTOR]
 * zd_ref: INT32_SPEED_FRAC
 */
#define K_FRAC 12
static inline void gv_adapt_run(int32_t zdd_meas, int32_t thrust_applied, int32_t zd_ref) {

  /* Do you really think we want to divide by zero ?
   * Negative commands are also prohibited here
   */
  if (thrust_applied <= 0) return;

  /* We don't propagate state, it's constant !       */
  /* We propagate our covariance                     */
  gv_adapt_P =  gv_adapt_P + GV_ADAPT_SYS_NOISE;

  /* Update only if accel and commands are in a valid range */
  if (thrust_applied < GV_ADAPT_MIN_CMD || thrust_applied > GV_ADAPT_MAX_CMD
      || zdd_meas < -GV_ADAPT_MAX_ACCEL || zdd_meas > GV_ADAPT_MAX_ACCEL) {
    return;
  }

  /* Compute our measurement. If zdd_meas is in the range +/-5g, meas is less than 24 bits */
  const int32_t g_m_zdd = ((int32_t)BFP_OF_REAL(9.81, INT32_ACCEL_FRAC) - zdd_meas)<<(GV_ADAPT_X_FRAC - INT32_ACCEL_FRAC);
  if ( g_m_zdd > 0) {
    gv_adapt_Xmeas = (g_m_zdd + (thrust_applied>>1)) / thrust_applied;
  } else {
    gv_adapt_Xmeas = (g_m_zdd - (thrust_applied>>1)) / thrust_applied;
  }

  /* Compute a residual */
  int32_t residual = gv_adapt_Xmeas - gv_adapt_X;

  /* Covariance Error   */
  int32_t ref = zd_ref >> (INT32_SPEED_FRAC - GV_ADAPT_P_FRAC);
  if (zd_ref < 0) ref = -ref;
  int32_t E = gv_adapt_P + GV_ADAPT_MEAS_NOISE_HOVER + ref * GV_ADAPT_MEAS_NOISE_OF_ZD;

  /* Kalman gain        */
  int32_t K = (gv_adapt_P<<K_FRAC) / E;

  /* Update Covariance */
  gv_adapt_P = gv_adapt_P - ((K * gv_adapt_P)>>K_FRAC);
  /* Don't let covariance climb over initial value */
  if (gv_adapt_P > GV_ADAPT_P0) gv_adapt_P = GV_ADAPT_P0;

  /* Update State */
  gv_adapt_X = gv_adapt_X + ((K * residual)>>K_FRAC);
  /* Again don't let it climb over a value that would
     give less than zero throttle and don't let it down to zero.
     30254 = MAX_ACCEL*GV_ADAPT_X_FRAC/MAX_THROTTLE
     aka
     30254 = 3*9.81*2^8/255
     2571632 = 9.81*2^18
  */
  Bound(gv_adapt_X, 10000, 2571632);
}


#endif /* GUIDANCE_V_C */

#endif /* GUIDANCE_V_ADPT */
