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

/**
 * @file firmwares/fixedwing/stabilization/stabilization_adaptive.c
 *
 * Fixed wing horizontal adaptive control.
 *
 */

#include "std.h"
#include "led.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#include "state.h"
#include "subsystems/nav.h"
#include "generated/airframe.h"
#include CTRL_TYPE_H
#include "firmwares/fixedwing/autopilot.h"

/* outer loop parameters */
float h_ctl_course_setpoint; /* rad, CW/north */
float h_ctl_course_pre_bank;
float h_ctl_course_pre_bank_correction;
float h_ctl_course_pgain;
float h_ctl_course_dgain;
float h_ctl_roll_max_setpoint;

/* roll and pitch disabling */
bool_t h_ctl_disabled;

/* AUTO1 rate mode */
bool_t h_ctl_auto1_rate;


float h_ctl_ref_roll_angle;
float h_ctl_ref_roll_rate;
float h_ctl_ref_roll_accel;
float h_ctl_ref_pitch_angle;
float h_ctl_ref_pitch_rate;
float h_ctl_ref_pitch_accel;
#define H_CTL_REF_W 5.
#define H_CTL_REF_XI 0.85
#define H_CTL_REF_MAX_P RadOfDeg(150.)
#define H_CTL_REF_MAX_P_DOT RadOfDeg(500.)
#define H_CTL_REF_MAX_Q RadOfDeg(150.)
#define H_CTL_REF_MAX_Q_DOT RadOfDeg(500.)

/* inner roll loop parameters */
float h_ctl_roll_setpoint;
float h_ctl_roll_attitude_gain;
float h_ctl_roll_rate_gain;
float h_ctl_roll_igain;
float h_ctl_roll_sum_err;
float h_ctl_roll_Kffa;
float h_ctl_roll_Kffd;
pprz_t h_ctl_aileron_setpoint;
float h_ctl_roll_slew;

float h_ctl_roll_pgain;

/* inner pitch loop parameters */
float h_ctl_pitch_setpoint;
float h_ctl_pitch_loop_setpoint;
float h_ctl_pitch_pgain;
float h_ctl_pitch_dgain;
float h_ctl_pitch_igain;
float h_ctl_pitch_sum_err;
float h_ctl_pitch_Kffa;
float h_ctl_pitch_Kffd;
pprz_t h_ctl_elevator_setpoint;

/* inner loop pre-command */
float h_ctl_aileron_of_throttle;
float h_ctl_elevator_of_roll;
float h_ctl_pitch_of_roll; // Should be used instead of elevator_of_roll

bool_t use_airspeed_ratio;
float airspeed_ratio2;
#define AIRSPEED_RATIO_MIN 0.5
#define AIRSPEED_RATIO_MAX 2.

float v_ctl_pitch_loiter_trim;
float v_ctl_pitch_dash_trim;

inline static void h_ctl_roll_loop( void );
inline static void h_ctl_pitch_loop( void );

// Some default course gains
// H_CTL_COURSE_PGAIN needs to be define in airframe
#ifndef H_CTL_COURSE_PRE_BANK_CORRECTION
#define H_CTL_COURSE_PRE_BANK_CORRECTION 1.
#endif
#ifndef H_CTL_COURSE_DGAIN
#define H_CTL_COURSE_DGAIN 0.
#endif

// Some default roll gains
// H_CTL_ROLL_ATTITUDE_GAIN needs to be define in airframe
#ifndef H_CTL_ROLL_RATE_GAIN
#define H_CTL_ROLL_RATE_GAIN 0.
#endif
#ifndef H_CTL_ROLL_IGAIN
#define H_CTL_ROLL_IGAIN 0.
#endif
#ifndef H_CTL_ROLL_KFFA
#define H_CTL_ROLL_KFFA 0.
#endif
#ifndef H_CTL_ROLL_KFFD
#define H_CTL_ROLL_KFFD 0.
#endif

// Some default pitch gains
// H_CTL_PITCH_PGAIN needs to be define in airframe
#ifndef H_CTL_PITCH_DGAIN
#define H_CTL_PITCH_DGAIN 0.
#endif
#ifndef H_CTL_PITCH_IGAIN
#define H_CTL_PITCH_IGAIN 0.
#endif
#ifndef H_CTL_PITCH_KFFA
#define H_CTL_PITCH_KFFA 0.
#endif
#ifndef H_CTL_PITCH_KFFD
#define H_CTL_PITCH_KFFD 0.
#endif

#if DOWNLINK
#include "subsystems/datalink/telemetry.h"

static void send_calibration(void) {
  DOWNLINK_SEND_CALIBRATION(DefaultChannel, DefaultDevice,  &v_ctl_auto_throttle_sum_err, &v_ctl_auto_throttle_submode);
}

static void send_tune_roll(void) {
  DOWNLINK_SEND_TUNE_ROLL(DefaultChannel, DefaultDevice,
      &(stateGetBodyRates_f()->p), &(stateGetNedToBodyEulers_f()->phi), &h_ctl_roll_setpoint);
}

static void send_ctl_a(void) {
  DOWNLINK_SEND_H_CTL_A(DefaultChannel, DefaultDevice,
      &h_ctl_roll_sum_err, &h_ctl_ref_roll_angle, &h_ctl_pitch_sum_err, &h_ctl_ref_pitch_angle)
}
#endif

void h_ctl_init( void ) {
  h_ctl_course_setpoint = 0.;
  h_ctl_course_pre_bank = 0.;
  h_ctl_course_pre_bank_correction = H_CTL_COURSE_PRE_BANK_CORRECTION;
  h_ctl_course_pgain = H_CTL_COURSE_PGAIN;
  h_ctl_course_dgain = H_CTL_COURSE_DGAIN;
  h_ctl_roll_max_setpoint = H_CTL_ROLL_MAX_SETPOINT;

  h_ctl_disabled = FALSE;

  h_ctl_roll_setpoint = 0.;
  h_ctl_roll_attitude_gain = H_CTL_ROLL_ATTITUDE_GAIN;
  h_ctl_roll_rate_gain = H_CTL_ROLL_RATE_GAIN;
  h_ctl_roll_igain = H_CTL_ROLL_IGAIN;
  h_ctl_roll_sum_err = 0;
  h_ctl_roll_Kffa = H_CTL_ROLL_KFFA;
  h_ctl_roll_Kffd = H_CTL_ROLL_KFFD;
  h_ctl_aileron_setpoint = 0;
#ifdef H_CTL_AILERON_OF_THROTTLE
  h_ctl_aileron_of_throttle = H_CTL_AILERON_OF_THROTTLE;
#endif

  h_ctl_pitch_setpoint = 0.;
  h_ctl_pitch_loop_setpoint = 0.;
  h_ctl_pitch_pgain = H_CTL_PITCH_PGAIN;
  h_ctl_pitch_dgain = H_CTL_PITCH_DGAIN;
  h_ctl_pitch_igain = H_CTL_PITCH_IGAIN;
  h_ctl_pitch_sum_err = 0.;
  h_ctl_pitch_Kffa = H_CTL_PITCH_KFFA;
  h_ctl_pitch_Kffd = H_CTL_PITCH_KFFD;
  h_ctl_elevator_setpoint = 0;
  h_ctl_elevator_of_roll = 0; //H_CTL_ELEVATOR_OF_ROLL;
#ifdef H_CTL_PITCH_OF_ROLL
  h_ctl_pitch_of_roll = H_CTL_PITCH_OF_ROLL;
#endif

  use_airspeed_ratio = FALSE;
  airspeed_ratio2 = 1.;

#if USE_PITCH_TRIM
  v_ctl_pitch_loiter_trim = V_CTL_PITCH_LOITER_TRIM;
  v_ctl_pitch_dash_trim = V_CTL_PITCH_DASH_TRIM;
#else
  v_ctl_pitch_loiter_trim = 0.;
  v_ctl_pitch_dash_trim = 0.;
#endif

#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, "CALIBRATION", send_calibration);
  register_periodic_telemetry(DefaultPeriodic, "TUNE_ROLL", send_tune_roll);
  register_periodic_telemetry(DefaultPeriodic, "H_CTL_A", send_ctl_a);
#endif
}

/**
 * \brief
 *
 */
void h_ctl_course_loop ( void ) {
  static float last_err;

  // Ground path error
  float err = h_ctl_course_setpoint - (*stateGetHorizontalSpeedDir_f());
  NormRadAngle(err);

  float d_err = err - last_err;
  last_err = err;
  NormRadAngle(d_err);

  float speed_depend_nav = (*stateGetHorizontalSpeedNorm_f())/NOMINAL_AIRSPEED;
  Bound(speed_depend_nav, 0.66, 1.5);

  h_ctl_roll_setpoint = h_ctl_course_pre_bank_correction * h_ctl_course_pre_bank
    + h_ctl_course_pgain * speed_depend_nav * err
    + h_ctl_course_dgain * d_err;

  BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);
}

#if USE_AIRSPEED
static inline void compute_airspeed_ratio( void ) {
  if (use_airspeed_ratio) {
    // low pass airspeed
    static float airspeed = 0.;
    airspeed = ( 15*airspeed + (*stateGetAirspeed_f()) ) / 16;
    // compute ratio
    float airspeed_ratio = airspeed / NOMINAL_AIRSPEED;
    Bound(airspeed_ratio, AIRSPEED_RATIO_MIN, AIRSPEED_RATIO_MAX);
    airspeed_ratio2 = airspeed_ratio*airspeed_ratio;
  } else {
    airspeed_ratio2 = 1.;
  }
}
#endif

void h_ctl_attitude_loop ( void ) {
  if (!h_ctl_disabled) {
#if USE_AIRSPEED
    compute_airspeed_ratio();
#endif
    h_ctl_roll_loop();
    h_ctl_pitch_loop();
  }
}

#define H_CTL_REF_DT (1./60.)
#define KFFA_UPDATE 0.1
#define KFFD_UPDATE 0.05

inline static void h_ctl_roll_loop( void ) {

  static float cmd_fb = 0.;

#if USE_ANGLE_REF
  // Update reference setpoints for roll
  h_ctl_ref_roll_angle += h_ctl_ref_roll_rate * H_CTL_REF_DT;
  h_ctl_ref_roll_rate += h_ctl_ref_roll_accel * H_CTL_REF_DT;
  h_ctl_ref_roll_accel = H_CTL_REF_W*H_CTL_REF_W * (h_ctl_roll_setpoint - h_ctl_ref_roll_angle) - 2*H_CTL_REF_XI*H_CTL_REF_W * h_ctl_ref_roll_rate;
  // Saturation on references
  BoundAbs(h_ctl_ref_roll_accel, H_CTL_REF_MAX_P_DOT);
  if (h_ctl_ref_roll_rate > H_CTL_REF_MAX_P) {
    h_ctl_ref_roll_rate = H_CTL_REF_MAX_P;
    if (h_ctl_ref_roll_accel > 0.) h_ctl_ref_roll_accel = 0.;
  }
  else if (h_ctl_ref_roll_rate < - H_CTL_REF_MAX_P) {
    h_ctl_ref_roll_rate = - H_CTL_REF_MAX_P;
    if (h_ctl_ref_roll_accel < 0.) h_ctl_ref_roll_accel = 0.;
  }
#else
  h_ctl_ref_roll_angle = h_ctl_roll_setpoint;
  h_ctl_ref_roll_rate = 0.;
  h_ctl_ref_roll_accel = 0.;
#endif

#ifdef USE_KFF_UPDATE
  // update Kff gains
  h_ctl_roll_Kffa += KFFA_UPDATE * h_ctl_ref_roll_accel * cmd_fb / (H_CTL_REF_MAX_P_DOT*H_CTL_REF_MAX_P_DOT);
  h_ctl_roll_Kffd += KFFD_UPDATE * h_ctl_ref_roll_rate  * cmd_fb / (H_CTL_REF_MAX_P*H_CTL_REF_MAX_P);
#ifdef SITL
  printf("%f %f %f\n", h_ctl_roll_Kffa, h_ctl_roll_Kffd, cmd_fb);
#endif
  h_ctl_roll_Kffa = Max(h_ctl_roll_Kffa, 0);
  h_ctl_roll_Kffd = Max(h_ctl_roll_Kffd, 0);
#endif

  // Compute errors
  float err = h_ctl_ref_roll_angle - stateGetNedToBodyEulers_f()->phi;
  struct FloatRates* body_rate = stateGetBodyRates_f();
#ifdef SITL
  static float last_err = 0;
  body_rate->p = (err - last_err)/(1/60.); // FIXME should be done in ahrs sim
  last_err = err;
#endif
  float d_err = h_ctl_ref_roll_rate - body_rate->p;

  if (pprz_mode == PPRZ_MODE_MANUAL || launch == 0) {
    h_ctl_roll_sum_err = 0.;
  }
  else {
    if (h_ctl_roll_igain > 0.) {
      h_ctl_roll_sum_err += err * H_CTL_REF_DT;
      BoundAbs(h_ctl_roll_sum_err, H_CTL_ROLL_SUM_ERR_MAX / h_ctl_roll_igain);
    } else {
      h_ctl_roll_sum_err = 0.;
    }
  }

  cmd_fb = h_ctl_roll_attitude_gain * err;
  float cmd = - h_ctl_roll_Kffa * h_ctl_ref_roll_accel
    - h_ctl_roll_Kffd * h_ctl_ref_roll_rate
    - cmd_fb
    - h_ctl_roll_rate_gain * d_err
    - h_ctl_roll_igain * h_ctl_roll_sum_err
    + v_ctl_throttle_setpoint * h_ctl_aileron_of_throttle;

  cmd /= airspeed_ratio2;

  // Set aileron commands
  h_ctl_aileron_setpoint = TRIM_PPRZ(cmd);
}


#if USE_PITCH_TRIM
inline static void loiter(void) {
  float pitch_trim;

#if USE_AIRSPEED
  if (stateGetAirspeed_f() > NOMINAL_AIRSPEED) {
    pitch_trim = v_ctl_pitch_dash_trim * (airspeed_ratio2-1) / ((AIRSPEED_RATIO_MAX * AIRSPEED_RATIO_MAX) - 1);
  } else {
    pitch_trim = v_ctl_pitch_loiter_trim * (airspeed_ratio2-1) / ((AIRSPEED_RATIO_MIN * AIRSPEED_RATIO_MIN) - 1);
  }
#else
  float throttle_diff = v_ctl_throttle_setpoint / (float)MAX_PPRZ - v_ctl_auto_throttle_nominal_cruise_throttle;
  if (throttle_diff > 0) {
    float max_diff = Max(V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE - v_ctl_auto_throttle_nominal_cruise_throttle, 0.1);
    pitch_trim = throttle_diff / max_diff * v_ctl_pitch_dash_trim;
  } else {
    float max_diff = Max(v_ctl_auto_throttle_nominal_cruise_throttle - V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE, 0.1);
    pitch_trim = -throttle_diff / max_diff * v_ctl_pitch_loiter_trim;
  }
#endif

  h_ctl_pitch_loop_setpoint += pitch_trim;
}
#endif


inline static void h_ctl_pitch_loop( void ) {
#if !USE_GYRO_PITCH_RATE
  static float last_err;
#endif

  /* sanity check */
  if (h_ctl_pitch_of_roll <0.)
    h_ctl_pitch_of_roll = 0.;

  h_ctl_pitch_loop_setpoint = h_ctl_pitch_setpoint + h_ctl_pitch_of_roll * fabs(stateGetNedToBodyEulers_f()->phi);
#if USE_PITCH_TRIM
  loiter();
#endif

#if USE_ANGLE_REF
  // Update reference setpoints for pitch
  h_ctl_ref_pitch_angle += h_ctl_ref_pitch_rate * H_CTL_REF_DT;
  h_ctl_ref_pitch_rate += h_ctl_ref_pitch_accel * H_CTL_REF_DT;
  h_ctl_ref_pitch_accel = H_CTL_REF_W*H_CTL_REF_W * (h_ctl_pitch_loop_setpoint - h_ctl_ref_pitch_angle) - 2*H_CTL_REF_XI*H_CTL_REF_W * h_ctl_ref_pitch_rate;
  // Saturation on references
  BoundAbs(h_ctl_ref_pitch_accel, H_CTL_REF_MAX_Q_DOT);
  if (h_ctl_ref_pitch_rate > H_CTL_REF_MAX_Q) {
    h_ctl_ref_pitch_rate = H_CTL_REF_MAX_Q;
    if (h_ctl_ref_pitch_accel > 0.) h_ctl_ref_pitch_accel = 0.;
  }
  else if (h_ctl_ref_pitch_rate < - H_CTL_REF_MAX_Q) {
    h_ctl_ref_pitch_rate = - H_CTL_REF_MAX_Q;
    if (h_ctl_ref_pitch_accel < 0.) h_ctl_ref_pitch_accel = 0.;
  }
#else
  h_ctl_ref_pitch_angle = h_ctl_pitch_loop_setpoint;
  h_ctl_ref_pitch_rate = 0.;
  h_ctl_ref_pitch_accel = 0.;
#endif

  // Compute errors
  float err =  h_ctl_ref_pitch_angle - stateGetNedToBodyEulers_f()->theta;
#if USE_GYRO_PITCH_RATE
  float d_err = h_ctl_ref_pitch_rate - stateGetBodyRates_f()->q;
#else // soft derivation
  float d_err = (err - last_err)/H_CTL_REF_DT - h_ctl_ref_pitch_rate;
  last_err = err;
#endif

  if (pprz_mode == PPRZ_MODE_MANUAL || launch == 0) {
    h_ctl_pitch_sum_err = 0.;
  }
  else {
    if (h_ctl_pitch_igain > 0.) {
      h_ctl_pitch_sum_err += err * H_CTL_REF_DT;
      BoundAbs(h_ctl_pitch_sum_err, H_CTL_PITCH_SUM_ERR_MAX / h_ctl_pitch_igain);
    } else {
      h_ctl_pitch_sum_err = 0.;
    }
  }

  float cmd = - h_ctl_pitch_Kffa * h_ctl_ref_pitch_accel
    - h_ctl_pitch_Kffd * h_ctl_ref_pitch_rate
    + h_ctl_pitch_pgain * err
    + h_ctl_pitch_dgain * d_err
    + h_ctl_pitch_igain * h_ctl_pitch_sum_err;

  cmd /= airspeed_ratio2;

  h_ctl_elevator_setpoint = TRIM_PPRZ(cmd);
}

