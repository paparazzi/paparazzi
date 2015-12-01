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
 * Fixed wing adaptive control.
 *
 *
 * to use the attitude referece generator define
 *   <define name="USE_ANGLE_REF"/>
 *
 * to use the adaptive part for roll and/or pitch define
 *  <define name="USE_KFF_UPDATE_ROLL"/>
 *  <define name="USE_KFF_UPDATE_PITCH"/>
 *
 * to use the yaw damper
 *  <section name="AUTO1" prefix="AUTO1_">
 *    <define name="MAX_YAW_RATE" value="RadOfDeg(100)"/>
 *  </section>
 *
 *  <section name="HORIZONTAL CONTROL" prefix="H_CTL_">
 *   <define name="YAW_LOOP" value="TRUE"/>
 *   <define name="YAW_DGAIN" value="5000."/>
 *  </section>
 *
 *   in addition "ny" can be trimed to minimize the sideslip angle
 *    <section name="HORIZONTAL CONTROL" prefix="H_CTL_">
 *     <define name="YAW_TRIM_NY" value="TRUE"/>
 *     <define name="YAW_NY_IGAIN" value="5000."/>
 *  </section>
 *
 * to use the automatic flap control define
 *  <section name="HORIZONTAL CONTROL" prefix="H_CTL_">
 *   <define name="CL_LOOP" value="TRUE"/>
 *   <define name="CL_LOOP_USE_AIRSPEED_SETPOINT" value="TRUE"/>
 *   <define name="CL_FLAPS_STALL" value="0.8"/>
 *   <define name="CL_FLAPS_NOMINAL" value="0."/>
 *   <define name="CL_FLAPS_RACE" value="-0.5"/>
 *   <define name="CL_DEADBAND" value="1."/>
 *  </section>
 *
 *   the actual flap setting can also be increased by the loadfactor "nz"
 *    <section name="HORIZONTAL CONTROL" prefix="H_CTL_">
 *     <define name="CL_LOOP_INCREASE_FLAPS_WITH_LOADFACTOR" value="TRUE"/>
 *    </section>
 *
 */

#include "std.h"
#include "led.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#include "state.h"
#include "firmwares/fixedwing/nav.h"
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

struct HCtlAdaptRef {
  float roll_angle;
  float roll_rate;
  float roll_accel;
  float pitch_angle;
  float pitch_rate;
  float pitch_accel;
  float yaw_angle;
  float yaw_rate;
  float yaw_accel;

  float max_p;
  float max_p_dot;
  float max_q;
  float max_q_dot;
  float max_r;
  float max_r_dot;
};

struct HCtlAdaptRef h_ctl_ref;

/* Reference generator */
#ifndef H_CTL_REF_W_P
#define H_CTL_REF_W_P 5.
#endif
#ifndef H_CTL_REF_XI_P
#define H_CTL_REF_XI_P 0.85
#endif
#ifndef H_CTL_REF_W_Q
#define H_CTL_REF_W_Q 5.
#endif
#ifndef H_CTL_REF_XI_Q
#define H_CTL_REF_XI_Q 0.85
#endif
#ifndef H_CTL_REF_W_R
#define H_CTL_REF_W_R 5.
#endif
#ifndef H_CTL_REF_XI_R
#define H_CTL_REF_XI_R 0.85
#endif
#ifndef H_CTL_REF_MAX_P
#define H_CTL_REF_MAX_P RadOfDeg(150.)
#endif
#ifndef H_CTL_REF_MAX_P_DOT
#define H_CTL_REF_MAX_P_DOT RadOfDeg(500.)
#endif
#ifndef H_CTL_REF_MAX_Q
#define H_CTL_REF_MAX_Q RadOfDeg(150.)
#endif
#ifndef H_CTL_REF_MAX_Q_DOT
#define H_CTL_REF_MAX_Q_DOT RadOfDeg(500.)
#endif

#if USE_ANGLE_REF
PRINT_CONFIG_MSG("USING ATTITUDE REFERENCE GENERATOR")
PRINT_CONFIG_VAR(H_CTL_REF_W_P)
PRINT_CONFIG_VAR(H_CTL_REF_W_Q)
PRINT_CONFIG_VAR(H_CTL_REF_MAX_P)
PRINT_CONFIG_VAR(H_CTL_REF_MAX_P_DOT)
PRINT_CONFIG_VAR(H_CTL_REF_MAX_Q)
PRINT_CONFIG_VAR(H_CTL_REF_MAX_Q_DOT)
#endif

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

/* inner yaw loop parameters */
#if H_CTL_YAW_LOOP
float h_ctl_yaw_rate_setpoint;
float h_ctl_yaw_dgain;
float h_ctl_yaw_ny_igain;
float h_ctl_yaw_ny_sum_err;
pprz_t h_ctl_rudder_setpoint;
#endif

/* inner CL loop parameters */
#if H_CTL_CL_LOOP
pprz_t h_ctl_flaps_setpoint;
#endif

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

// Pitch trim rate limiter
#ifndef PITCH_TRIM_RATE_LIMITER
#define PITCH_TRIM_RATE_LIMITER 3.
#endif
inline static void h_ctl_roll_loop(void);
inline static void h_ctl_pitch_loop(void);
#if H_CTL_YAW_LOOP
inline static void h_ctl_yaw_loop(void);
#endif
#if H_CTL_CL_LOOP
inline static void h_ctl_cl_loop(void);
#endif

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

// H_CTL_YAW_GAINS to be defined in airframe
#ifndef H_CTL_YAW_KFFD
#define H_CTL_YAW_KFFD 0.
#endif

#ifndef USE_GYRO_PITCH_RATE
#define USE_GYRO_PITCH_RATE TRUE
#endif

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_calibration(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_CALIBRATION(trans, dev, AC_ID,  &v_ctl_auto_throttle_sum_err, &v_ctl_auto_throttle_submode);
}

static void send_tune_roll(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_TUNE_ROLL(trans, dev, AC_ID,
                          &(stateGetBodyRates_f()->p), &(stateGetNedToBodyEulers_f()->phi), &h_ctl_roll_setpoint);
}

static void send_ctl_a(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_H_CTL_A(trans, dev, AC_ID,
                        &h_ctl_roll_sum_err,
                        &h_ctl_roll_setpoint,
                        &h_ctl_ref.roll_angle,
                        &(stateGetNedToBodyEulers_f()->phi),
                        &h_ctl_aileron_setpoint,
                        &h_ctl_pitch_sum_err,
                        &h_ctl_pitch_loop_setpoint,
                        &h_ctl_ref.pitch_angle,
                        &(stateGetNedToBodyEulers_f()->theta),
                        &h_ctl_elevator_setpoint);
}
#endif

void h_ctl_init(void)
{
  h_ctl_ref.max_p = H_CTL_REF_MAX_P;
  h_ctl_ref.max_p_dot = H_CTL_REF_MAX_P_DOT;
  h_ctl_ref.max_q = H_CTL_REF_MAX_Q;
  h_ctl_ref.max_q_dot = H_CTL_REF_MAX_Q_DOT;

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

#if H_CTL_YAW_LOOP
  h_ctl_yaw_dgain = H_CTL_YAW_DGAIN;
  h_ctl_yaw_ny_igain = H_CTL_YAW_NY_IGAIN;
  h_ctl_yaw_ny_sum_err = 0.;
  h_ctl_rudder_setpoint = 0;
#endif

#if H_CTL_CL_LOOP
  h_ctl_flaps_setpoint = 0;
#endif

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

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CALIBRATION, send_calibration);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_TUNE_ROLL, send_tune_roll);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_H_CTL_A, send_ctl_a);
#endif
}

/**
 * \brief
 *
 */
void h_ctl_course_loop(void)
{
  static float last_err;

  // Ground path error
  float err = h_ctl_course_setpoint - stateGetHorizontalSpeedDir_f();
  NormRadAngle(err);

  float d_err = err - last_err;
  last_err = err;
  NormRadAngle(d_err);

  float speed_depend_nav = stateGetHorizontalSpeedNorm_f() / NOMINAL_AIRSPEED;
  Bound(speed_depend_nav, 0.66, 1.5);

  h_ctl_roll_setpoint = h_ctl_course_pre_bank_correction * h_ctl_course_pre_bank
                        + h_ctl_course_pgain * speed_depend_nav * err
                        + h_ctl_course_dgain * d_err;

  BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);
}

#if USE_AIRSPEED
static inline void compute_airspeed_ratio(void)
{
  if (use_airspeed_ratio) {
    // low pass airspeed
    static float airspeed = 0.;
    airspeed = (15 * airspeed + stateGetAirspeed_f()) / 16;
    // compute ratio
    float airspeed_ratio = airspeed / NOMINAL_AIRSPEED;
    Bound(airspeed_ratio, AIRSPEED_RATIO_MIN, AIRSPEED_RATIO_MAX);
    airspeed_ratio2 = airspeed_ratio * airspeed_ratio;
  } else {
    airspeed_ratio2 = 1.;
  }
}
#endif

void h_ctl_attitude_loop(void)
{
  if (!h_ctl_disabled) {
#if USE_AIRSPEED
    compute_airspeed_ratio();
#endif
    h_ctl_roll_loop();
    h_ctl_pitch_loop();
#if H_CTL_YAW_LOOP
    h_ctl_yaw_loop();
#endif
#if H_CTL_CL_LOOP
    h_ctl_cl_loop();
#endif
  }
}

/** Define reference generator time step
 *  default to control frequency
 *  and ahrs propagation freq if control is triggered by ahrs
 */
#ifdef AHRS_TRIGGERED_ATTITUDE_LOOP
#define H_CTL_REF_DT (1./AHRS_PROPAGATE_FREQUENCY)
#else
#define H_CTL_REF_DT (1./CONTROL_FREQUENCY)
#endif

/** Adaptive control tuning parameters
 *  activate with USE_KFF_UPDATE
 *
 *  !!! under development, use with care !!!
 */
#define KFFA_UPDATE 0.1
#define KFFD_UPDATE 0.05

inline static void h_ctl_roll_loop(void)
{

  static float cmd_fb = 0.;

#if USE_ANGLE_REF
  // Update reference setpoints for roll
  h_ctl_ref.roll_angle += h_ctl_ref.roll_rate * H_CTL_REF_DT;
  h_ctl_ref.roll_rate += h_ctl_ref.roll_accel * H_CTL_REF_DT;
  h_ctl_ref.roll_accel = H_CTL_REF_W_P * H_CTL_REF_W_P * (h_ctl_roll_setpoint - h_ctl_ref.roll_angle) - 2 * H_CTL_REF_XI_P
                         * H_CTL_REF_W_P * h_ctl_ref.roll_rate;
  // Saturation on references
  BoundAbs(h_ctl_ref.roll_accel, h_ctl_ref.max_p_dot);
  if (h_ctl_ref.roll_rate > h_ctl_ref.max_p) {
    h_ctl_ref.roll_rate = h_ctl_ref.max_p;
    if (h_ctl_ref.roll_accel > 0.) { h_ctl_ref.roll_accel = 0.; }
  } else if (h_ctl_ref.roll_rate < - h_ctl_ref.max_p) {
    h_ctl_ref.roll_rate = - h_ctl_ref.max_p;
    if (h_ctl_ref.roll_accel < 0.) { h_ctl_ref.roll_accel = 0.; }
  }
#else
  h_ctl_ref.roll_angle = h_ctl_roll_setpoint;
  h_ctl_ref.roll_rate = 0.;
  h_ctl_ref.roll_accel = 0.;
#endif

#ifdef USE_KFF_UPDATE_ROLL
  // update Kff gains
  h_ctl_roll_Kffa += KFFA_UPDATE * h_ctl_ref.roll_accel * cmd_fb / (h_ctl_ref.max_p_dot * h_ctl_ref.max_p_dot);
  h_ctl_roll_Kffd += KFFD_UPDATE * h_ctl_ref.roll_rate  * cmd_fb / (h_ctl_ref.max_p * h_ctl_ref.max_p);
#ifdef SITL
  printf("%f %f %f\n", h_ctl_roll_Kffa, h_ctl_roll_Kffd, cmd_fb);
#endif
  h_ctl_roll_Kffa = Max(h_ctl_roll_Kffa, 0);
  h_ctl_roll_Kffd = Max(h_ctl_roll_Kffd, 0);
#endif

  // Compute errors
  float err = h_ctl_ref.roll_angle - stateGetNedToBodyEulers_f()->phi;
  struct FloatRates *body_rate = stateGetBodyRates_f();
  float d_err = h_ctl_ref.roll_rate - body_rate->p;

  if (pprz_mode == PPRZ_MODE_MANUAL || launch == 0) {
    h_ctl_roll_sum_err = 0.;
  } else {
    if (h_ctl_roll_igain > 0.) {
      h_ctl_roll_sum_err += err * H_CTL_REF_DT;
      BoundAbs(h_ctl_roll_sum_err, H_CTL_ROLL_SUM_ERR_MAX / h_ctl_roll_igain);
    } else {
      h_ctl_roll_sum_err = 0.;
    }
  }

  cmd_fb = h_ctl_roll_attitude_gain * err;
  float cmd = - h_ctl_roll_Kffa * h_ctl_ref.roll_accel
              - h_ctl_roll_Kffd * h_ctl_ref.roll_rate
              - cmd_fb
              - h_ctl_roll_rate_gain * d_err
              - h_ctl_roll_igain * h_ctl_roll_sum_err
              + v_ctl_throttle_setpoint * h_ctl_aileron_of_throttle;

  cmd /= airspeed_ratio2;

  // Set aileron commands
  h_ctl_aileron_setpoint = TRIM_PPRZ(cmd);
}


#if USE_PITCH_TRIM
inline static void loiter(void)
{
  float pitch_trim;
  static float last_pitch_trim;

#if USE_AIRSPEED
  if (stateGetAirspeed_f() > NOMINAL_AIRSPEED) {
    pitch_trim = v_ctl_pitch_dash_trim * (airspeed_ratio2 - 1) / ((AIRSPEED_RATIO_MAX * AIRSPEED_RATIO_MAX) - 1);
  } else {
    pitch_trim = v_ctl_pitch_loiter_trim * (airspeed_ratio2 - 1) / ((AIRSPEED_RATIO_MIN * AIRSPEED_RATIO_MIN) - 1);
  }
#else
  float throttle_diff = v_ctl_auto_throttle_cruise_throttle - v_ctl_auto_throttle_nominal_cruise_throttle;
  if (throttle_diff > 0) {
    float max_diff = Max(v_ctl_auto_throttle_max_cruise_throttle - v_ctl_auto_throttle_nominal_cruise_throttle, 0.1);
    pitch_trim = throttle_diff / max_diff * v_ctl_pitch_dash_trim;
  } else {
    float max_diff = Max(v_ctl_auto_throttle_nominal_cruise_throttle - v_ctl_auto_throttle_min_cruise_throttle, 0.1);
    pitch_trim = -throttle_diff / max_diff * v_ctl_pitch_loiter_trim;
  }
#endif

  // Pitch trim rate limiter
  float max_change = (v_ctl_pitch_loiter_trim - v_ctl_pitch_dash_trim) * H_CTL_REF_DT / PITCH_TRIM_RATE_LIMITER;
  Bound(pitch_trim, last_pitch_trim - max_change, last_pitch_trim + max_change);

  last_pitch_trim = pitch_trim;
  h_ctl_pitch_loop_setpoint += pitch_trim;
}
#endif


inline static void h_ctl_pitch_loop(void)
{
  static float cmd_fb = 0.;
#if !USE_GYRO_PITCH_RATE
  static float last_err;
#endif

  /* sanity check */
  if (h_ctl_pitch_of_roll < 0.) {
    h_ctl_pitch_of_roll = 0.;
  }

  h_ctl_pitch_loop_setpoint = h_ctl_pitch_setpoint + h_ctl_pitch_of_roll * fabsf(stateGetNedToBodyEulers_f()->phi);
#if USE_PITCH_TRIM
  loiter();
#endif

#if USE_ANGLE_REF
  // Update reference setpoints for pitch
  h_ctl_ref.pitch_angle += h_ctl_ref.pitch_rate * H_CTL_REF_DT;
  h_ctl_ref.pitch_rate += h_ctl_ref.pitch_accel * H_CTL_REF_DT;
  h_ctl_ref.pitch_accel = H_CTL_REF_W_Q * H_CTL_REF_W_Q * (h_ctl_pitch_loop_setpoint - h_ctl_ref.pitch_angle) - 2 *
                          H_CTL_REF_XI_Q * H_CTL_REF_W_Q * h_ctl_ref.pitch_rate;
  // Saturation on references
  BoundAbs(h_ctl_ref.pitch_accel, h_ctl_ref.max_q_dot);
  if (h_ctl_ref.pitch_rate > h_ctl_ref.max_q) {
    h_ctl_ref.pitch_rate = h_ctl_ref.max_q;
    if (h_ctl_ref.pitch_accel > 0.) { h_ctl_ref.pitch_accel = 0.; }
  } else if (h_ctl_ref.pitch_rate < - h_ctl_ref.max_q) {
    h_ctl_ref.pitch_rate = - h_ctl_ref.max_q;
    if (h_ctl_ref.pitch_accel < 0.) { h_ctl_ref.pitch_accel = 0.; }
  }
#else
  h_ctl_ref.pitch_angle = h_ctl_pitch_loop_setpoint;
  h_ctl_ref.pitch_rate = 0.;
  h_ctl_ref.pitch_accel = 0.;
#endif

#ifdef USE_KFF_UPDATE_PITCH
  // update Kff gains
  h_ctl_pitch_Kffa += KFFA_UPDATE * h_ctl_ref.pitch_accel * cmd_fb / (h_ctl_ref.max_q_dot * h_ctl_ref.max_q_dot);
  h_ctl_pitch_Kffd += KFFD_UPDATE * h_ctl_ref.pitch_rate  * cmd_fb / (h_ctl_ref.max_q * h_ctl_ref.max_q);
#ifdef SITL
  printf("%f %f %f\n", h_ctl_pitch_Kffa, h_ctl_pitch_Kffd, cmd_fb);
#endif
  h_ctl_pitch_Kffa = Max(h_ctl_pitch_Kffa, 0);
  h_ctl_pitch_Kffd = Max(h_ctl_pitch_Kffd, 0);
#endif

  // Compute errors
  float err =  h_ctl_ref.pitch_angle - stateGetNedToBodyEulers_f()->theta;
#if USE_GYRO_PITCH_RATE
  float d_err = h_ctl_ref.pitch_rate - stateGetBodyRates_f()->q;
#else // soft derivation
  float d_err = (err - last_err) / H_CTL_REF_DT - h_ctl_ref.pitch_rate;
  last_err = err;
#endif

  if (pprz_mode == PPRZ_MODE_MANUAL || launch == 0) {
    h_ctl_pitch_sum_err = 0.;
  } else {
    if (h_ctl_pitch_igain > 0.) {
      h_ctl_pitch_sum_err += err * H_CTL_REF_DT;
      BoundAbs(h_ctl_pitch_sum_err, H_CTL_PITCH_SUM_ERR_MAX / h_ctl_pitch_igain);
    } else {
      h_ctl_pitch_sum_err = 0.;
    }
  }

  cmd_fb = h_ctl_pitch_pgain * err;
  float cmd = - h_ctl_pitch_Kffa * h_ctl_ref.pitch_accel
              - h_ctl_pitch_Kffd * h_ctl_ref.pitch_rate
              + cmd_fb
              + h_ctl_pitch_dgain * d_err
              + h_ctl_pitch_igain * h_ctl_pitch_sum_err;

  cmd /= airspeed_ratio2;

  h_ctl_elevator_setpoint = TRIM_PPRZ(cmd);
}

/* yaw damper */
#if H_CTL_YAW_LOOP
inline static void h_ctl_yaw_loop(void)
{

#if H_CTL_YAW_TRIM_NY
  // Actual Acceleration from IMU:
#if (!defined SITL || defined USE_NPS)
  struct Int32Vect3 accel_meas_body, accel_ned;
  struct Int32RMat *ned_to_body_rmat = stateGetNedToBodyRMat_i();
  struct NedCoor_i *accel_tmp = stateGetAccelNed_i();
  VECT3_COPY(accel_ned, (*accel_tmp));
  accel_ned.z -= ACCEL_BFP_OF_REAL(9.81f);
  int32_rmat_vmult(&accel_meas_body, ned_to_body_rmat, &accel_ned);
  float ny = -ACCEL_FLOAT_OF_BFP(accel_meas_body.y) / 9.81f; // Lateral load factor (in g)
#else
  float ny = 0.f;
#endif

  if (pprz_mode == PPRZ_MODE_MANUAL || launch == 0) {
    h_ctl_yaw_ny_sum_err = 0.;
  } else {
    if (h_ctl_yaw_ny_igain > 0.) {
      // only update when: phi<60degrees and ny<2g
      if (fabsf(stateGetNedToBodyEulers_f()->phi) < 1.05 && fabsf(ny) < 2.) {
        h_ctl_yaw_ny_sum_err += ny * H_CTL_REF_DT;
        // max half rudder deflection for trim
        BoundAbs(h_ctl_yaw_ny_sum_err, MAX_PPRZ / (2. * h_ctl_yaw_ny_igain));
      }
    } else {
      h_ctl_yaw_ny_sum_err = 0.;
    }
  }
#endif

#ifdef USE_AIRSPEED
  float Vo = stateGetAirspeed_f();
  Bound(Vo, STALL_AIRSPEED, RACE_AIRSPEED);
#else
  float Vo = NOMINAL_AIRSPEED;
#endif

  h_ctl_ref.yaw_rate = h_ctl_yaw_rate_setpoint // set by RC
                       + 9.81f / Vo * sinf(h_ctl_roll_setpoint); // for turns
  float d_err = h_ctl_ref.yaw_rate - stateGetBodyRates_f()->r;

  float cmd = + h_ctl_yaw_dgain * d_err
#if H_CTL_YAW_TRIM_NY
              + h_ctl_yaw_ny_igain * h_ctl_yaw_ny_sum_err
#endif
              ;
  cmd /= airspeed_ratio2;
  h_ctl_rudder_setpoint = TRIM_PPRZ(cmd);
}
#endif

/* CL with Flaps - direct lift control */
#if H_CTL_CL_LOOP
inline static void h_ctl_cl_loop(void)
{

#if H_CTL_CL_LOOP_INCREASE_FLAPS_WITH_LOADFACTOR
#if (!defined SITL || defined USE_NPS)
  struct Int32Vect3 accel_meas_body, accel_ned;
  struct Int32RMat *ned_to_body_rmat = stateGetNedToBodyRMat_i();
  struct NedCoor_i *accel_tmp = stateGetAccelNed_i();
  VECT3_COPY(accel_ned, (*accel_tmp));
  accel_ned.z -= ACCEL_BFP_OF_REAL(9.81f);
  int32_rmat_vmult(&accel_meas_body, ned_to_body_rmat, &accel_ned);
  float nz = -ACCEL_FLOAT_OF_BFP(accel_meas_body.z) / 9.81f;
  // max load factor to be taken into acount
  // to prevent negative flap movement du to negative acceleration
  Bound(nz, 1.f, 2.f);
#else
  float nz = 1.f;
#endif
#endif

  // Compute a corrected airspeed corresponding to the current load factor nz
  // with Cz the lift coef at 1g, Czn the lift coef at n g, both at the same speed V,
  // the corrected airspeed Vn is so that nz = Czn/Cz = V^2 / Vn^2,
  // thus Vn = V / sqrt(nz)
#if H_CTL_CL_LOOP_USE_AIRSPEED_SETPOINT
  float corrected_airspeed = v_ctl_auto_airspeed_setpoint;
#else
  float corrected_airspeed = stateGetAirspeed_f();
#endif
#if H_CTL_CL_LOOP_INCREASE_FLAPS_WITH_LOADFACTOR
  corrected_airspeed /= sqrtf(nz);
#endif
  Bound(corrected_airspeed, STALL_AIRSPEED, RACE_AIRSPEED);

  float cmd = 0.f;
  // deadband around NOMINAL_AIRSPEED, rest linear
  if (corrected_airspeed > NOMINAL_AIRSPEED + H_CTL_CL_DEADBAND) {
    cmd = (corrected_airspeed - NOMINAL_AIRSPEED) * (H_CTL_CL_FLAPS_RACE - H_CTL_CL_FLAPS_NOMINAL) / (RACE_AIRSPEED - NOMINAL_AIRSPEED);
  }
  else if (corrected_airspeed < NOMINAL_AIRSPEED - H_CTL_CL_DEADBAND) {
    cmd = (corrected_airspeed - NOMINAL_AIRSPEED) * (H_CTL_CL_FLAPS_STALL - H_CTL_CL_FLAPS_NOMINAL) / (STALL_AIRSPEED - NOMINAL_AIRSPEED);
  }

  // no control in manual mode
  if (pprz_mode == PPRZ_MODE_MANUAL) {
    cmd = 0.f;
  }
  // bound max flap angle
  Bound(cmd, H_CTL_CL_FLAPS_RACE, H_CTL_CL_FLAPS_STALL);
  // from percent to pprz
  cmd = cmd * MAX_PPRZ;
  h_ctl_flaps_setpoint = TRIM_PPRZ(cmd);
}
#endif

