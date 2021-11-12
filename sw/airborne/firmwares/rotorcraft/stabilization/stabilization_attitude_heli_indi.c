/*
 * Copyright (c) 2016 Bart Slinger <bartslinger@gmail.com>
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

/** @file stabilization_attitude_heli_indi.c
 * Helicopter quaternion INDI attitude stabilization
 */

#include "paparazzi.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_simple_matrix.h"
#include "state.h"
#include "generated/airframe.h"
#include "autopilot.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_heli_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"
#include "filters/low_pass_filter.h"

#ifndef STABILIZATION_ATTITUDE_HELI_INDI_USE_NOTCHFILTER
#define STABILIZATION_ATTITUDE_HELI_INDI_USE_NOTCHFILTER 0
#endif
#if STABILIZATION_ATTITUDE_HELI_INDI_USE_NOTCHFILTER
#ifndef RPM_PWM_CHANNEL
#error notch filter requires module rpm_sensor.xml
#endif
#include "modules/sensors/rpm_sensor.h"
#endif // USE_NOTCHFILTER

/* Setup of all default values, these are configured for walkera genius cp v2 */
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_STEADY_STATE_ROLL
#define STABILIZATION_ATTITUDE_HELI_INDI_STEADY_STATE_ROLL 4.5
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_STEADY_STATE_PITCH
#define STABILIZATION_ATTITUDE_HELI_INDI_STEADY_STATE_PITCH 0
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_ROLL_P
#define STABILIZATION_ATTITUDE_HELI_INDI_ROLL_P 12
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_PITCH_P
#define STABILIZATION_ATTITUDE_HELI_INDI_PITCH_P 8
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_YAW_P
#define STABILIZATION_ATTITUDE_HELI_INDI_YAW_P 10
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_YAW_D
#define STABILIZATION_ATTITUDE_HELI_INDI_YAW_D 30
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_GINV_ROLL_TO_ROLL
#define STABILIZATION_ATTITUDE_HELI_INDI_GINV_ROLL_TO_ROLL 11681
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_GINV_PITCH_TO_PITCH
#define STABILIZATION_ATTITUDE_HELI_INDI_GINV_PITCH_TO_PITCH 13873
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_GINV_YAW_TO_YAW
#define STABILIZATION_ATTITUDE_HELI_INDI_GINV_YAW_TO_YAW 730
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_ROLL_COMMAND_ROTATION
#define STABILIZATION_ATTITUDE_HELI_INDI_ROLL_COMMAND_ROTATION 11.0
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_PITCH_COMMAND_ROTATION
#define STABILIZATION_ATTITUDE_HELI_INDI_PITCH_COMMAND_ROTATION -30.0
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_DEFAULT
#define STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_DEFAULT 10.0
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_ROLL
#define STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_ROLL STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_DEFAULT
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_PITCH
#define STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_PITCH STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_DEFAULT
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_YAW
#define STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_YAW STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_DEFAULT
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_THRUST
#define STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_THRUST STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_DEFAULT
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_DEFAULT
#define STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_DEFAULT 40.0
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_ROLL
#define STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_ROLL STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_DEFAULT
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_PITCH
#define STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_PITCH STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_DEFAULT
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_YAW
#define STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_YAW STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_DEFAULT
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_THRUST
#define STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_THRUST STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_DEFAULT
#endif
#ifndef STABILIZATION_ATTITUDE_HELI_INDI_NOTCH_MIN_RPM
#define STABILIZATION_ATTITUDE_HELI_INDI_NOTCH_MIN_RPM 1500
#endif
#define INDI_NOTCH_MIN_RPM STABILIZATION_ATTITUDE_HELI_INDI_NOTCH_MIN_RPM

/* Shorter defines to use lateron in the matrix */
#define INVG_00 STABILIZATION_ATTITUDE_HELI_INDI_GINV_ROLL_TO_ROLL
#define INVG_11 STABILIZATION_ATTITUDE_HELI_INDI_GINV_PITCH_TO_PITCH
#define INVG_22 STABILIZATION_ATTITUDE_HELI_INDI_GINV_YAW_TO_YAW
#define INVG_33 -50000 // Not used at the moment
#define INT32_INVG_FRAC 16

struct Int32Quat   stab_att_sp_quat;
struct Int32Eulers stab_att_sp_euler;
struct Int32Quat sp_offset; // non-zero neutral attitude

struct HeliIndiGains heli_indi_gains = {
  STABILIZATION_ATTITUDE_HELI_INDI_ROLL_P,
  STABILIZATION_ATTITUDE_HELI_INDI_PITCH_P,
  STABILIZATION_ATTITUDE_HELI_INDI_YAW_P,
  STABILIZATION_ATTITUDE_HELI_INDI_YAW_D
};

/* Main controller struct */
struct IndiController_int heli_indi_ctl;

/* Filter functions */
struct delayed_first_order_lowpass_filter_t actuator_model[INDI_DOF];
int32_t alpha_yaw_inc; // Tail model parameters for spinning up
int32_t alpha_yaw_dec; // Tail model parameters for spinning down
/* Measurement filters */
Butterworth2LowPass_int actuator_lowpass_filters[INDI_DOF];
Butterworth2LowPass_int measurement_lowpass_filters[INDI_DOF];
struct SecondOrderNotchFilter actuator_notchfilter[INDI_DOF];
struct SecondOrderNotchFilter measurement_notchfilter[INDI_DOF];
#if STABILIZATION_ATTITUDE_HELI_INDI_USE_FAST_DYN_FILTERS
struct delayed_first_order_lowpass_filter_t fast_dynamics_model[2]; // only pitch and roll
#endif

/* Private functions declarations */
void indi_apply_actuator_notch_filters(int32_t _out[], int32_t _in[]);
void indi_apply_measurement_notch_filters(int32_t _out[], int32_t _in[]);
void indi_apply_actuator_butterworth_filters(int32_t _out[], int32_t _in[]);
void indi_apply_measurement_butterworth_filters(int32_t _out[], int32_t _in[]);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

/* Telemetry messages here, at the moment there are none */

#endif // PERIODIC_TELEMETRY

static inline void indi_apply_actuator_models(int32_t _out[], int32_t _in[])
{
  int32_t temp_roll;
  int32_t temp_pitch;
  temp_roll  = delayed_first_order_lowpass_propagate(&actuator_model[INDI_ROLL], _in[INDI_ROLL]);
  temp_pitch = delayed_first_order_lowpass_propagate(&actuator_model[INDI_PITCH], _in[INDI_PITCH]);

  /* Depending on yaw direction, change filter coefficient */
  int32_t prev = actuator_model[INDI_YAW].buffer[actuator_model[INDI_YAW].idx];
  if (_in[INDI_YAW] - prev > 0) {
    // Tail spinning up
    actuator_model[INDI_YAW].alpha = alpha_yaw_inc;
  } else {
    // Tail spinning down
    actuator_model[INDI_YAW].alpha = alpha_yaw_dec;
  }
  _out[INDI_YAW] = delayed_first_order_lowpass_propagate(&actuator_model[INDI_YAW], _in[INDI_YAW]);

  _out[INDI_THRUST] = delayed_first_order_lowpass_propagate(&actuator_model[INDI_THRUST], _in[INDI_THRUST]);

#if STABILIZATION_ATTITUDE_HELI_INDI_USE_FAST_DYN_FILTERS
  /* Also apply first order filter that represents fast damping dynamics in pitch and roll rate */
  _out[INDI_ROLL]  = delayed_first_order_lowpass_propagate(&fast_dynamics_model[INDI_ROLL], temp_roll);
  _out[INDI_PITCH] = delayed_first_order_lowpass_propagate(&fast_dynamics_model[INDI_PITCH], temp_pitch);
#else
  _out[INDI_ROLL] = temp_roll;
  _out[INDI_PITCH] = temp_pitch;
#endif
}

/**
 * The main idea of this function is to slow down a certain actuator, so that
 * the actuator dynamics filtered by the compensater are equal to another
 * slower actuator. In this case, the tail rotor is slower than collective
 * pitch. The collective pitch signal can be reduced so that the total dynamics
 * match the dynamics of the tail rotor. This prevents the tail from
 * breaking out. But at the moment thrust is not implemented so this is not
 * used.
 */
static inline void indi_apply_compensator_filters(int32_t _out[], int32_t _in[])
{
  _out[INDI_ROLL]   = _in[INDI_ROLL];
  _out[INDI_PITCH]  = _in[INDI_PITCH];

  /* Delay the tail by 9 samples */
  static int32_t yaw_output_buffer[INDI_YAW_BUFFER_SIZE];
  static uint8_t buf_idx = 0;

  buf_idx %= (INDI_YAW_BUFFER_SIZE - 1);
  _out[INDI_YAW] = yaw_output_buffer[buf_idx];
  yaw_output_buffer[buf_idx] = _in[INDI_YAW];
  buf_idx++;

  // Disregard, just use input:
  _out[INDI_YAW] = _in[INDI_YAW];

  /* Thrust compensated for slow tail dynamics:
   * Step 1. What would be the next output of the system if the thrust cmd would be applied to the tail dynamics?
   * Step 2. What input is required to obtain this output when assuming dynamics of thrust actuator?
   *
   * We can re-use alpha_yaw_dec and alpha_yaw_inc
   */
  static int32_t prev_thrust_out = 0;
  int32_t alpha;
  if (_in[INDI_THRUST] - prev_thrust_out > 0) {
    alpha = alpha_yaw_inc;
  } else {
    alpha = alpha_yaw_dec;
  }
  int32_t output_target = (alpha * prev_thrust_out + ((1 << DELAYED_FIRST_ORDER_LOWPASS_FILTER_FILTER_ALPHA_SHIFT) -
                           alpha) * _in[INDI_THRUST]) >> DELAYED_FIRST_ORDER_LOWPASS_FILTER_FILTER_ALPHA_SHIFT;

  /* Now the target output is known, the collective dynamics is known. What input is required? */
  int32_t alpha_thrust = actuator_model[INDI_THRUST].alpha;
  _out[INDI_THRUST] = ((output_target << DELAYED_FIRST_ORDER_LOWPASS_FILTER_FILTER_ALPHA_SHIFT) - alpha_thrust *
                       prev_thrust_out) / ((1 << DELAYED_FIRST_ORDER_LOWPASS_FILTER_FILTER_ALPHA_SHIFT) - alpha_thrust);
  prev_thrust_out = _out[INDI_THRUST];

  /* At the moment, thrust is not fully implemented */
  //_out[INDI_THRUST] = _in[INDI_THRUST];
}

static inline void indi_apply_notch_filters(struct SecondOrderNotchFilter *filter, int32_t _out[], int32_t _in[])
{
  if (heli_indi_ctl.motor_rpm > INDI_NOTCH_MIN_RPM && heli_indi_ctl.enable_notch) {
    notch_filter_set_filter_frequency(&filter[INDI_ROLL], heli_indi_ctl.motor_rpm / 60.0f);
    notch_filter_set_filter_frequency(&filter[INDI_PITCH], heli_indi_ctl.motor_rpm / 60.0f);
    notch_filter_set_filter_frequency(&filter[INDI_YAW], heli_indi_ctl.motor_rpm / 60.0f);
    notch_filter_set_filter_frequency(&filter[INDI_THRUST], heli_indi_ctl.motor_rpm / 60.0f);
    notch_filter_update(&filter[INDI_ROLL], &_in[INDI_ROLL], &_out[INDI_ROLL]);
    notch_filter_update(&filter[INDI_PITCH], &_in[INDI_PITCH], &_out[INDI_PITCH]);
    notch_filter_update(&filter[INDI_YAW], &_in[INDI_YAW], &_out[INDI_YAW]);
    notch_filter_update(&filter[INDI_THRUST], &_in[INDI_THRUST], &_out[INDI_THRUST]);
  } else {
    _out[INDI_ROLL]   = _in[INDI_ROLL];
    _out[INDI_PITCH]  = _in[INDI_PITCH];
    _out[INDI_YAW]    = _in[INDI_YAW];
    _out[INDI_THRUST] = _in[INDI_THRUST];
  }
}

void indi_apply_actuator_notch_filters(int32_t _out[], int32_t _in[])
{
  indi_apply_notch_filters(actuator_notchfilter, _out, _in);
}

void indi_apply_measurement_notch_filters(int32_t _out[], int32_t _in[])
{
  indi_apply_notch_filters(measurement_notchfilter, _out, _in);
}

void indi_apply_actuator_butterworth_filters(int32_t _out[], int32_t _in[])
{
  for (uint8_t i = 0; i < INDI_DOF; i++) {
    _out[i] = update_butterworth_2_low_pass_int(&actuator_lowpass_filters[i], _in[i]);
  }
}

void indi_apply_measurement_butterworth_filters(int32_t _out[], int32_t _in[])
{
  for (uint8_t i = 0; i < INDI_DOF; i++) {
    _out[i] = update_butterworth_2_low_pass_int(&measurement_lowpass_filters[i], _in[i]);
  }
}

/**
 * @brief stabilization_attitude_heli_indi_set_steadystate_pitch
 * @param pitch neutral pitch angle [deg].
 *
 * Change the neutral pitch angle.
 */
void stabilization_attitude_heli_indi_set_steadystate_pitch(float pitch)
{
  heli_indi_ctl.sp_offset_pitch = pitch;
  stabilization_attitude_heli_indi_set_steadystate_pitchroll();
}

/**
 * @brief stabilization_attitude_heli_indi_set_steadystate_roll
 * @param roll neutral roll angle [deg].
 *
 * Change the neutral roll angle. Especially useful for helicopters,
 * since they need a small roll angle in hover to compensate the tail force.
 */
void stabilization_attitude_heli_indi_set_steadystate_roll(float roll)
{
  heli_indi_ctl.sp_offset_roll = roll;
  stabilization_attitude_heli_indi_set_steadystate_pitchroll();
}

/**
 * @brief stabilization_attitude_heli_indi_set_steadystate_pitchroll
 *
 * Updates the neutral pitch and roll angles and calculates the
 * compensation quaternion.
 */
void stabilization_attitude_heli_indi_set_steadystate_pitchroll()
{
  /* Pitch roll setpoint not zero, because then helicopter drifts sideways */
  /* orientation vector describing simultaneous rotation of roll/pitch */
  struct FloatVect3 ov;
  struct FloatQuat q;
  ov.x = -heli_indi_ctl.sp_offset_roll * M_PI / 180;
  ov.y = -heli_indi_ctl.sp_offset_pitch * M_PI / 180;
  ov.z = 0.0;
  /* quaternion from that orientation vector */
  float_quat_of_orientation_vect(&q, &ov);
  QUAT_BFP_OF_REAL(sp_offset, q);
}

/**
 * @brief stabilization_attitude_init
 *
 * Initialize the heli indi attitude controller.
 */
void stabilization_attitude_init(void)
{
  /* Initialization code INDI */
  struct IndiController_int *c = &heli_indi_ctl;
  c->roll_comp_angle = ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_HELI_INDI_PITCH_COMMAND_ROTATION * M_PI / 180.0);
  c->pitch_comp_angle = ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_HELI_INDI_ROLL_COMMAND_ROTATION * M_PI / 180.0);
#if STABILIZATION_ATTITUDE_HELI_INDI_USE_NOTCHFILTER
  c->enable_notch = TRUE;
#else
  c->enable_notch = FALSE;
#endif
  c->motor_rpm = 0;

  /* Set steady-state pitch and roll values */
  c->sp_offset_roll = STABILIZATION_ATTITUDE_HELI_INDI_STEADY_STATE_ROLL;
  c->sp_offset_pitch = STABILIZATION_ATTITUDE_HELI_INDI_STEADY_STATE_PITCH;
  stabilization_attitude_heli_indi_set_steadystate_pitchroll();

  /* Initialize inv(G) */
  c->invG[0][0] = INVG_00; c->invG[0][1] =       0; c->invG[0][2] =       0; c->invG[0][3] =       0;
  c->invG[1][0] =       0; c->invG[1][1] = INVG_11; c->invG[1][2] =       0; c->invG[1][3] =       0;
  c->invG[2][0] =       0; c->invG[2][1] =       0; c->invG[2][2] = INVG_22; c->invG[2][3] =       0;
  c->invG[3][0] =       0; c->invG[3][1] =       0; c->invG[3][2] =       0; c->invG[3][3] = INVG_33;

  /* Actuator filter initialization */
  delayed_first_order_lowpass_initialize(&actuator_model[INDI_ROLL], 70, 9, 900, PERIODIC_FREQUENCY);
  delayed_first_order_lowpass_initialize(&actuator_model[INDI_PITCH], 70, 9, 900, PERIODIC_FREQUENCY);
  delayed_first_order_lowpass_initialize(&actuator_model[INDI_YAW], 37, 0, 9600, PERIODIC_FREQUENCY);
  delayed_first_order_lowpass_initialize(&actuator_model[INDI_THRUST], 70, 9, 900 / 2,
                                         PERIODIC_FREQUENCY); /* 450 because dynamic range is only 0-9600 */
  /* Different dynamics for up and down */
  alpha_yaw_inc = actuator_model[INDI_YAW].alpha;
  alpha_yaw_dec = (PERIODIC_FREQUENCY << DELAYED_FIRST_ORDER_LOWPASS_FILTER_FILTER_ALPHA_SHIFT) /
                  (PERIODIC_FREQUENCY + 13); // OMEGA_DOWN = 13 rad/s, shift = 14

  /* Notch filter initialization, bandwidth in Hz */
  notch_filter_init(&actuator_notchfilter[INDI_ROLL], 0.0, STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_ROLL,
                    PERIODIC_FREQUENCY);
  notch_filter_init(&measurement_notchfilter[INDI_ROLL], 0.0, STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_ROLL,
                    PERIODIC_FREQUENCY);
  notch_filter_init(&actuator_notchfilter[INDI_PITCH], 0.0, STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_PITCH,
                    PERIODIC_FREQUENCY);
  notch_filter_init(&measurement_notchfilter[INDI_PITCH], 0.0, STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_PITCH,
                    PERIODIC_FREQUENCY);
  notch_filter_init(&actuator_notchfilter[INDI_YAW], 0.0, STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_YAW,
                    PERIODIC_FREQUENCY);
  notch_filter_init(&measurement_notchfilter[INDI_YAW], 0.0, STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_YAW,
                    PERIODIC_FREQUENCY);
  notch_filter_init(&actuator_notchfilter[INDI_THRUST], 0.0, STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_THRUST,
                    PERIODIC_FREQUENCY);
  notch_filter_init(&measurement_notchfilter[INDI_THRUST], 0.0, STABILIZATION_ATTITUDE_HELI_INDI_NOTCHFILT_BW_THRUST,
                    PERIODIC_FREQUENCY);

  /* Low pass filter initialization, cutoff frequency in Hz */
  init_butterworth_2_low_pass_int(&actuator_lowpass_filters[INDI_ROLL],
                                  STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_ROLL, 1.0 / PERIODIC_FREQUENCY, 0);
  init_butterworth_2_low_pass_int(&measurement_lowpass_filters[INDI_ROLL],
                                  STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_ROLL, 1.0 / PERIODIC_FREQUENCY, 0);
  init_butterworth_2_low_pass_int(&actuator_lowpass_filters[INDI_PITCH],
                                  STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_PITCH, 1.0 / PERIODIC_FREQUENCY, 0);
  init_butterworth_2_low_pass_int(&measurement_lowpass_filters[INDI_PITCH],
                                  STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_PITCH, 1.0 / PERIODIC_FREQUENCY, 0);
  init_butterworth_2_low_pass_int(&actuator_lowpass_filters[INDI_YAW],
                                  STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_YAW, 1.0 / PERIODIC_FREQUENCY, 0);
  init_butterworth_2_low_pass_int(&measurement_lowpass_filters[INDI_YAW],
                                  STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_YAW, 1.0 / PERIODIC_FREQUENCY, 0);
  init_butterworth_2_low_pass_int(&actuator_lowpass_filters[INDI_THRUST],
                                  STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_THRUST, 1.0 / PERIODIC_FREQUENCY, 0);
  init_butterworth_2_low_pass_int(&measurement_lowpass_filters[INDI_THRUST],
                                  STABILIZATION_ATTITUDE_HELI_INDI_BUTTERW_CUTOFF_THRUST, 1.0 / PERIODIC_FREQUENCY, 0);

#if STABILIZATION_ATTITUDE_HELI_INDI_USE_FAST_DYN_FILTERS
  /* Fast dynamics in roll and pitch model */
  delayed_first_order_lowpass_initialize(&fast_dynamics_model[INDI_ROLL],
                                         STABILIZATION_ATTITUDE_HELI_INDI_FAST_DYN_ROLL_BW, 0, MAX_PPRZ, PERIODIC_FREQUENCY);
  delayed_first_order_lowpass_initialize(&fast_dynamics_model[INDI_PITCH],
                                         STABILIZATION_ATTITUDE_HELI_INDI_FAST_DYN_PITCH_BW, 0, MAX_PPRZ, PERIODIC_FREQUENCY);
#endif

  /* Assign filter functions: */
  c->apply_actuator_models = &indi_apply_actuator_models;
  c->apply_compensator_filters = &indi_apply_compensator_filters;
  c->apply_measurement_filters[0] = &indi_apply_measurement_notch_filters;
  c->apply_measurement_filters[1] = &indi_apply_measurement_butterworth_filters;
  c->apply_actuator_filters[0] = &indi_apply_actuator_notch_filters;
  c->apply_actuator_filters[1] = &indi_apply_actuator_butterworth_filters;

#if PERIODIC_TELEMETRY
  //register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_<<MSG>>, function);
#endif
}

void stabilization_attitude_run(bool in_flight)
{
  (void) in_flight; // unused variable
  struct IndiController_int *c = &heli_indi_ctl;

  /* calculate acceleration in body frame */
  struct NedCoor_i *ltp_accel_nedcoor = stateGetAccelNed_i();
  struct Int32Vect3 ltp_accel;
  struct Int32Vect3 body_accel; // Acceleration measurement in body frame
  ltp_accel.x = ltp_accel_nedcoor->x;
  ltp_accel.y = ltp_accel_nedcoor->y;
  ltp_accel.z = ltp_accel_nedcoor->z;
  int32_rmat_vmult(&body_accel, stateGetNedToBodyRMat_i(), &ltp_accel);

  /* attitude error */
  struct Int32Quat att_err;
  struct Int32Quat *att_quat = stateGetNedToBodyQuat_i();

  /* Add steady-state value to attitude setpoint, because heli has
   * non-zero roll angle by default
   */
  struct Int32Quat corr_att_sp_quat; // Corrected attitude setpoint
  int32_quat_comp_inv(&corr_att_sp_quat, &stab_att_sp_quat, &sp_offset);

  int32_quat_inv_comp(&att_err, att_quat, &corr_att_sp_quat);
  /* wrap it in the shortest direction */
  int32_quat_wrap_shortest(&att_err);
  int32_quat_normalize(&att_err);

  /* rate error (setpoint for rates = 0) */
  struct Int32Rates *body_rate = stateGetBodyRates_i();

  /* Inform INDI about the measurement */
  c->measurement[INDI_ROLL]  = body_rate->p;
  c->measurement[INDI_PITCH] = body_rate->q;
  c->measurement[INDI_YAW]   = body_rate->r;
  c->measurement[INDI_THRUST] = body_accel.z;

  /* Get RPM measurement */
#if STABILIZATION_ATTITUDE_HELI_INDI_USE_NOTCHFILTER
  if (heli_indi_ctl.enable_notch) {
    heli_indi_ctl.motor_rpm = rpm_sensor_get_rpm();
  } else {
    heli_indi_ctl.motor_rpm = 0;
  }
#endif

  /* Apply actuator dynamics model to previously commanded values
   * input  = actuator command in previous cycle
   * output = actual actuator position right now
   */
  c->apply_actuator_models(c->actuator_out, c->command_out[__k - 1]);

  /* Apply a set of filters, both to the actuator and the measurement */
  c->apply_actuator_filters[0](c->filtered_actuator[0], c->actuator_out);
  c->apply_measurement_filters[0](c->filtered_measurement[0], c->measurement);
  for (uint8_t i = 1; i < INDI_NR_FILTERS; i++) {
    c->apply_actuator_filters[i](c->filtered_actuator[i], c->filtered_actuator[i - 1]);
    c->apply_measurement_filters[i](c->filtered_measurement[i], c->filtered_measurement[i - 1]);
  }

  /* RADIO throttle stick value, for 4dof mode */
  //int32_t accel_z_sp = (-1)*3*((guidance_v_rc_delta_t - MAX_PPRZ/2) << INT32_ACCEL_FRAC) / (MAX_PPRZ/2);
  //accel_z_sp = ((accel_z_sp << INT32_TRIG_FRAC) / guidance_v_thrust_coeff);

  /* Transform yaw into a delta yaw while keeping filtered yawrate (kinda hacky) */
  int32_t filtered_measurement_vector[INDI_DOF];
  int32_vect_copy(filtered_measurement_vector, c->filtered_measurement[INDI_NR_FILTERS - 1], INDI_DOF);
  static int32_t previous_filt_yawrate = 0;
  filtered_measurement_vector[INDI_YAW] = 512 * (c->filtered_measurement[INDI_NR_FILTERS - 1][INDI_YAW] -
                                          previous_filt_yawrate);  // = approximately yaw acceleration error
  previous_filt_yawrate = c->filtered_measurement[INDI_NR_FILTERS - 1][INDI_YAW];

  /* Obtain virtual control input with P controller on pitch and roll */
  int32_t roll_virtual_control  = (heli_indi_gains.roll_p * att_err.qx)  / 4;
  int32_t pitch_virtual_control = (heli_indi_gains.pitch_p * att_err.qy) / 4;

  /* Yaw with cascaded PD-controller to generate virtual control */
  int32_t yaw_rate_reference = (heli_indi_gains.yaw_p * att_err.qz / 8);
  int32_t yaw_virtual_control = heli_indi_gains.yaw_d * (yaw_rate_reference - body_rate->r);

  /* Set INDI references */
  c->reference[INDI_ROLL]   = roll_virtual_control;
  c->reference[INDI_PITCH]  = pitch_virtual_control;
  c->reference[INDI_YAW]    = yaw_virtual_control;
  //c->reference[INDI_THRUST] = accel_z_sp;

  /* Subtract (filtered) measurement from reference to get the error */
  int32_vect_diff(c->error, c->reference, filtered_measurement_vector, INDI_DOF);

  /* Multiply error with inverse of actuator effectiveness, to get delta u (required increment in input) */
  MAT_MUL_VECT(INDI_DOF, c->du, c->invG, c->error);

  /* Bitshift back */
  c->du[INDI_ROLL]  >>= INT32_INVG_FRAC;
  c->du[INDI_PITCH] >>= INT32_INVG_FRAC;
  c->du[INDI_YAW]   >>= INT32_INVG_FRAC;
  c->du[INDI_THRUST] >>= INT32_INVG_FRAC;

  /* Take the current (filtered) actuator position and add the incremental value. */
  int32_vect_sum(c->u_setpoint, c->filtered_actuator[INDI_NR_FILTERS - 1], c->du, INDI_DOF);
  //c->u_setpoint[INDI_THRUST] = stabilization_cmd[COMMAND_THRUST];

  /* bound the result */
  BoundAbs(c->u_setpoint[INDI_ROLL], MAX_PPRZ);
  BoundAbs(c->u_setpoint[INDI_PITCH], MAX_PPRZ);
  Bound(c->u_setpoint[INDI_YAW], 0, MAX_PPRZ);
  Bound(c->u_setpoint[INDI_THRUST], 0.15 * MAX_PPRZ, MAX_PPRZ);

  /* Apply a compensator to the actuator setpoint to obtain actuator command */
  c->apply_compensator_filters(c->command_out[__k], c->u_setpoint);

  /* At the end, set 'previous' output to current output */
  int32_vect_copy(c->command_out[__k - 1], c->command_out[__k], INDI_DOF);

  /* Two correction angles, don't rotate but just add.
   * sin/cos = tan
   */
  stabilization_cmd[COMMAND_ROLL] = c->command_out[__k][INDI_ROLL]
                                    + c->command_out[__k][INDI_PITCH] * pprz_itrig_sin(c->pitch_comp_angle) / pprz_itrig_cos(c->pitch_comp_angle);
  stabilization_cmd[COMMAND_PITCH] = c->command_out[__k][INDI_PITCH]
                                     + c->command_out[__k][INDI_ROLL] * pprz_itrig_sin(c->roll_comp_angle) / pprz_itrig_cos(c->roll_comp_angle);
  stabilization_cmd[COMMAND_YAW] = c->command_out[__k][INDI_YAW];
  /* Thrust is not applied */

  /* Disable tail when not armed, because this thing goes crazy */
  if (!autopilot_get_motors_on()) {
    stabilization_cmd[COMMAND_YAW] = 0;
  }
}

void stabilization_attitude_enter(void)
{
  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();
}

void stabilization_attitude_set_failsafe_setpoint(void)
{
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
}

void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy)
{
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler = *rpy;

  int32_quat_of_eulers(&stab_att_sp_quat, &stab_att_sp_euler);
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
{
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler.psi = heading;

  // compute sp_euler phi/theta for debugging/telemetry
  /* Rotate horizontal commands to body frame by psi */
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);
  stab_att_sp_euler.phi = (-s_psi * cmd->x + c_psi * cmd->y) >> INT32_TRIG_FRAC;
  stab_att_sp_euler.theta = -(c_psi * cmd->x + s_psi * cmd->y) >> INT32_TRIG_FRAC;

  quat_from_earth_cmd_i(&stab_att_sp_quat, cmd, heading);
}

void stabilization_attitude_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn)
{
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}
