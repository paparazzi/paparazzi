/*
 * Copyright (C) Ewoud Smeur <ewoud_smeur@msn.com>
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

/** @file stabilization_attitude_quat_indi_simple.c
 * @brief MAVLab Delft University of Technology
 * This control algorithm is Incremental Nonlinear Dynamic Inversion (INDI)
 *
 * This is a simplified implementation of the publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 * http://arc.aiaa.org/doi/pdf/10.2514/1.G001490
 */

#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include "state.h"
#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/radio_control.h"

#if !defined(STABILIZATION_INDI_ACT_DYN_P) && !defined(STABILIZATION_INDI_ACT_DYN_Q) && !defined(STABILIZATION_INDI_ACT_DYN_R)
#error You have to define the first order time constant of the actuator dynamics!
#endif

// these parameters are used in the filtering of the angular acceleration
// define them in the airframe file if different values are required
#ifndef STABILIZATION_INDI_FILT_CUTOFF
#define STABILIZATION_INDI_FILT_CUTOFF 8.0
#endif

// the yaw sometimes requires more filtering
#ifndef STABILIZATION_INDI_FILT_CUTOFF_R
#define STABILIZATION_INDI_FILT_CUTOFF_R STABILIZATION_INDI_FILT_CUTOFF
#endif

#ifndef STABILIZATION_INDI_MAX_RATE
#define STABILIZATION_INDI_MAX_RATE 6.0
#endif

#if STABILIZATION_INDI_USE_ADAPTIVE
#warning "Use caution with adaptive indi. See the wiki for more info"
#endif

#ifndef STABILIZATION_INDI_MAX_R
#define STABILIZATION_INDI_MAX_R STABILIZATION_ATTITUDE_SP_MAX_R
#endif

#ifndef STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF
#define STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF 4.0
#endif

struct Int32Eulers stab_att_sp_euler;
struct Int32Quat   stab_att_sp_quat;

static int32_t stabilization_att_indi_cmd[COMMANDS_NB];
static inline void stabilization_indi_calc_cmd(int32_t indi_commands[]);
static inline void lms_estimation(void);
static void indi_init_filters(void);

//The G values are scaled to avoid numerical problems during the estimation
#define INDI_EST_SCALE 0.001

struct IndiVariables indi = {
  .max_rate = STABILIZATION_INDI_MAX_RATE,
  .attitude_max_yaw_rate = STABILIZATION_INDI_MAX_R,

  .g1 = {STABILIZATION_INDI_G1_P, STABILIZATION_INDI_G1_Q, STABILIZATION_INDI_G1_R},
  .g2 = STABILIZATION_INDI_G2_R,
  .reference_acceleration = {
    STABILIZATION_INDI_REF_ERR_P,
    STABILIZATION_INDI_REF_ERR_Q,
    STABILIZATION_INDI_REF_ERR_R,
    STABILIZATION_INDI_REF_RATE_P,
    STABILIZATION_INDI_REF_RATE_Q,
    STABILIZATION_INDI_REF_RATE_R
  },

  /* Estimation parameters for adaptive INDI */
  .est = {
    .g1 = {
      STABILIZATION_INDI_G1_P / INDI_EST_SCALE,
      STABILIZATION_INDI_G1_Q / INDI_EST_SCALE,
      STABILIZATION_INDI_G1_R / INDI_EST_SCALE
    },
    .g2 = STABILIZATION_INDI_G2_R / INDI_EST_SCALE,
    .mu = STABILIZATION_INDI_ADAPTIVE_MU,
  },

#if STABILIZATION_INDI_USE_ADAPTIVE
  .adaptive = TRUE,
#else
  .adaptive = FALSE,
#endif
};

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_att_indi(struct transport_tx *trans, struct link_device *dev)
{
  //The estimated G values are scaled, so scale them back before sending
  struct FloatRates g1_disp;
  RATES_SMUL(g1_disp, indi.est.g1, INDI_EST_SCALE);
  float g2_disp = indi.est.g2 * INDI_EST_SCALE;

  pprz_msg_send_STAB_ATTITUDE_INDI(trans, dev, AC_ID,
                                   &indi.rate_d[0],
                                   &indi.rate_d[1],
                                   &indi.rate_d[2],
                                   &indi.angular_accel_ref.p,
                                   &indi.angular_accel_ref.q,
                                   &indi.angular_accel_ref.r,
                                   &g1_disp.p,
                                   &g1_disp.q,
                                   &g1_disp.r,
                                   &g2_disp);
}

static void send_ahrs_ref_quat(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  pprz_msg_send_AHRS_REF_QUAT(trans, dev, AC_ID,
                              &stab_att_sp_quat.qi,
                              &stab_att_sp_quat.qx,
                              &stab_att_sp_quat.qy,
                              &stab_att_sp_quat.qz,
                              &(quat->qi),
                              &(quat->qx),
                              &(quat->qy),
                              &(quat->qz));
}
#endif

void stabilization_indi_init(void)
{
  // Initialize filters
  indi_init_filters();

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_INDI, send_att_indi);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_REF_QUAT, send_ahrs_ref_quat);
#endif
}

void indi_init_filters(void)
{
  // tau = 1/(2*pi*Fc)
  float tau = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_FILT_CUTOFF);
  float tau_r = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_FILT_CUTOFF_R);
  float tau_axis[3] = {tau, tau, tau_r};
  float tau_est = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  // Filtering of gyroscope and actuators
  for (int8_t i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&indi.u[i], tau_axis[i], sample_time, 0.0);
    init_butterworth_2_low_pass(&indi.rate[i], tau_axis[i], sample_time, 0.0);
    init_butterworth_2_low_pass(&indi.est.u[i], tau_est, sample_time, 0.0);
    init_butterworth_2_low_pass(&indi.est.rate[i], tau_est, sample_time, 0.0);
  }
}

void stabilization_indi_enter(void)
{
  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  FLOAT_RATES_ZERO(indi.angular_accel_ref);
  FLOAT_RATES_ZERO(indi.u_act_dyn);
  FLOAT_RATES_ZERO(indi.u_in);

  // Re-initialize filters
  indi_init_filters();
}

void stabilization_indi_set_failsafe_setpoint(void)
{
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
}

/**
 * @brief Set attitude quaternion setpoint from rpy
 *
 * @param rpy roll pitch yaw input
 */
void stabilization_indi_set_rpy_setpoint_i(struct Int32Eulers *rpy)
{
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler = *rpy;

  int32_quat_of_eulers(&stab_att_sp_quat, &stab_att_sp_euler);
}

/**
 * @brief Set attitude setpoint from command in earth axes
 *
 * @param cmd The command in earth axes (North East)
 * @param heading The desired heading
 */
void stabilization_indi_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
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

/**
 * @brief Update butterworth filter for p, q and r of a FloatRates struct
 *
 * @param filter The filter array to use
 * @param new_values The new values
 */
static inline void filter_pqr(Butterworth2LowPass *filter, struct FloatRates *new_values)
{
  update_butterworth_2_low_pass(&filter[0], new_values->p);
  update_butterworth_2_low_pass(&filter[1], new_values->q);
  update_butterworth_2_low_pass(&filter[2], new_values->r);
}

/**
 * @brief Caclulate finite difference form a filter array
 * The filter already contains the previous values
 *
 * @param output The output array
 * @param filter The filter array input
 */
static inline void finite_difference_from_filter(float *output, Butterworth2LowPass *filter)
{
  for (int8_t i = 0; i < 3; i++) {
    output[i] = (filter[i].o[0] - filter[i].o[1]) * PERIODIC_FREQUENCY;
  }
}

/**
 * @brief Calculate derivative of an array via finite difference
 *
 * @param output[3] The output array
 * @param new[3] The newest values
 * @param old[3] The values of the previous timestep
 */
static inline void finite_difference(float output[3], float new[3], float old[3])
{
  for (int8_t i = 0; i < 3; i++) {
    output[i] = (new[i] - old[i])*PERIODIC_FREQUENCY;
  }
}

/**
 * @brief Does the INDI calculations
 *
 * @param indi_commands[] Array of commands that the function will write to
 * @param att_err quaternion attitude error
 * @param rate_control rate control enabled, otherwise attitude control
 */
void stabilization_indi_calc_cmd(int32_t indi_commands[])
{
  //Increment in angular acceleration requires increment in control input
  //G1 is the control effectiveness. In the yaw axis, we need something additional: G2.
  //It takes care of the angular acceleration caused by the change in rotation rate of the propellers
  //(they have significant inertia, see the paper mentioned in the header for more explanation)
  indi.du.p = 1.0 / indi.g1.p * (indi.angular_accel_ref.p - indi.rate_d[0]);
  indi.du.q = 1.0 / indi.g1.q * (indi.angular_accel_ref.q - indi.rate_d[1]);
  indi.du.r = 1.0 / (indi.g1.r + indi.g2) * (indi.angular_accel_ref.r - indi.rate_d[2] + indi.g2 * indi.du.r);

  //add the increment to the total control input
  indi.u_in.p = indi.u[0].o[0] + indi.du.p;
  indi.u_in.q = indi.u[1].o[0] + indi.du.q;
  indi.u_in.r = indi.u[2].o[0] + indi.du.r;

  //bound the total control input
#if STABILIZATION_INDI_FULL_AUTHORITY
  Bound(indi.u_in.p, -9600, 9600);
  Bound(indi.u_in.q, -9600, 9600);
  float rlim = 9600 - fabs(indi.u_in.q);
  Bound(indi.u_in.r, -rlim, rlim);
  Bound(indi.u_in.r, -9600, 9600);
#else
  Bound(indi.u_in.p, -4500, 4500);
  Bound(indi.u_in.q, -4500, 4500);
  Bound(indi.u_in.r, -4500, 4500);
#endif

  //Propagate input filters
  //first order actuator dynamics
  indi.u_act_dyn.p = indi.u_act_dyn.p + STABILIZATION_INDI_ACT_DYN_P * (indi.u_in.p - indi.u_act_dyn.p);
  indi.u_act_dyn.q = indi.u_act_dyn.q + STABILIZATION_INDI_ACT_DYN_Q * (indi.u_in.q - indi.u_act_dyn.q);
  indi.u_act_dyn.r = indi.u_act_dyn.r + STABILIZATION_INDI_ACT_DYN_R * (indi.u_in.r - indi.u_act_dyn.r);

  //Don't increment if thrust is off
  //TODO: this should be something more elegant, but without this the inputs
  //will increment to the maximum before even getting in the air.
  if (stabilization_cmd[COMMAND_THRUST] < 300) {
    FLOAT_RATES_ZERO(indi.du);
    FLOAT_RATES_ZERO(indi.u_act_dyn);
    FLOAT_RATES_ZERO(indi.u_in);
  } else {
    // only run the estimation if the commands are not zero.
    lms_estimation();
  }

  /*  INDI feedback */
  indi_commands[COMMAND_ROLL] = indi.u_in.p;
  indi_commands[COMMAND_PITCH] = indi.u_in.q;
  indi_commands[COMMAND_YAW] = indi.u_in.r;
}

/**
 * @brief runs stabilization indi
 *
 * @param in_flight not used
 * @param rate_control rate control enabled, otherwise attitude control
 */
void stabilization_indi_attitude_run(bool in_flight __attribute__((unused)), struct Int32Quat quat_sp)
{
  /* attitude error                          */
  struct Int32Quat att_err;
  struct Int32Quat *att_quat = stateGetNedToBodyQuat_i();
  int32_quat_inv_comp(&att_err, att_quat, &quat_sp);
  /* wrap it in the shortest direction       */
  int32_quat_wrap_shortest(&att_err);
  int32_quat_normalize(&att_err);

  // Propagate the filter on the gyroscopes and actuators
  struct FloatRates *body_rates = stateGetBodyRates_f();
  filter_pqr(indi.u, &indi.u_act_dyn);
  filter_pqr(indi.rate, body_rates);

  // Calculate the derivative of the rates
  finite_difference_from_filter(indi.rate_d, indi.rate);

  //The rates used for feedback are by default the measured rates. If needed they can be filtered (see below)
  struct FloatRates rates_for_feedback;
  RATES_COPY(rates_for_feedback, (*body_rates));

  //If there is a lot of noise on the gyroscope, it might be good to use the filtered value for feedback.
  //Note that due to the delay, the PD controller can not be as aggressive.
#if STABILIZATION_INDI_FILTER_ROLL_RATE
  rates_for_feedback.p = indi.rate[0].o[0];
#endif
#if STABILIZATION_INDI_FILTER_PITCH_RATE
  rates_for_feedback.q = indi.rate[1].o[0];
#endif
#if STABILIZATION_INDI_FILTER_YAW_RATE
  rates_for_feedback.r = indi.rate[2].o[0];
#endif

  indi.angular_accel_ref.p = indi.reference_acceleration.err_p * QUAT1_FLOAT_OF_BFP(att_err->qx)
                             - indi.reference_acceleration.rate_p * rates_for_feedback.p;

  indi.angular_accel_ref.q = indi.reference_acceleration.err_q * QUAT1_FLOAT_OF_BFP(att_err->qy)
                             - indi.reference_acceleration.rate_q * rates_for_feedback.q;

  //This separates the P and D controller and lets you impose a maximum yaw rate.
  float rate_ref_r = indi.reference_acceleration.err_r * QUAT1_FLOAT_OF_BFP(att_err->qz) / indi.reference_acceleration.rate_r;
  BoundAbs(rate_ref_r, indi.attitude_max_yaw_rate);
  indi.angular_accel_ref.r = indi.reference_acceleration.rate_r * (rate_ref_r - rates_for_feedback.r);

  /* compute the INDI command */
  stabilization_indi_calc_cmd(stabilization_att_indi_cmd);

  /* copy the INDI command */
  stabilization_cmd[COMMAND_ROLL] = stabilization_att_indi_cmd[COMMAND_ROLL];
  stabilization_cmd[COMMAND_PITCH] = stabilization_att_indi_cmd[COMMAND_PITCH];
  stabilization_cmd[COMMAND_YAW] = stabilization_att_indi_cmd[COMMAND_YAW];

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
}

/**
 * This function reads rc commands
 *
 * @param in_flight boolean that states if the UAV is in flight or not
 */
void stabilization_indi_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn)
{
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}

/**
 * This is a Least Mean Squares adaptive filter
 * It estimates the actuator effectiveness online, by comparing the expected
 * angular acceleration based on the inputs with the measured angular
 * acceleration
 */
static inline void lms_estimation(void)
{
  static struct IndiEstimation *est = &indi.est;
  // Only pass really low frequencies so you don't adapt to noise
  struct FloatRates *body_rates = stateGetBodyRates_f();
  filter_pqr(est->u, &indi.u_act_dyn);
  filter_pqr(est->rate, body_rates);

  // Calculate the first and second derivatives of the rates and actuators
  float rate_d_prev[3];
  float u_d_prev[3];
  float_vect_copy(rate_d_prev, est->rate_d, 3);
  float_vect_copy(u_d_prev, est->u_d, 3);
  finite_difference_from_filter(est->rate_d, est->rate);
  finite_difference_from_filter(est->u_d, est->u);
  finite_difference(est->rate_dd, est->rate_d, rate_d_prev);
  finite_difference(est->u_dd, est->u_d, u_d_prev);

  // The inputs are scaled in order to avoid overflows
  float du[3];
  float_vect_copy(du, est->u_d, 3);
  float_vect_scale(du, INDI_EST_SCALE, 3);
  est->g1.p = est->g1.p - (est->g1.p * du[0] - est->rate_dd[0]) * du[0] * est->mu;
  est->g1.q = est->g1.q - (est->g1.q * du[1] - est->rate_dd[1]) * du[1] * est->mu;
  float ddu = est->u_dd[2] * INDI_EST_SCALE / PERIODIC_FREQUENCY;
  float error = (est->g1.r * du[2] + est->g2 * ddu - est->rate_dd[2]);
  est->g1.r = est->g1.r - error * du[2] * est->mu / 3;
  est->g2 = est->g2 - error * 1000 * ddu * est->mu / 3;

  //the g values should be larger than zero, otherwise there is positive feedback, the command will go to max and there is nothing to learn anymore...
  if (est->g1.p < 0.01) { est->g1.p = 0.01; }
  if (est->g1.q < 0.01) { est->g1.q = 0.01; }
  if (est->g1.r < 0.01) { est->g1.r = 0.01; }
  if (est->g2   < 0.01) { est->g2 = 0.01; }

  if (indi.adaptive) {
    //Commit the estimated G values and apply the scaling
    indi.g1.p = est->g1.p * INDI_EST_SCALE;
    indi.g1.q = est->g1.q * INDI_EST_SCALE;
    indi.g1.r = est->g1.r * INDI_EST_SCALE;
    indi.g2   = est->g2 * INDI_EST_SCALE;
  }
}
