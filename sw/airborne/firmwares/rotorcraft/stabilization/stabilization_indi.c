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

/** @file stabilization_attitude_quat_indi.c
 * @brief MAVLab Delft University of Technology
 * This control algorithm is Incremental Nonlinear Dynamic Inversion (INDI)
 *
 * This is a simplified implementation of the publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 * http://arc.aiaa.org/doi/pdf/10.2514/1.G001490
 */

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
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
#ifndef STABILIZATION_INDI_FILT_OMEGA
#define STABILIZATION_INDI_FILT_OMEGA 50.0
#endif

#ifndef STABILIZATION_INDI_FILT_ZETA
#define STABILIZATION_INDI_FILT_ZETA 0.55
#endif

// the yaw sometimes requires more filtering
#ifndef STABILIZATION_INDI_FILT_OMEGA_R
#define STABILIZATION_INDI_FILT_OMEGA_R STABILIZATION_INDI_FILT_OMEGA
#endif

#ifndef STABILIZATION_INDI_FILT_ZETA_R
#define STABILIZATION_INDI_FILT_ZETA_R STABILIZATION_INDI_FILT_ZETA
#endif

#ifndef STABILIZATION_INDI_MAX_RATE
#define STABILIZATION_INDI_MAX_RATE 6.0
#endif

#if STABILIZATION_INDI_USE_ADAPTIVE
#warning "Use caution with adaptive indi. See the wiki for more info"
#endif

struct Int32Eulers stab_att_sp_euler;
struct Int32Quat   stab_att_sp_quat;

static int32_t stabilization_att_indi_cmd[COMMANDS_NB];
static inline void stabilization_indi_calc_cmd(int32_t indi_commands[], struct Int32Quat *att_err, bool rate_control);
static void stabilization_indi_second_order_filter_init(struct IndiFilter *filter, float omega, float zeta, float omega_r);
static void stabilization_indi_second_order_filter(struct IndiFilter *filter, struct FloatRates *input);
static inline void lms_estimation(void);

#define INDI_EST_SCALE 0.001 //The G values are scaled to avoid numerical problems during the estimation
struct IndiVariables indi = {
  .max_rate = STABILIZATION_INDI_MAX_RATE,

  .g1 = {STABILIZATION_INDI_G1_P, STABILIZATION_INDI_G1_Q, STABILIZATION_INDI_G1_R},
  .g2 = STABILIZATION_INDI_G2_R,
  .reference_acceleration = {
    STABILIZATION_INDI_REF_ERR_P,
    STABILIZATION_INDI_REF_ERR_Q,
    STABILIZATION_INDI_REF_ERR_R,
    STABILIZATION_INDI_REF_RATE_P,
    STABILIZATION_INDI_REF_RATE_Q,
    STABILIZATION_INDI_REF_RATE_R},

  /* Estimation parameters for adaptive INDI */
  .est = {
    .g1 = {
      STABILIZATION_INDI_G1_P / INDI_EST_SCALE,
      STABILIZATION_INDI_G1_Q / INDI_EST_SCALE,
      STABILIZATION_INDI_G1_R / INDI_EST_SCALE},
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
                                   &indi.rate.dx.p,
                                   &indi.rate.dx.q,
                                   &indi.rate.dx.r,
                                   &indi.angular_accel_ref.p,
                                   &indi.angular_accel_ref.q,
                                   &indi.angular_accel_ref.r,
                                   &g1_disp.p,
                                   &g1_disp.q,
                                   &g1_disp.r,
                                   &g2_disp);
}
#endif

void stabilization_indi_init(void)
{
  // Initialize filters
  stabilization_indi_second_order_filter_init(&indi.rate, STABILIZATION_INDI_FILT_OMEGA, STABILIZATION_INDI_FILT_ZETA, STABILIZATION_INDI_FILT_OMEGA_R);
  stabilization_indi_second_order_filter_init(&indi.u, STABILIZATION_INDI_FILT_OMEGA, STABILIZATION_INDI_FILT_ZETA, STABILIZATION_INDI_FILT_OMEGA_R);
  stabilization_indi_second_order_filter_init(&indi.est.rate, 10.0, 0.8, 10.0); //FIXME: no magic number
  stabilization_indi_second_order_filter_init(&indi.est.u, 10.0, 0.8, 10.0); //FIXME: no magic number

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_INDI, send_att_indi);
#endif
}

void stabilization_indi_enter(void)
{
  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  FLOAT_RATES_ZERO(indi.rate.x);
  FLOAT_RATES_ZERO(indi.rate.dx);
  FLOAT_RATES_ZERO(indi.rate.ddx);
  FLOAT_RATES_ZERO(indi.angular_accel_ref);
  FLOAT_RATES_ZERO(indi.du);
  FLOAT_RATES_ZERO(indi.u_act_dyn);
  FLOAT_RATES_ZERO(indi.u_in);
  FLOAT_RATES_ZERO(indi.u.x);
  FLOAT_RATES_ZERO(indi.u.dx);
  FLOAT_RATES_ZERO(indi.u.ddx);
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

void stabilization_indi_set_rpy_setpoint_i(struct Int32Eulers *rpy)
{
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler = *rpy;

  quat_from_rpy_cmd_i(&stab_att_sp_quat, &stab_att_sp_euler);
}

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

static inline void stabilization_indi_calc_cmd(int32_t indi_commands[], struct Int32Quat *att_err, bool rate_control)
{
  /* Propagate the second order filter on the gyroscopes */
  struct FloatRates *body_rates = stateGetBodyRates_f();
  stabilization_indi_second_order_filter(&indi.rate, body_rates);

#if STABILIZATION_INDI_FILTER_ROLL_RATE
  indi.angular_accel_ref.p = indi.reference_acceleration.err_p * QUAT1_FLOAT_OF_BFP(att_err->qx)
                             - indi.reference_acceleration.rate_p * indi.rate.x.p;
#else
  indi.angular_accel_ref.p = indi.reference_acceleration.err_p * QUAT1_FLOAT_OF_BFP(att_err->qx)
                             - indi.reference_acceleration.rate_p * body_rates->p;
#endif
#if STABILIZATION_INDI_FILTER_PITCH_RATE
  indi.angular_accel_ref.q = indi.reference_acceleration.err_q * QUAT1_FLOAT_OF_BFP(att_err->qy)
                             - indi.reference_acceleration.rate_q * indi.rate.x.q;
#else
  indi.angular_accel_ref.q = indi.reference_acceleration.err_q * QUAT1_FLOAT_OF_BFP(att_err->qy)
                             - indi.reference_acceleration.rate_q * body_rates->q;
#endif
#if STABILIZATION_INDI_FILTER_YAW_RATE
  indi.angular_accel_ref.r = indi.reference_acceleration.err_r * QUAT1_FLOAT_OF_BFP(att_err->qz)
                             - indi.reference_acceleration.rate_r * indi.rate.x.r;
#else
  indi.angular_accel_ref.r = indi.reference_acceleration.err_r * QUAT1_FLOAT_OF_BFP(att_err->qz)
                             - indi.reference_acceleration.rate_r * body_rates->r;
#endif

  /* Check if we are running the rate controller and overwrite */
  if(rate_control) {
    indi.angular_accel_ref.p =  indi.reference_acceleration.rate_p * ((float)radio_control.values[RADIO_ROLL]  / MAX_PPRZ * indi.max_rate - body_rates->p);
    indi.angular_accel_ref.q =  indi.reference_acceleration.rate_q * ((float)radio_control.values[RADIO_PITCH] / MAX_PPRZ * indi.max_rate - body_rates->q);
    indi.angular_accel_ref.r =  indi.reference_acceleration.rate_r * ((float)radio_control.values[RADIO_YAW]   / MAX_PPRZ * indi.max_rate - body_rates->r);
  }

  //Increment in angular acceleration requires increment in control input
  //G1 is the control effectiveness. In the yaw axis, we need something additional: G2.
  //It takes care of the angular acceleration caused by the change in rotation rate of the propellers
  //(they have significant inertia, see the paper mentioned in the header for more explanation)
  indi.du.p = 1.0 / indi.g1.p * (indi.angular_accel_ref.p - indi.rate.dx.p);
  indi.du.q = 1.0 / indi.g1.q * (indi.angular_accel_ref.q - indi.rate.dx.q);
  indi.du.r = 1.0 / (indi.g1.r + indi.g2) * (indi.angular_accel_ref.r - indi.rate.dx.r + indi.g2 * indi.du.r);

  //add the increment to the total control input
  indi.u_in.p = indi.u.x.p + indi.du.p;
  indi.u_in.q = indi.u.x.q + indi.du.q;
  indi.u_in.r = indi.u.x.r + indi.du.r;

  //bound the total control input
  Bound(indi.u_in.p, -4500, 4500);
  Bound(indi.u_in.q, -4500, 4500);
  Bound(indi.u_in.r, -4500, 4500);

  //Propagate input filters
  //first order actuator dynamics
  indi.u_act_dyn.p = indi.u_act_dyn.p + STABILIZATION_INDI_ACT_DYN_P * (indi.u_in.p - indi.u_act_dyn.p);
  indi.u_act_dyn.q = indi.u_act_dyn.q + STABILIZATION_INDI_ACT_DYN_Q * (indi.u_in.q - indi.u_act_dyn.q);
  indi.u_act_dyn.r = indi.u_act_dyn.r + STABILIZATION_INDI_ACT_DYN_R * (indi.u_in.r - indi.u_act_dyn.r);

  //sensor filter
  stabilization_indi_second_order_filter(&indi.u, &indi.u_act_dyn);

  //Don't increment if thrust is off
  //TODO: this should be something more elegant, but without this the inputs will increment to the maximum before
  //even getting in the air.
  if (stabilization_cmd[COMMAND_THRUST] < 300) {
    FLOAT_RATES_ZERO(indi.du);
    FLOAT_RATES_ZERO(indi.u_act_dyn);
    FLOAT_RATES_ZERO(indi.u_in);
    FLOAT_RATES_ZERO(indi.u.x);
    FLOAT_RATES_ZERO(indi.u.dx);
    FLOAT_RATES_ZERO(indi.u.ddx);
  } else {
    // only run the estimation if the commands are not zero.
    lms_estimation();
  }

  /*  INDI feedback */
  indi_commands[COMMAND_ROLL] = indi.u_in.p;
  indi_commands[COMMAND_PITCH] = indi.u_in.q;
  indi_commands[COMMAND_YAW] = indi.u_in.r;
}

void stabilization_indi_run(bool enable_integrator __attribute__((unused)), bool rate_control)
{
  /* attitude error                          */
  struct Int32Quat att_err;
  struct Int32Quat *att_quat = stateGetNedToBodyQuat_i();
  int32_quat_inv_comp(&att_err, att_quat, &stab_att_sp_quat);
  /* wrap it in the shortest direction       */
  int32_quat_wrap_shortest(&att_err);
  int32_quat_normalize(&att_err);

  /* compute the INDI command */
  stabilization_indi_calc_cmd(stabilization_att_indi_cmd, &att_err, rate_control);

  /* copy the INDI command */
  stabilization_cmd[COMMAND_ROLL] = stabilization_att_indi_cmd[COMMAND_ROLL];
  stabilization_cmd[COMMAND_PITCH] = stabilization_att_indi_cmd[COMMAND_PITCH];
  stabilization_cmd[COMMAND_YAW] = stabilization_att_indi_cmd[COMMAND_YAW];

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
}

// This function reads rc commands
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

// Initialize a second order low pass filter
static void stabilization_indi_second_order_filter_init(struct IndiFilter *filter, float omega, float zeta, float omega_r)
{
  filter->omega = omega;
  filter->omega2 = omega * omega;
  filter->zeta = zeta;
  filter->omega_r = omega_r;
  filter->omega2_r = omega_r * omega_r;
}

// This is a simple second order low pass filter
static void stabilization_indi_second_order_filter(struct IndiFilter *filter, struct FloatRates *input)
{
  float_rates_integrate_fi(&filter->x, &filter->dx, 1.0 / PERIODIC_FREQUENCY);
  float_rates_integrate_fi(&filter->dx, &filter->ddx, 1.0 / PERIODIC_FREQUENCY);

  filter->ddx.p = -filter->dx.p * 2 * filter->zeta * filter->omega   + (input->p - filter->x.p) * filter->omega2;
  filter->ddx.q = -filter->dx.q * 2 * filter->zeta * filter->omega   + (input->q - filter->x.q) * filter->omega2;
  filter->ddx.r = -filter->dx.r * 2 * filter->zeta * filter->omega_r + (input->r - filter->x.r) * filter->omega2_r;
}

// This is a Least Mean Squares adaptive filter
// It estiamtes the actuator effectiveness online by comparing the expected angular acceleration based on the inputs with the measured angular acceleration
static inline void lms_estimation(void)
{
  static struct IndiEstimation *est = &indi.est;
  // Only pass really low frequencies so you don't adapt to noise
  stabilization_indi_second_order_filter(&est->u, &indi.u_act_dyn);
  struct FloatRates *body_rates = stateGetBodyRates_f();
  stabilization_indi_second_order_filter(&est->rate, body_rates);

  // The inputs are scaled in order to avoid overflows
  float du = est->u.dx.p * INDI_EST_SCALE;
  est->g1.p = est->g1.p - (est->g1.p * du - est->rate.ddx.p) * du * est->mu;
  du = est->u.dx.q * INDI_EST_SCALE;
  est->g1.q = est->g1.q - (est->g1.q * du - est->rate.ddx.q) * du * est->mu;
  du = est->u.dx.r * INDI_EST_SCALE;
  float ddu = est->u.ddx.r * INDI_EST_SCALE / PERIODIC_FREQUENCY;
  float error = (est->g1.r * du + est->g2 * ddu - est->rate.ddx.r);
  est->g1.r = est->g1.r - error * du * est->mu / 3;
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
