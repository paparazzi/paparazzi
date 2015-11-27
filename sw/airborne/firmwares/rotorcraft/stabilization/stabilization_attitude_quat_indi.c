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
 * MAVLab Delft University of Technology
 * This control algorithm is Incremental Nonlinear Dynamic Inversion (INDI)
 *
 * This is a simplified implementation of the (soon to be) publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 */

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include "state.h"
#include "generated/airframe.h"
#include "paparazzi.h"

#if !defined(STABILIZATION_INDI_ACT_DYN_P) && !defined(STABILIZATION_INDI_ACT_DYN_Q) && !defined(STABILIZATION_INDI_ACT_DYN_R)
#error You have to define the first order time constant of the actuator dynamics!
#endif

int32_t stabilization_att_indi_cmd[COMMANDS_NB];

struct FloatRates g1 = {STABILIZATION_INDI_G1_P, STABILIZATION_INDI_G1_Q, STABILIZATION_INDI_G1_R};
float g2 = STABILIZATION_INDI_G2_R;
struct ReferenceSystem reference_acceleration = {STABILIZATION_INDI_REF_ERR_P,
         STABILIZATION_INDI_REF_ERR_Q,
         STABILIZATION_INDI_REF_ERR_R,
         STABILIZATION_INDI_REF_RATE_P,
         STABILIZATION_INDI_REF_RATE_Q,
         STABILIZATION_INDI_REF_RATE_R,
};

struct IndiVariables indi = {
  {0., 0., 0.},
  {0., 0., 0.},
  {0., 0., 0.},
  {0., 0., 0.},
  {0., 0., 0.},
  {0., 0., 0.},
  {0., 0., 0.},
  {0., 0., 0.},
  {0., 0., 0.},
  {0., 0., 0.}
};

struct Int32Eulers stab_att_sp_euler;
struct Int32Quat   stab_att_sp_quat;

// Variables for adaptation
struct FloatRates filtered_rate_estimation = {0., 0., 0.};
struct FloatRates filtered_rate_deriv_estimation  = {0., 0., 0.};
struct FloatRates filtered_rate_2deriv_estimation  = {0., 0., 0.};
struct FloatRates indi_u_estimation  = {0., 0., 0.};
struct FloatRates udot_estimation  = {0., 0., 0.};
struct FloatRates udotdot_estimation  = {0., 0., 0.};
#define INDI_EST_SCALE 0.001 //The G values are scaled to avoid numerical problems during the estimation
struct FloatRates g_est = {STABILIZATION_INDI_G1_P / INDI_EST_SCALE, STABILIZATION_INDI_G1_Q / INDI_EST_SCALE, STABILIZATION_INDI_G1_R / INDI_EST_SCALE};
float g2_est = STABILIZATION_INDI_G2_R / INDI_EST_SCALE;
float mu = STABILIZATION_INDI_ADAPTIVE_MU;

#if STABILIZATION_INDI_USE_ADAPTIVE
#warning "Use caution with adaptive indi. See the wiki for more info"
bool_t use_adaptive_indi = TRUE;
#else
bool_t use_adaptive_indi = FALSE;
#endif

#ifndef STABILIZATION_INDI_FILT_OMEGA
#define STABILIZATION_INDI_FILT_OMEGA 50.0
#endif

#ifndef STABILIZATION_INDI_FILT_ZETA
#define STABILIZATION_INDI_FILT_ZETA 0.55
#endif

#define STABILIZATION_INDI_FILT_OMEGA2 (STABILIZATION_INDI_FILT_OMEGA*STABILIZATION_INDI_FILT_OMEGA)

#ifndef STABILIZATION_INDI_FILT_OMEGA_R
#define STABILIZATION_INDI_FILT_OMEGA_R STABILIZATION_INDI_FILT_OMEGA
#define STABILIZATION_INDI_FILT_ZETA_R STABILIZATION_INDI_FILT_ZETA
#endif

#define STABILIZATION_INDI_FILT_OMEGA2_R (STABILIZATION_INDI_FILT_OMEGA_R*STABILIZATION_INDI_FILT_OMEGA_R)

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_att_indi(struct transport_tx *trans, struct link_device *dev)
{
  //The estimated G values are scaled, so scale them back before sending
  struct FloatRates g_est_disp;
  RATES_SMUL(g_est_disp, g_est, INDI_EST_SCALE);
  float g2_est_disp = g2_est * INDI_EST_SCALE;

  pprz_msg_send_STAB_ATTITUDE_INDI(trans, dev, AC_ID,
                                   &indi.filtered_rate_deriv.p,
                                   &indi.filtered_rate_deriv.q,
                                   &indi.filtered_rate_deriv.r,
                                   &indi.angular_accel_ref.p,
                                   &indi.angular_accel_ref.q,
                                   &indi.angular_accel_ref.r,
                                   &g_est_disp.p,
                                   &g_est_disp.q,
                                   &g_est_disp.r,
                                   &g2_est_disp);
}
#endif

void stabilization_attitude_init(void)
{

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_INDI, send_att_indi);
#endif
}

void stabilization_attitude_enter(void)
{

  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  FLOAT_RATES_ZERO(indi.filtered_rate);
  FLOAT_RATES_ZERO(indi.filtered_rate_deriv);
  FLOAT_RATES_ZERO(indi.filtered_rate_2deriv);
  FLOAT_RATES_ZERO(indi.angular_accel_ref);
  FLOAT_RATES_ZERO(indi.u);
  FLOAT_RATES_ZERO(indi.du);
  FLOAT_RATES_ZERO(indi.u_act_dyn);
  FLOAT_RATES_ZERO(indi.u_in);
  FLOAT_RATES_ZERO(indi.udot);
  FLOAT_RATES_ZERO(indi.udotdot);

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

static void attitude_run_indi(int32_t indi_commands[], struct Int32Quat *att_err, bool_t in_flight __attribute__((unused)))
{
  //Calculate required angular acceleration
  struct FloatRates *body_rate = stateGetBodyRates_f();
#if STABILIZATION_INDI_FILTER_ROLL_RATE
  indi.angular_accel_ref.p = reference_acceleration.err_p * QUAT1_FLOAT_OF_BFP(att_err->qx)
                             - reference_acceleration.rate_p * indi.filtered_rate.p;
#else
  indi.angular_accel_ref.p = reference_acceleration.err_p * QUAT1_FLOAT_OF_BFP(att_err->qx)
                             - reference_acceleration.rate_p * body_rate->p;
#endif
#if STABILIZATION_INDI_FILTER_PITCH_RATE
  indi.angular_accel_ref.q = reference_acceleration.err_q * QUAT1_FLOAT_OF_BFP(att_err->qy)
                             - reference_acceleration.rate_q * indi.filtered_rate.q;
#else
  indi.angular_accel_ref.q = reference_acceleration.err_q * QUAT1_FLOAT_OF_BFP(att_err->qy)
                             - reference_acceleration.rate_q * body_rate->q;
#endif
#if STABILIZATION_INDI_FILTER_YAW_RATE
  indi.angular_accel_ref.r = reference_acceleration.err_r * QUAT1_FLOAT_OF_BFP(att_err->qz)
                             - reference_acceleration.rate_r * indi.filtered_rate.r;
#else
  indi.angular_accel_ref.r = reference_acceleration.err_r * QUAT1_FLOAT_OF_BFP(att_err->qz)
                             - reference_acceleration.rate_r * body_rate->r;
#endif

  //Incremented in angular acceleration requires increment in control input
  //G1 is the actuator effectiveness. In the yaw axis, we need something additional: G2.
  //It takes care of the angular acceleration caused by the change in rotation rate of the propellers
  //(they have significant inertia, see the paper mentioned in the header for more explanation)
  indi.du.p = 1.0 / g1.p * (indi.angular_accel_ref.p - indi.filtered_rate_deriv.p);
  indi.du.q = 1.0 / g1.q * (indi.angular_accel_ref.q - indi.filtered_rate_deriv.q);
  indi.du.r = 1.0 / (g1.r + g2) * (indi.angular_accel_ref.r - indi.filtered_rate_deriv.r + g2 * indi.du.r);

  //add the increment to the total control input
  indi.u_in.p = indi.u.p + indi.du.p;
  indi.u_in.q = indi.u.q + indi.du.q;
  indi.u_in.r = indi.u.r + indi.du.r;

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
  stabilization_indi_second_order_filter(&indi.u_act_dyn, &indi.udotdot, &indi.udot, &indi.u,
                                         STABILIZATION_INDI_FILT_OMEGA, STABILIZATION_INDI_FILT_ZETA, STABILIZATION_INDI_FILT_OMEGA_R);

  //Don't increment if thrust is off
  if (stabilization_cmd[COMMAND_THRUST] < 300) {
    FLOAT_RATES_ZERO(indi.u);
    FLOAT_RATES_ZERO(indi.du);
    FLOAT_RATES_ZERO(indi.u_act_dyn);
    FLOAT_RATES_ZERO(indi.u_in);
    FLOAT_RATES_ZERO(indi.udot);
    FLOAT_RATES_ZERO(indi.udotdot);
  } else {
    lms_estimation();
  }

  /*  INDI feedback */
  indi_commands[COMMAND_ROLL] = indi.u_in.p;
  indi_commands[COMMAND_PITCH] = indi.u_in.q;
  indi_commands[COMMAND_YAW] = indi.u_in.r;
}

void stabilization_attitude_run(bool_t enable_integrator)
{

  /* Propagate the second order filter on the gyroscopes */
  struct FloatRates *body_rates = stateGetBodyRates_f();
  stabilization_indi_second_order_filter(body_rates, &indi.filtered_rate_2deriv, &indi.filtered_rate_deriv,
                                         &indi.filtered_rate, STABILIZATION_INDI_FILT_OMEGA, STABILIZATION_INDI_FILT_ZETA, STABILIZATION_INDI_FILT_OMEGA_R);

  /* attitude error                          */
  struct Int32Quat att_err;
  struct Int32Quat *att_quat = stateGetNedToBodyQuat_i();
//   INT32_QUAT_INV_COMP(att_err, *att_quat, stab_att_sp_quat);
  int32_quat_inv_comp(&att_err, att_quat, &stab_att_sp_quat);
  /* wrap it in the shortest direction       */
  int32_quat_wrap_shortest(&att_err);
  int32_quat_normalize(&att_err);

  /* compute the INDI command */
  attitude_run_indi(stabilization_att_indi_cmd, &att_err, enable_integrator);

  stabilization_cmd[COMMAND_ROLL] = stabilization_att_indi_cmd[COMMAND_ROLL];
  stabilization_cmd[COMMAND_PITCH] = stabilization_att_indi_cmd[COMMAND_PITCH];
  stabilization_cmd[COMMAND_YAW] = stabilization_att_indi_cmd[COMMAND_YAW];

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
}

// This function reads rc commands
void stabilization_attitude_read_rc(bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn)
{
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}

// This is a simple second order low pass filter
void stabilization_indi_second_order_filter(struct FloatRates *input, struct FloatRates *filter_ddx,
    struct FloatRates *filter_dx, struct FloatRates *filter_x, float omega, float zeta, float omega_r)
{
  float_rates_integrate_fi(filter_x, filter_dx, 1.0 / PERIODIC_FREQUENCY);
  float_rates_integrate_fi(filter_dx, filter_ddx, 1.0 / PERIODIC_FREQUENCY);
  float omega2 = omega * omega;
  float omega2_r = omega_r * omega_r;

  filter_ddx->p = -filter_dx->p * 2 * zeta * omega   + (input->p - filter_x->p) * omega2;    \
  filter_ddx->q = -filter_dx->q * 2 * zeta * omega   + (input->q - filter_x->q) * omega2;    \
  filter_ddx->r = -filter_dx->r * 2 * zeta * omega_r + (input->r - filter_x->r) * omega2_r;
}

// This is a Least Mean Squares adaptive filter
// It estiamtes the actuator effectiveness online by comparing the expected angular acceleration based on the inputs with the measured angular acceleration
void lms_estimation(void)
{

  // Only pass really low frequencies so you don't adapt to noise
  float omega = 10.0;
  float zeta = 0.8;
  stabilization_indi_second_order_filter(&indi.u_act_dyn, &udotdot_estimation, &udot_estimation, &indi_u_estimation,
                                         omega, zeta, omega);
  struct FloatRates *body_rates = stateGetBodyRates_f();
  stabilization_indi_second_order_filter(body_rates, &filtered_rate_2deriv_estimation, &filtered_rate_deriv_estimation,
                                         &filtered_rate_estimation, omega, zeta, omega);

  // The inputs are scaled in order to avoid overflows
  float du = udot_estimation.p * INDI_EST_SCALE;
  g_est.p = g_est.p - (g_est.p * du - filtered_rate_2deriv_estimation.p) * du * mu;
  du = udot_estimation.q * INDI_EST_SCALE;
  g_est.q = g_est.q - (g_est.q * du - filtered_rate_2deriv_estimation.q) * du * mu;
  du = udot_estimation.r * INDI_EST_SCALE;
  float ddu = udotdot_estimation.r * INDI_EST_SCALE / PERIODIC_FREQUENCY;
  float error = (g_est.r * du + g2_est * ddu - filtered_rate_2deriv_estimation.r);
  g_est.r = g_est.r - error * du * mu / 3;
  g2_est = g2_est - error * 1000 * ddu * mu / 3;

  //the g values should be larger than zero, otherwise there is positive feedback, the command will go to max and there is nothing to learn anymore...
  if (g_est.p < 0.01) { g_est.p = 0.01; }
  if (g_est.q < 0.01) { g_est.q = 0.01; }
  if (g_est.r < 0.01) { g_est.r = 0.01; }
  if (g2_est < 0.01) { g2_est = 0.01; }

  if (use_adaptive_indi) {
    //Commit the estimated G values and apply the scaling
    g1.p = g_est.p * INDI_EST_SCALE;
    g1.q = g_est.q * INDI_EST_SCALE;
    g1.r = g_est.r * INDI_EST_SCALE;
    g2 = g2_est * INDI_EST_SCALE;
  }
}
