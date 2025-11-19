/*
 * Copyright (C) 2015 Ewoud Smeur <ewoud.smeur@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi.c
 *
 * A guidance mode based on Incremental Nonlinear Dynamic Inversion
 *
 * Based on the papers:
 * Cascaded Incremental Nonlinear Dynamic Inversion Control for MAV Disturbance Rejection
 * https://www.researchgate.net/publication/312907985_Cascaded_Incremental_Nonlinear_Dynamic_Inversion_Control_for_MAV_Disturbance_Rejection
 *
 * Gust Disturbance Alleviation with Incremental Nonlinear Dynamic Inversion
 * https://www.researchgate.net/publication/309212603_Gust_Disturbance_Alleviation_with_Incremental_Nonlinear_Dynamic_Inversion
 */
#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_indi.h"
#include "modules/radio_control/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "mcu_periph/sys_time.h"
#include "state.h"
#include "autopilot.h"
#include "filters/low_pass_filter.h"
#include "modules/core/abi.h"

// The acceleration reference is calculated with these gains. If you use GPS,
// they are probably limited by the update rate of your GPS. The default
// values are tuned for 4 Hz GPS updates. If you have high speed position updates, the
// gains can be higher, depending on the speed of the inner loop.
#ifdef GUIDANCE_INDI_POS_GAIN
float guidance_indi_pos_gain = GUIDANCE_INDI_POS_GAIN;
#else
float guidance_indi_pos_gain = 0.5;
#endif

#ifdef GUIDANCE_INDI_SPEED_GAIN
float guidance_indi_speed_gain = GUIDANCE_INDI_SPEED_GAIN;
#else
float guidance_indi_speed_gain = 1.8;
#endif

#ifndef GUIDANCE_INDI_ACCEL_SP_ID
#define GUIDANCE_INDI_ACCEL_SP_ID ABI_BROADCAST
#endif
abi_event accel_sp_ev;
static void accel_sp_cb(uint8_t sender_id, uint8_t flag, struct FloatVect3 *accel_sp);
struct FloatVect3 indi_accel_sp = {0.0f, 0.0f, 0.0f};
bool indi_accel_sp_set_2d = false;
bool indi_accel_sp_set_3d = false;

// FIXME make a proper structure for these variables
struct FloatVect3 speed_sp = {0.0f, 0.0f, 0.0f};
struct FloatVect3 sp_accel = {0.0f, 0.0f, 0.0f};
#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
float guidance_indi_specific_force_gain = GUIDANCE_INDI_SPECIFIC_FORCE_GAIN;
static void guidance_indi_filter_thrust(void);

#ifdef GUIDANCE_INDI_THRUST_DYNAMICS
#warning GUIDANCE_INDI_THRUST_DYNAMICS is deprecated, use GUIDANCE_INDI_THRUST_DYNAMICS_FREQ instead.
#warning "The thrust dynamics are now specified in continuous time with the corner frequency of the first order model!"
#warning "define GUIDANCE_INDI_THRUST_DYNAMICS_FREQ in rad/s"
#warning "Use -ln(1 - old_number) * PERIODIC_FREQUENCY to compute it from the old value."
#endif

#ifndef GUIDANCE_INDI_THRUST_DYNAMICS_FREQ
#ifndef STABILIZATION_INDI_ACT_FREQ_P
#error "You need to define GUIDANCE_INDI_THRUST_DYNAMICS_FREQ to be able to use indi vertical control"
#else // assume that the same actuators are used for thrust as for roll (e.g. quadrotor)
#define GUIDANCE_INDI_THRUST_DYNAMICS_FREQ STABILIZATION_INDI_ACT_FREQ_P
#endif
#endif //GUIDANCE_INDI_THRUST_DYNAMICS_FREQ


#endif //GUIDANCE_INDI_SPECIFIC_FORCE_GAIN

#ifndef GUIDANCE_INDI_FILTER_CUTOFF
#ifdef STABILIZATION_INDI_FILT_CUTOFF
#define GUIDANCE_INDI_FILTER_CUTOFF STABILIZATION_INDI_FILT_CUTOFF
#else
#define GUIDANCE_INDI_FILTER_CUTOFF 3.0
#endif
#endif

float thrust_dyn = 0.f;
float thrust_act = 0.f;
Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass yaw_filt;
Butterworth2LowPass thrust_filt;

static float Gmat[GUIDANCE_INDI_NV][GUIDANCE_INDI_NU];

#if GUIDANCE_INDI_USE_WLS
#include "math/wls/wls_alloc.h"

#define NU_MAX 6    // Example constant value // [dtheta, dphi, dthrust, dtx , dty]
#define NV_MAX 3    // Example constant value (ax,ay,az)
static float du_guidance[GUIDANCE_INDI_NU];
static float *Bwls_gi[GUIDANCE_INDI_NV];

static struct WLS_t wls_guid_p = {
  .nu        = GUIDANCE_INDI_NU,
  .nv        = GUIDANCE_INDI_NV,
  .gamma_sq  = 1000.0,
  .v         = {0.0},
  .Wv        = {100.f, 100.f, 100.f},         // x,y,z
  .Wu        = {10.f,10.f,0.f,0.f,0.f,10.f}, // minimize the control input (thetq,phi,Tx,Ty,Tz,psi)
  .u_pref    = {0.0},
  .u_min     = {0.0},
  .u_max     = {0.0},
  .PC        = 0.0,
  .SC        = 0.0,
  .iter      = 0
};

#else

// inverse matrix directly if not using WLS
struct FloatMat33 Ga;
struct FloatMat33 Ga_inv;
struct FloatVect3 control_increment; // [dtheta, dphi, dthrust]

#endif

float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;
float guidance_indi_max_bank = GUIDANCE_H_MAX_BANK;

float time_of_accel_sp_2d = 0.0;
float time_of_accel_sp_3d = 0.0;

struct FloatEulers guidance_euler_cmd;
struct ThrustSetpoint thrust_sp;
float thrust_in;
float thrust_vect[3];

static void guidance_indi_propagate_filters(struct FloatEulers *eulers);

//------------------------------------------------------------//

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_indi_guidance(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE_INDI_HYBRID(trans, dev, AC_ID,
      &sp_accel.x,
      &sp_accel.y,
      &sp_accel.z,
#if GUIDANCE_INDI_USE_WLS
      &du_guidance[0],
      &du_guidance[1],
      &du_guidance[2],
#else
      &control_increment.x,
      &control_increment.y,
      &control_increment.z,
#endif
      &filt_accel_ned[0].o[0],
      &filt_accel_ned[1].o[0],
      &filt_accel_ned[2].o[0],
      &speed_sp.x,
      &speed_sp.y,
      &speed_sp.z);
}
#endif

/**
 * @brief Init function
 */
void guidance_indi_init(void)
{
  FLOAT_EULERS_ZERO(guidance_euler_cmd);
  THRUST_SP_SET_ZERO(thrust_sp);
  AbiBindMsgACCEL_SP(GUIDANCE_INDI_ACCEL_SP_ID, &accel_sp_ev, accel_sp_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_INDI_HYBRID, send_indi_guidance);
#endif
}

/**
 *
 * Call upon entering indi guidance
 */
void guidance_indi_enter(void)
{
  /* set nav_heading to current heading */
  nav.heading = stateGetNedToBodyEulers_f()->psi;

  thrust_in = stabilization.cmd[COMMAND_THRUST];
  thrust_act = thrust_in;

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
#ifdef GUIDANCE_INDI_THRUST_DYNAMICS
  thrust_dyn = GUIDANCE_INDI_THRUST_DYNAMICS;
#else
  thrust_dyn = 1-exp(-GUIDANCE_INDI_THRUST_DYNAMICS_FREQ/PERIODIC_FREQUENCY);
#endif //GUIDANCE_INDI_THRUST_DYNAMICS
#endif //GUIDANCE_INDI_SPECIFIC_FORCE_GAIN

  float tau = 1.0 / (2.0 * M_PI * filter_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  for (int8_t i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, stateGetNedToBodyEulers_f()->phi);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, stateGetNedToBodyEulers_f()->theta);
  init_butterworth_2_low_pass(&yaw_filt, tau, sample_time, stateGetNedToBodyEulers_f()->psi);
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, thrust_in);
}

/**
 * @param accel_sp accel setpoint in NED frame [m/s^2]
 * @param heading_sp the desired heading [rad]
 * @return stabilization setpoint structure
 *
 * main indi guidance function
 */
struct StabilizationSetpoint guidance_indi_run(struct FloatVect3 *accel_sp, float heading_sp)
{
  struct FloatEulers euler_yxz;
  struct FloatQuat * statequat = stateGetNedToBodyQuat_f();
  float_eulers_of_quat_yxz(&euler_yxz, statequat);

  // set global accel sp variable FIXME clean this
  sp_accel = *accel_sp;

  //filter accel to get rid of noise and filter attitude to synchronize with accel
  guidance_indi_propagate_filters(&euler_yxz);

  struct FloatVect3 a_diff = {
    sp_accel.x - filt_accel_ned[0].o[0],
    sp_accel.y - filt_accel_ned[1].o[0],
    sp_accel.z - filt_accel_ned[2].o[0]
  };
  Bound(a_diff.x, -6.0, 6.0);
  Bound(a_diff.y, -6.0, 6.0);
  Bound(a_diff.z, -9.0, 9.0);

  //If the thrust to specific force ratio has been defined, include vertical control
  //else ignore the vertical acceleration error
#ifndef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
#ifndef STABILIZATION_ATTITUDE_INDI_FULL
  a_diff.z = 0.0;
#endif
#endif

#if GUIDANCE_INDI_USE_WLS
  float m = GUIDANCE_INDI_MASS;
  wls_guid_p.v[0] = m*a_diff.x;
  wls_guid_p.v[1] = m*a_diff.y;
  wls_guid_p.v[2] = m*a_diff.z;

  guidance_indi_set_wls_settings(&wls_guid_p, &euler_yxz, heading_sp);
  guidance_indi_calcG(Gmat, euler_yxz);
  for (int i = 0; i < GUIDANCE_INDI_NV; i++) {
    Bwls_gi[i] = Gmat[i];
  }
  wls_alloc(&wls_guid_p,Bwls_gi, 0, 0, 10);
  for (int i = 0; i < GUIDANCE_INDI_NU; i++) {
    du_guidance [i] = wls_guid_p.u[i];
  }

  guidance_euler_cmd.phi   = roll_filt.o[0]  + du_guidance[0];
  guidance_euler_cmd.theta = pitch_filt.o[0] + du_guidance[1];
  guidance_euler_cmd.psi   = yaw_filt.o[0]   + du_guidance[2];
  thrust_vect[0] = du_guidance[3]; // (TX)
  thrust_vect[1] = du_guidance[4]; // (TY)
  thrust_vect[2] = du_guidance[5]; // (TZ)
  thrust_sp = th_sp_from_incr_vect_f(thrust_vect);

#else // !USE_WLS

  // Calculate matrix of partial derivatives
  guidance_indi_calcG(Gmat, euler_yxz);

  RMAT_ELMT(Ga, 0, 0) = Gmat[0][0];
  RMAT_ELMT(Ga, 1, 0) = Gmat[1][0];
  RMAT_ELMT(Ga, 2, 0) = Gmat[2][0];
  RMAT_ELMT(Ga, 0, 1) = Gmat[0][1];
  RMAT_ELMT(Ga, 1, 1) = Gmat[1][1];
  RMAT_ELMT(Ga, 2, 1) = Gmat[2][1];
  RMAT_ELMT(Ga, 0, 2) = Gmat[0][2];
  RMAT_ELMT(Ga, 1, 2) = Gmat[1][2];
  RMAT_ELMT(Ga, 2, 2) = Gmat[2][2];
  // Invert this matrix // FIXME input format
  MAT33_INV(Ga_inv, Ga);
  // Calculate roll,pitch and thrust command
  MAT33_VECT3_MUL(control_increment, Ga_inv, a_diff);

  guidance_euler_cmd.theta = pitch_filt.o[0] + control_increment.x;
  guidance_euler_cmd.phi   = roll_filt.o[0]  + control_increment.y;
  guidance_euler_cmd.psi   = heading_sp;

  // Compute and store thust setpoint
#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
  guidance_indi_filter_thrust();
  //Add the increment in specific force * specific_force_to_thrust_gain to the filtered thrust
  thrust_in = thrust_filt.o[0] + control_increment.z * guidance_indi_specific_force_gain;
  Bound(thrust_in, 0, MAX_PPRZ);
  thrust_sp = th_sp_from_thrust_i(thrust_in, THRUST_AXIS_Z);

#else
  // basic quad, no side force
  thrust_vect[0] = 0.0f;
  thrust_vect[1] = 0.0f;
  thrust_vect[2] = control_increment.z;

  // specific force not defined, return required increment
  thrust_sp = th_sp_from_incr_vect_f(thrust_vect);
#endif

#endif // USE_WLS

  // Bound euler angles to prevent flipping
  Bound(guidance_euler_cmd.phi, -guidance_indi_max_bank, guidance_indi_max_bank);
  Bound(guidance_euler_cmd.theta, -guidance_indi_max_bank, guidance_indi_max_bank);

  // set the quat setpoint with the calculated roll and pitch
  struct FloatQuat q_sp;
  float_quat_of_eulers_yxz(&q_sp, &guidance_euler_cmd);

  return stab_sp_from_quat_f(&q_sp);
}

struct FloatVect3 WEAK guidance_indi_controller(bool in_flight UNUSED, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceIndi_HMode h_mode, enum GuidanceIndi_VMode v_mode)
{
  struct FloatVect3 pos_err = { 0 };
  struct FloatVect3 accel_sp = { 0 };

  struct FloatVect3 speed_fb;

  if (h_mode == GUIDANCE_INDI_H_ACCEL) {
    // Speed feedback is included in the guidance when running in ACCEL mode
    speed_fb.x = 0.;
    speed_fb.y = 0.;
  }
  else {
    // Generate speed feedback for acceleration, as it is estimated
    if (h_mode == GUIDANCE_INDI_H_SPEED) {
      speed_sp.x = SPEED_FLOAT_OF_BFP(gh->ref.speed.x);
      speed_sp.y = SPEED_FLOAT_OF_BFP(gh->ref.speed.y);
    }
    else { // H_POS
      pos_err.x = POS_FLOAT_OF_BFP(gh->ref.pos.x) - stateGetPositionNed_f()->x;
      pos_err.y = POS_FLOAT_OF_BFP(gh->ref.pos.y) - stateGetPositionNed_f()->y;
      speed_sp.x = pos_err.x * guidance_indi_pos_gain + SPEED_FLOAT_OF_BFP(gh->ref.speed.x);
      speed_sp.y = pos_err.y * guidance_indi_pos_gain + SPEED_FLOAT_OF_BFP(gh->ref.speed.y);
    }
    speed_fb.x = (speed_sp.x - stateGetSpeedNed_f()->x) * guidance_indi_speed_gain;
    speed_fb.y = (speed_sp.y - stateGetSpeedNed_f()->y) * guidance_indi_speed_gain;
  }

  if (v_mode == GUIDANCE_INDI_V_ACCEL)  {
    // Speed feedback is included in the guidance when running in ACCEL mode
    speed_fb.z = 0;
  }
  else {
    // Generate speed feedback for acceleration, as it is estimated
    if (v_mode == GUIDANCE_INDI_V_SPEED) {
      speed_sp.z = SPEED_FLOAT_OF_BFP(gv->zd_ref);
    }
    else { // V_POS
      pos_err.z = POS_FLOAT_OF_BFP(gv->z_ref) - stateGetPositionNed_f()->z;
      speed_sp.z = pos_err.z * guidance_indi_pos_gain + SPEED_FLOAT_OF_BFP(gv->zd_ref);
    }
    speed_fb.z = (speed_sp.z - stateGetSpeedNed_f()->z) * guidance_indi_speed_gain;
  }

  accel_sp.x = speed_fb.x + ACCEL_FLOAT_OF_BFP(gh->ref.accel.x);
  accel_sp.y = speed_fb.y + ACCEL_FLOAT_OF_BFP(gh->ref.accel.y);
  accel_sp.z = speed_fb.z + ACCEL_FLOAT_OF_BFP(gv->zdd_ref);

  return accel_sp;
}

struct StabilizationSetpoint guidance_indi_run_mode(bool in_flight, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceIndi_HMode h_mode, enum GuidanceIndi_VMode v_mode)
{
  struct FloatVect3 accel_sp;

  // If set and valid, the ABI message overwrite the accel setpoint
  // TODO This is not ideal, guided mode with accel setpoint would be better
  if (indi_accel_sp_set_2d) {
    accel_sp.x = indi_accel_sp.x;
    accel_sp.y = indi_accel_sp.y;
    // In 2D the vertical motion is derived from the flight plan
    accel_sp.z = (speed_sp.z - stateGetSpeedNed_f()->z) * guidance_indi_speed_gain;
    float dt = get_sys_time_float() - time_of_accel_sp_2d;
    // If the input command is not updated after a timeout, switch back to flight plan control
    if (dt > 0.5) {
      indi_accel_sp_set_2d = false;
    }
  } else if (indi_accel_sp_set_3d) {
    accel_sp.x = indi_accel_sp.x;
    accel_sp.y = indi_accel_sp.y;
    accel_sp.z = indi_accel_sp.z;
    float dt = get_sys_time_float() - time_of_accel_sp_3d;
    // If the input command is not updated after a timeout, switch back to flight plan control
    if (dt > 0.5) {
      indi_accel_sp_set_3d = false;
    }
  }
  else {
    accel_sp = guidance_indi_controller(in_flight, gh, gv, h_mode, v_mode);
  }

  return guidance_indi_run(&accel_sp, gh->sp.heading);
}


#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
/**
 * Filter the thrust, such that it corresponds to the filtered acceleration
 */
void guidance_indi_filter_thrust(void)
{
  // Actuator dynamics
  thrust_act = thrust_act + thrust_dyn * (thrust_in - thrust_act);

  // same filter as for the acceleration
  update_butterworth_2_low_pass(&thrust_filt, thrust_act);
}
#endif

/**
 * Low pass the accelerometer measurements to remove noise from vibrations.
 * The roll and pitch also need to be filtered to synchronize them with the
 * acceleration
 */
void guidance_indi_propagate_filters(struct FloatEulers *eulers)
{
  struct NedCoor_f *accel = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&filt_accel_ned[0], accel->x);
  update_butterworth_2_low_pass(&filt_accel_ned[1], accel->y);
  update_butterworth_2_low_pass(&filt_accel_ned[2], accel->z);

  update_butterworth_2_low_pass(&roll_filt, eulers->phi);
  update_butterworth_2_low_pass(&pitch_filt, eulers->theta);
  update_butterworth_2_low_pass(&yaw_filt, eulers->psi);
}

/**
 * ABI callback that obtains the acceleration setpoint from telemetry
 * flag: 0 -> 2D, 1 -> 3D
 */
static void accel_sp_cb(uint8_t sender_id __attribute__((unused)), uint8_t flag, struct FloatVect3 *accel_sp)
{
  if (flag == 0) {
    indi_accel_sp.x = accel_sp->x;
    indi_accel_sp.y = accel_sp->y;
    indi_accel_sp_set_2d = true;
    time_of_accel_sp_2d = get_sys_time_float();
  } else if (flag == 1) {
    indi_accel_sp.x = accel_sp->x;
    indi_accel_sp.y = accel_sp->y;
    indi_accel_sp.z = accel_sp->z;
    indi_accel_sp_set_3d = true;
    time_of_accel_sp_3d = get_sys_time_float();
  }
}

#if GUIDANCE_INDI_USE_AS_DEFAULT
// guidance indi control function is implementing the default functions of guidance

void guidance_h_run_enter(void)
{
  guidance_indi_enter();
}

void guidance_v_run_enter(void)
{
  // nothing to do
}

static struct VerticalGuidance *_gv = &guidance_v;
static enum GuidanceIndi_VMode _v_mode = GUIDANCE_INDI_V_POS;

struct StabilizationSetpoint guidance_h_run_pos(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_H_POS, _v_mode);
}

struct StabilizationSetpoint guidance_h_run_speed(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_H_SPEED, _v_mode);
}

struct StabilizationSetpoint guidance_h_run_accel(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_indi_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_H_ACCEL, _v_mode);
}

struct ThrustSetpoint guidance_v_run_pos(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_INDI_V_POS;
  return thrust_sp;
}

struct ThrustSetpoint guidance_v_run_speed(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_INDI_V_SPEED;
  return thrust_sp;
}

struct ThrustSetpoint guidance_v_run_accel(bool in_flight UNUSED, struct VerticalGuidance *gv)
{
  _gv = gv;
  _v_mode = GUIDANCE_INDI_V_ACCEL;
  return thrust_sp;
}

#endif

