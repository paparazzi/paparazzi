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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
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
#include "subsystems/ins/ins_int.h"
#include "subsystems/radio_control.h"
#include "state.h"
#include "subsystems/imu.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "filters/low_pass_filter.h"
#include "subsystems/abi.h"

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
struct FloatVect3 indi_accel_sp = {0.0, 0.0, 0.0};
bool indi_accel_sp_set_2d = false;
bool indi_accel_sp_set_3d = false;

struct FloatVect3 sp_accel = {0.0, 0.0, 0.0};
#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
float thrust_in_specific_force_gain = GUIDANCE_INDI_SPECIFIC_FORCE_GAIN;
static void guidance_indi_filter_thrust(void);

#ifndef GUIDANCE_INDI_THRUST_DYNAMICS
#ifndef STABILIZATION_INDI_ACT_DYN_P
#error "You need to define GUIDANCE_INDI_THRUST_DYNAMICS to be able to use indi vertical control"
#else // assume that the same actuators are used for thrust as for roll (e.g. quadrotor)
#define GUIDANCE_INDI_THRUST_DYNAMICS STABILIZATION_INDI_ACT_DYN_P
#endif
#endif //GUIDANCE_INDI_THRUST_DYNAMICS

#endif //GUIDANCE_INDI_SPECIFIC_FORCE_GAIN

#ifndef GUIDANCE_INDI_FILTER_CUTOFF
#ifdef STABILIZATION_INDI_FILT_CUTOFF
#define GUIDANCE_INDI_FILTER_CUTOFF STABILIZATION_INDI_FILT_CUTOFF
#else
#define GUIDANCE_INDI_FILTER_CUTOFF 3.0
#endif
#endif

float thrust_act = 0;
Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass thrust_filt;

struct FloatMat33 Ga;
struct FloatMat33 Ga_inv;
struct FloatVect3 control_increment; // [dtheta, dphi, dthrust]

float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;
float guidance_indi_max_bank = GUIDANCE_H_MAX_BANK;

float time_of_accel_sp_2d = 0.0;
float time_of_accel_sp_3d = 0.0;

struct FloatEulers guidance_euler_cmd;
float thrust_in;

static void guidance_indi_propagate_filters(struct FloatEulers *eulers);
static void guidance_indi_calcG(struct FloatMat33 *Gmat);
static void guidance_indi_calcG_yxz(struct FloatMat33 *Gmat, struct FloatEulers *euler_yxz);

/**
 * @brief Init function
 */
void guidance_indi_init(void)
{
  AbiBindMsgACCEL_SP(GUIDANCE_INDI_ACCEL_SP_ID, &accel_sp_ev, accel_sp_cb);
}

/**
 *
 * Call upon entering indi guidance
 */
void guidance_indi_enter(void)
{
  thrust_in = stabilization_cmd[COMMAND_THRUST];
  thrust_act = thrust_in;

  float tau = 1.0 / (2.0 * M_PI * filter_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  for (int8_t i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, stateGetNedToBodyEulers_f()->phi);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, stateGetNedToBodyEulers_f()->theta);
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, thrust_in);
}

/**
 * @param heading_sp the desired heading [rad]
 *
 * main indi guidance function
 */
void guidance_indi_run(float heading_sp)
{
  struct FloatEulers eulers_yxz;
  struct FloatQuat * statequat = stateGetNedToBodyQuat_f();
  float_eulers_of_quat_yxz(&eulers_yxz, statequat);

  //filter accel to get rid of noise and filter attitude to synchronize with accel
  guidance_indi_propagate_filters(&eulers_yxz);

  //Linear controller to find the acceleration setpoint from position and velocity
  float pos_x_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.x) - stateGetPositionNed_f()->x;
  float pos_y_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.y) - stateGetPositionNed_f()->y;
  float pos_z_err = POS_FLOAT_OF_BFP(guidance_v_z_ref - stateGetPositionNed_i()->z);

  float speed_sp_x = pos_x_err * guidance_indi_pos_gain;
  float speed_sp_y = pos_y_err * guidance_indi_pos_gain;
  float speed_sp_z = pos_z_err * guidance_indi_pos_gain;

  // If the acceleration setpoint is set over ABI message
  if (indi_accel_sp_set_2d) {
    sp_accel.x = indi_accel_sp.x;
    sp_accel.y = indi_accel_sp.y;
    // In 2D the vertical motion is derived from the flight plan
    sp_accel.z = (speed_sp_z - stateGetSpeedNed_f()->z) * guidance_indi_speed_gain;
    float dt = get_sys_time_float() - time_of_accel_sp_2d;
    // If the input command is not updated after a timeout, switch back to flight plan control
    if (dt > 0.5) {
      indi_accel_sp_set_2d = false;
    }
  } else if (indi_accel_sp_set_3d) {
    sp_accel.x = indi_accel_sp.x;
    sp_accel.y = indi_accel_sp.y;
    sp_accel.z = indi_accel_sp.z;
    float dt = get_sys_time_float() - time_of_accel_sp_3d;
    // If the input command is not updated after a timeout, switch back to flight plan control
    if (dt > 0.5) {
      indi_accel_sp_set_3d = false;
    }
  } else {
    sp_accel.x = (speed_sp_x - stateGetSpeedNed_f()->x) * guidance_indi_speed_gain;
    sp_accel.y = (speed_sp_y - stateGetSpeedNed_f()->y) * guidance_indi_speed_gain;
    sp_accel.z = (speed_sp_z - stateGetSpeedNed_f()->z) * guidance_indi_speed_gain;
  }

#if GUIDANCE_INDI_RC_DEBUG
#warning "GUIDANCE_INDI_RC_DEBUG lets you control the accelerations via RC, but disables autonomous flight!"
  //for rc control horizontal, rotate from body axes to NED
  float psi = stateGetNedToBodyEulers_f()->psi;
  float rc_x = -(radio_control.values[RADIO_PITCH] / 9600.0) * 8.0;
  float rc_y = (radio_control.values[RADIO_ROLL] / 9600.0) * 8.0;
  sp_accel.x = cosf(psi) * rc_x - sinf(psi) * rc_y;
  sp_accel.y = sinf(psi) * rc_x + cosf(psi) * rc_y;

  //for rc vertical control
  sp_accel.z = -(radio_control.values[RADIO_THROTTLE] - 4500) * 8.0 / 9600.0;
#endif

  //Calculate matrix of partial derivatives
  guidance_indi_calcG_yxz(&Ga, &eulers_yxz);

  //Invert this matrix
  MAT33_INV(Ga_inv, Ga);

  struct FloatVect3 a_diff = { sp_accel.x - filt_accel_ned[0].o[0], sp_accel.y - filt_accel_ned[1].o[0], sp_accel.z - filt_accel_ned[2].o[0]};

  //Bound the acceleration error so that the linearization still holds
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

  //Calculate roll,pitch and thrust command
  MAT33_VECT3_MUL(control_increment, Ga_inv, a_diff);

  AbiSendMsgTHRUST(THRUST_INCREMENT_ID, control_increment.z);

  guidance_euler_cmd.theta = pitch_filt.o[0] + control_increment.x;
  guidance_euler_cmd.phi = roll_filt.o[0] + control_increment.y;
  guidance_euler_cmd.psi = heading_sp;

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
  guidance_indi_filter_thrust();

  //Add the increment in specific force * specific_force_to_thrust_gain to the filtered thrust
  thrust_in = thrust_filt.o[0] + control_increment.z * thrust_in_specific_force_gain;
  Bound(thrust_in, 0, 9600);

#if GUIDANCE_INDI_RC_DEBUG
  if (radio_control.values[RADIO_THROTTLE] < 300) {
    thrust_in = 0;
  }
#endif

  //Overwrite the thrust command from guidance_v
  stabilization_cmd[COMMAND_THRUST] = thrust_in;
#endif

  //Bound euler angles to prevent flipping
  Bound(guidance_euler_cmd.phi, -guidance_indi_max_bank, guidance_indi_max_bank);
  Bound(guidance_euler_cmd.theta, -guidance_indi_max_bank, guidance_indi_max_bank);

  //set the quat setpoint with the calculated roll and pitch
  struct FloatQuat q_sp;
  float_quat_of_eulers_yxz(&q_sp, &guidance_euler_cmd);
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
/**
 * Filter the thrust, such that it corresponds to the filtered acceleration
 */
void guidance_indi_filter_thrust(void)
{
  // Actuator dynamics
  thrust_act = thrust_act + GUIDANCE_INDI_THRUST_DYNAMICS * (thrust_in - thrust_act);

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
}

/**
 * @param Gmat array to write the matrix to [3x3]
 *
 * Calculate the matrix of partial derivatives of the pitch, roll and thrust.
 * w.r.t. the NED accelerations for YXZ eulers
 * ddx = G*[dtheta,dphi,dT]
 */
void guidance_indi_calcG_yxz(struct FloatMat33 *Gmat, struct FloatEulers *euler_yxz)
{

  float sphi = sinf(euler_yxz->phi);
  float cphi = cosf(euler_yxz->phi);
  float stheta = sinf(euler_yxz->theta);
  float ctheta = cosf(euler_yxz->theta);
  //minus gravity is a guesstimate of the thrust force, thrust measurement would be better
  float T = -9.81;

  RMAT_ELMT(*Gmat, 0, 0) = ctheta * cphi * T;
  RMAT_ELMT(*Gmat, 1, 0) = 0;
  RMAT_ELMT(*Gmat, 2, 0) = -stheta * cphi * T;
  RMAT_ELMT(*Gmat, 0, 1) = -stheta * sphi * T;
  RMAT_ELMT(*Gmat, 1, 1) = -cphi * T;
  RMAT_ELMT(*Gmat, 2, 1) = -ctheta * sphi * T;
  RMAT_ELMT(*Gmat, 0, 2) = stheta * cphi;
  RMAT_ELMT(*Gmat, 1, 2) = -sphi;
  RMAT_ELMT(*Gmat, 2, 2) = ctheta * cphi;
}

/**
 * @param Gmat array to write the matrix to [3x3]
 *
 * Calculate the matrix of partial derivatives of the roll, pitch and thrust.
 * w.r.t. the NED accelerations for ZYX eulers
 * ddx = G*[dtheta,dphi,dT]
 */
UNUSED void guidance_indi_calcG(struct FloatMat33 *Gmat)
{

  struct FloatEulers *euler = stateGetNedToBodyEulers_f();

  float sphi = sinf(euler->phi);
  float cphi = cosf(euler->phi);
  float stheta = sinf(euler->theta);
  float ctheta = cosf(euler->theta);
  float spsi = sinf(euler->psi);
  float cpsi = cosf(euler->psi);
  //minus gravity is a guesstimate of the thrust force, thrust measurement would be better
  float T = -9.81;

  RMAT_ELMT(*Gmat, 0, 0) = (cphi * spsi - sphi * cpsi * stheta) * T;
  RMAT_ELMT(*Gmat, 1, 0) = (-sphi * spsi * stheta - cpsi * cphi) * T;
  RMAT_ELMT(*Gmat, 2, 0) = -ctheta * sphi * T;
  RMAT_ELMT(*Gmat, 0, 1) = (cphi * cpsi * ctheta) * T;
  RMAT_ELMT(*Gmat, 1, 1) = (cphi * spsi * ctheta) * T;
  RMAT_ELMT(*Gmat, 2, 1) = -stheta * cphi * T;
  RMAT_ELMT(*Gmat, 0, 2) = sphi * spsi + cphi * cpsi * stheta;
  RMAT_ELMT(*Gmat, 1, 2) = cphi * spsi * stheta - cpsi * sphi;
  RMAT_ELMT(*Gmat, 2, 2) = cphi * ctheta;
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

