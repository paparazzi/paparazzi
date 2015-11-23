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
 * Come to ICRA2016 to learn more!
 *
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

float guidance_indi_pos_gain = 0.5;
float guidance_indi_speed_gain = 1.8;
struct FloatVect3 sp_accel = {0.0,0.0,0.0};

struct FloatVect3 filt_accel_ned;
struct FloatVect3 filt_accel_ned_d;
struct FloatVect3 filt_accel_ned_dd;
float filt_accelzbody = 0;
float filt_accelzbodyd = 0;
float filt_accelzbodydd = 0;
float roll_filt = 0;
float roll_filtd = 0;
float roll_filtdd = 0;
float pitch_filt = 0;
float pitch_filtd = 0;
float pitch_filtdd = 0;

struct FloatMat33 Ga;
struct FloatMat33 Ga_inv;
struct FloatVect3 euler_cmd;

float filter_omega = 20.0;
float filter_zeta = 0.65;

struct FloatEulers guidance_euler_cmd;

void guidance_indi_enter(void) {
  filt_accelzbody = 0;
  filt_accelzbodyd = 0;
  filt_accelzbodydd = 0;
  roll_filt = 0;
  roll_filtd = 0;
  roll_filtdd = 0;
  pitch_filt = 0;
  pitch_filtd = 0;
  pitch_filtdd = 0;
  FLOAT_VECT3_ZERO(filt_accel_ned);
  FLOAT_VECT3_ZERO(filt_accel_ned_d);
  FLOAT_VECT3_ZERO(filt_accel_ned_dd);
}

void guidance_indi_run(bool_t in_flight, int32_t heading) {

  //filter accel to get rid of noise
  //filter attitude to synchronize with accel
  guidance_indi_filter_attitude();
  guidance_indi_filter_accel();

  float pos_x_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.x) - stateGetPositionNed_f()->x; //+-ov.y/ (STABILIZATION_ATTITUDE_SP_MAX_THETA)*3.0;
  float pos_y_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.y) - stateGetPositionNed_f()->y; //+ ov.x/ (STABILIZATION_ATTITUDE_SP_MAX_PHI)*3.0;

  float speed_sp_x = pos_x_err*guidance_indi_pos_gain;
  float speed_sp_y = pos_y_err*guidance_indi_pos_gain;

  sp_accel.x = (speed_sp_x - stateGetSpeedNed_f()->x)*guidance_indi_speed_gain;
  sp_accel.y = (speed_sp_y - stateGetSpeedNed_f()->y)*guidance_indi_speed_gain;
//   sp_accel.x = (radio_control.values[RADIO_PITCH]/9600.0)*8.0;
//   sp_accel.y = -(radio_control.values[RADIO_ROLL]/9600.0)*8.0;

  //   struct FloatMat33 Ga;
  guidance_indi_calcG(&Ga);
  MAT33_INV(Ga_inv, Ga);

  float altitude_sp = POS_FLOAT_OF_BFP(guidance_v_z_sp);
  float vertical_velocity_sp = guidance_indi_pos_gain*(altitude_sp - stateGetPositionNed_f()->z);
//     float vertical_velocity_rc_euler = -(stabilization_cmd[COMMAND_THRUST]-4500.0)/4500.0*2.0;
  float vertical_velocity_err_euler = vertical_velocity_sp - stateGetSpeedNed_f()->z;
  sp_accel.z = vertical_velocity_err_euler*guidance_indi_speed_gain;

  struct FloatVect3 a_diff = { sp_accel.x - filt_accel_ned.x, sp_accel.y -filt_accel_ned.y, 0.0};

  Bound(a_diff.x, -6.0, 6.0);
  Bound(a_diff.y, -6.0, 6.0);
  Bound(a_diff.z, -9.0, 9.0);

  MAT33_VECT3_MUL(euler_cmd, Ga_inv, a_diff);

  guidance_euler_cmd.phi = roll_filt + euler_cmd.x;
  guidance_euler_cmd.theta = pitch_filt + euler_cmd.y;
  guidance_euler_cmd.psi = 0;//stateGetNedToBodyEulers_f()->psi;

  //Bound euler angles to prevent flipping and keep upright
  Bound(guidance_euler_cmd.phi, -GUIDANCE_H_MAX_BANK, GUIDANCE_H_MAX_BANK);
  Bound(guidance_euler_cmd.theta, -GUIDANCE_H_MAX_BANK, GUIDANCE_H_MAX_BANK);

  stabilization_attitude_set_setpoint_rp_quat_f(in_flight, heading);

}

//low pass the accelerometer measurements with a second order filter to remove noise from vibrations
void guidance_indi_filter_accel(void)
{
  VECT3_ADD_SCALED(filt_accel_ned, filt_accel_ned_d, 1.0/PERIODIC_FREQUENCY);
//   filt_accelzbody = filt_accelzbody + filt_accelzbodyd / PERIODIC_FREQUENCY; //also do body z accel

  VECT3_ADD_SCALED(filt_accel_ned_d, filt_accel_ned_dd, 1.0/PERIODIC_FREQUENCY);
//   filt_accelzbodyd = filt_accelzbodyd + filt_accelzbodydd / PERIODIC_FREQUENCY; //also do body z accel

  filt_accel_ned_dd.x = -filt_accel_ned_d.x * 2 * filter_zeta * filter_omega + (stateGetAccelNed_f()->x - filt_accel_ned.x) * filter_omega*filter_omega;
  filt_accel_ned_dd.y = -filt_accel_ned_d.y * 2 * filter_zeta * filter_omega + (stateGetAccelNed_f()->y - filt_accel_ned.y) * filter_omega*filter_omega;
  filt_accel_ned_dd.z = -filt_accel_ned_d.z * 2 * filter_zeta * filter_omega + (stateGetAccelNed_f()->z - filt_accel_ned.z) * filter_omega*filter_omega;
//   filt_accelzbodydd= -filt_accelzbodyd * 2 * filter_zeta * filter_omega + (accel_meas_body_f.z - filt_accelzbody) * filter_omega*filter_omega;
}

void guidance_indi_filter_attitude(void)
{
  roll_filt = roll_filt + roll_filtd / PERIODIC_FREQUENCY;
  pitch_filt = pitch_filt + pitch_filtd / PERIODIC_FREQUENCY;

  roll_filtd = roll_filtd + roll_filtdd / PERIODIC_FREQUENCY;
  pitch_filtd = pitch_filtd + pitch_filtdd / PERIODIC_FREQUENCY;
//   float cospsi = cosf(stateGetNedToBodyEulers_f()->psi);
//   float sinpsi = sinf(stateGetNedToBodyEulers_f()->psi);

  roll_filtdd = -roll_filtd * 2 * filter_zeta * filter_omega + (stateGetNedToBodyEulers_f()->phi - roll_filt) * filter_omega*filter_omega;
  pitch_filtdd = -pitch_filtd * 2 * filter_zeta * filter_omega + (stateGetNedToBodyEulers_f()->theta - pitch_filt) * filter_omega*filter_omega;
}

void guidance_indi_calcG(struct FloatMat33 *Gmat) {

  struct FloatEulers *euler = stateGetNedToBodyEulers_f();

  float sphi = sinf(euler->phi);
  float cphi = cosf(euler->phi);
  float stheta = sinf(euler->theta);
  float ctheta = cosf(euler->theta);
  float spsi = sinf(euler->psi);
  float cpsi = cosf(euler->psi);
  float T = -9.81; //minus gravity is a good guesstimate of the thrust force
//   float T = (filt_accelzn-9.81)/(cphi*ctheta); //calculate specific force in body z axis by using the accelerometer
//   float T = filt_accelzbody; //if body acceleration is available, use that!

  RMAT_ELMT(*Gmat, 0, 0) = (cphi*spsi - sphi*cpsi*stheta)*T;
  RMAT_ELMT(*Gmat, 1, 0) = (-sphi*spsi*stheta - cpsi*cphi)*T;
  RMAT_ELMT(*Gmat, 2, 0) = -ctheta*sphi*T;
  RMAT_ELMT(*Gmat, 0, 1) = (cphi*cpsi*ctheta)*T;
  RMAT_ELMT(*Gmat, 1, 1) = (cphi*spsi*ctheta)*T;
  RMAT_ELMT(*Gmat, 2, 1) = -stheta*cphi*T;
  RMAT_ELMT(*Gmat, 0, 2) = sphi*spsi + cphi*cpsi*stheta;
  RMAT_ELMT(*Gmat, 1, 2) = cphi*spsi*stheta - cpsi*sphi;
  RMAT_ELMT(*Gmat, 2, 2) = cphi*ctheta;
}

void stabilization_attitude_set_setpoint_rp_quat_f(bool_t in_flight, int32_t heading)
{
  struct FloatQuat q_rp_cmd;
  float_quat_of_eulers(&q_rp_cmd, &guidance_euler_cmd); //TODO this is a quaternion without yaw! add the desired yaw before you use it!

  /* get current heading */
  const struct FloatVect3 zaxis = {0., 0., 1.};
  struct FloatQuat q_yaw;

  float_quat_of_axis_angle(&q_yaw, &zaxis, stateGetNedToBodyEulers_f()->psi);

  /* roll/pitch commands applied to to current heading */
  struct FloatQuat q_rp_sp;
  float_quat_comp(&q_rp_sp, &q_yaw, &q_rp_cmd);
  float_quat_normalize(&q_rp_sp);

  struct FloatQuat q_sp;

  if (in_flight) {
    /* get current heading setpoint */
    struct FloatQuat q_yaw_sp;
    float_quat_of_axis_angle(&q_yaw_sp, &zaxis, ANGLE_FLOAT_OF_BFP(heading));


    /* rotation between current yaw and yaw setpoint */
    struct FloatQuat q_yaw_diff;
    float_quat_comp_inv(&q_yaw_diff, &q_yaw_sp, &q_yaw);

    /* compute final setpoint with yaw */
    float_quat_comp_norm_shortest(&q_sp, &q_rp_sp, &q_yaw_diff);
  } else {
    QUAT_COPY(q_sp, q_rp_sp);
  }

  QUAT_BFP_OF_REAL(stab_att_sp_quat,q_sp);
}
