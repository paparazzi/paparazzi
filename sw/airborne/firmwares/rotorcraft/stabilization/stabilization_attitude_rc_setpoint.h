/*
 * Copyright (C) 2012 Felix Ruess <felix.ruess@gmail.com>
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

/** @file stabilization_attitude_rc_setpoint.h
 *  Read an attitude setpoint from the RC.
 */

#ifndef STABILIZATION_ATTITUDE_RC_SETPOINT_H
#define STABILIZATION_ATTITUDE_RC_SETPOINT_H

#include "std.h"
#include "generated/airframe.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

#include "subsystems/radio_control.h"
#include "state.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/imu.h"

#if defined STABILIZATION_ATTITUDE_TYPE_INT
#define SP_MAX_PHI     (int32_t)ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_PHI)
#define SP_MAX_THETA   (int32_t)ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_THETA)
#define SP_MAX_R       (int32_t)ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_R)
#elif defined STABILIZATION_ATTITUDE_TYPE_FLOAT
#define SP_MAX_PHI   STABILIZATION_ATTITUDE_SP_MAX_PHI
#define SP_MAX_THETA STABILIZATION_ATTITUDE_SP_MAX_THETA
#define SP_MAX_R     STABILIZATION_ATTITUDE_SP_MAX_R
#else
#error "STABILIZATION_ATTITUDE_TYPE not defined"
#endif

#define RC_UPDATE_FREQ 40

#ifdef STABILIZATION_ATTITUDE_DEADBAND_A
#define ROLL_DEADBAND_EXCEEDED()                                        \
  (radio_control.values[RADIO_ROLL] >  STABILIZATION_ATTITUDE_DEADBAND_A || \
   radio_control.values[RADIO_ROLL] < -STABILIZATION_ATTITUDE_DEADBAND_A)
#else
#define ROLL_DEADBAND_EXCEEDED() (TRUE)
#endif /* STABILIZATION_ATTITUDE_DEADBAND_A */

#ifdef STABILIZATION_ATTITUDE_DEADBAND_E
#define PITCH_DEADBAND_EXCEEDED()                                       \
  (radio_control.values[RADIO_PITCH] >  STABILIZATION_ATTITUDE_DEADBAND_E || \
   radio_control.values[RADIO_PITCH] < -STABILIZATION_ATTITUDE_DEADBAND_E)
#else
#define PITCH_DEADBAND_EXCEEDED() (TRUE)
#endif /* STABILIZATION_ATTITUDE_DEADBAND_E */

#define YAW_DEADBAND_EXCEEDED()                                         \
  (radio_control.values[RADIO_YAW] >  STABILIZATION_ATTITUDE_DEADBAND_R || \
   radio_control.values[RADIO_YAW] < -STABILIZATION_ATTITUDE_DEADBAND_R)


static inline void stabilization_attitude_read_rc_setpoint_eulers(struct Int32Eulers *sp, bool_t in_flight) {

  sp->phi = ((int32_t) radio_control.values[RADIO_ROLL]  * SP_MAX_PHI / MAX_PPRZ);
  sp->theta = ((int32_t) radio_control.values[RADIO_PITCH] * SP_MAX_THETA / MAX_PPRZ);

  if (in_flight) {
    if (YAW_DEADBAND_EXCEEDED()) {
      sp->psi += ((int32_t) radio_control.values[RADIO_YAW] * SP_MAX_R / MAX_PPRZ / RC_UPDATE_FREQ);
      INT32_ANGLE_NORMALIZE(sp->psi);
    }
    if (autopilot_mode == AP_MODE_FORWARD) {
      //Coordinated turn
      //feedforward estimate angular rotation omega = g*tan(phi)/v
      //Take v = 9.81/1.3 m/s
      int32_t omega;
      if(abs(sp->phi) < ANGLE_BFP_OF_REAL(60.0/180.0*M_PI))
	omega = ANGLE_BFP_OF_REAL(1.3*tan(ANGLE_FLOAT_OF_BFP(sp->phi)));
      else //max 60 degrees roll, then take constant omega
	omega = ANGLE_BFP_OF_REAL(1.3*1.72305* ((sp->phi > 0) - (sp->phi < 0)));

      sp->psi += omega/RC_UPDATE_FREQ;
    }

#ifdef STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT
//     This is a different way to obtain yaw. It will not switch when going beyond 90 degrees pitch.
//     However, when rolling more then 90 degrees in combination with pitch it switches switching. For a
//     transition vehicle this is better as 90 degrees pitch will occur, but more than 90 degrees roll probably not.
    int32_t yaw;
    if(abs(stateGetNedToBodyEulers_i()->phi) < INT32_ANGLE_PI_2) {
      int32_t sin_theta;
      PPRZ_ITRIG_SIN(sin_theta, stateGetNedToBodyEulers_i()->theta);
      yaw = stateGetNedToBodyEulers_i()->psi - INT_MULT_RSHIFT(sin_theta, stateGetNedToBodyEulers_i()->phi, INT32_TRIG_FRAC);
    }
    else if(ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->theta) > 0)
      yaw = stateGetNedToBodyEulers_i()->psi - stateGetNedToBodyEulers_i()->phi;
    else
      yaw = stateGetNedToBodyEulers_i()->psi + stateGetNedToBodyEulers_i()->phi;

    // Make sure the yaw setpoint does not differ too much from the real yaw to prevent a sudden switch at 180 deg
    int32_t delta_psi = sp->psi - yaw;
    int32_t delta_limit = ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT);
    INT32_ANGLE_NORMALIZE(delta_psi);
    if (delta_psi > delta_limit){
      sp->psi = yaw + delta_limit;
    }
    else if (delta_psi < -delta_limit){
      sp->psi = yaw - delta_limit;
    }
    INT32_ANGLE_NORMALIZE(sp->psi);
#endif
  }
  else { /* if not flying, use current yaw as setpoint */
    sp->psi = stateGetNedToBodyEulers_i()->psi;
  }
}


static inline void stabilization_attitude_read_rc_setpoint_eulers_f(struct FloatEulers *sp, bool_t in_flight) {
  sp->phi = (radio_control.values[RADIO_ROLL]  * SP_MAX_PHI / MAX_PPRZ);
  sp->theta = (radio_control.values[RADIO_PITCH] * SP_MAX_THETA / MAX_PPRZ);

  if (in_flight) {
    if (YAW_DEADBAND_EXCEEDED()) {
      sp->psi += (radio_control.values[RADIO_YAW] * SP_MAX_R / MAX_PPRZ / RC_UPDATE_FREQ);
      FLOAT_ANGLE_NORMALIZE(sp->psi);
    }
    if (autopilot_mode == AP_MODE_FORWARD) {
      //Coordinated turn
      //feedforward estimate angular rotation omega = g*tan(phi)/v
      //Take v = 9.81/1.3 m/s
      float omega;
      if(abs(sp->phi) < (60.0/180.0*M_PI))
	omega = 1.3*tan(sp->phi);
      else //max 60 degrees roll, then take constant omega
	omega = 1.3*1.72305* ((sp->phi > 0) - (sp->phi < 0));

      sp->psi += omega/RC_UPDATE_FREQ;
    }

#ifdef STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT
//     This is a different way to obtain yaw. It will not switch when going beyond 90 degrees pitch.
//     However, when rolling more then 90 degrees in combination with pitch it switches switching. For a
//     transition vehicle this is better as 90 degrees pitch will occur, but more than 90 degrees roll probably not.
    float yaw;
    if(abs(stateGetNedToBodyEulers_f()->phi) < (M_PI/2.0))
      yaw = stateGetNedToBodyEulers_f()->psi - sin(stateGetNedToBodyEulers_f()->theta) * stateGetNedToBodyEulers_f()->phi;
    else if(ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->theta) > 0)
      yaw = stateGetNedToBodyEulers_f()->psi - stateGetNedToBodyEulers_f()->phi;
    else
      yaw = stateGetNedToBodyEulers_f()->psi + stateGetNedToBodyEulers_f()->phi;

    // Make sure the yaw setpoint does not differ too much from the real yaw to prevent a sudden switch at 180 deg
    float delta_psi = sp->psi - yaw;
    FLOAT_ANGLE_NORMALIZE(delta_psi);
    if (delta_psi > STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT){
      sp->psi = yaw + STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT;
    }
    else if (delta_psi < -STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT){
      sp->psi = yaw - STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT;
    }
    FLOAT_ANGLE_NORMALIZE(sp->psi);
#endif
  }
  else { /* if not flying, use current yaw as setpoint */
    sp->psi = stateGetNedToBodyEulers_f()->psi;
  }
}


/** Read roll/pitch command from RC as quaternion.
 * Interprets the stick positions as axes.
 * @param[out] q quaternion representing the RC roll/pitch input
 */
static inline void stabilization_attitude_read_rc_roll_pitch_quat_f(struct FloatQuat* q) {
  q->qx = radio_control.values[RADIO_ROLL] * STABILIZATION_ATTITUDE_SP_MAX_PHI / MAX_PPRZ / 2;
  q->qy = radio_control.values[RADIO_PITCH] * STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ / 2;
  q->qz = 0.0;

  /* normalize */
  float norm = sqrtf(1.0 + SQUARE(q->qx)+ SQUARE(q->qy));
  q->qi = 1.0 / norm;
  q->qx /= norm;
  q->qy /= norm;
}

/** Read roll/pitch command from RC as quaternion.
 * Both angles are are interpreted relative to to the horizontal plane (earth bound).
 * @param[out] q quaternion representing the RC roll/pitch input
 */
static inline void stabilization_attitude_read_rc_roll_pitch_earth_quat_f(struct FloatQuat* q) {
  /* only non-zero entries for roll quaternion */
  float roll2 = radio_control.values[RADIO_ROLL] * STABILIZATION_ATTITUDE_SP_MAX_PHI / MAX_PPRZ / 2;
  float qx_roll = sinf(roll2);
  float qi_roll = cosf(roll2);

  //An offset is added if in forward mode
  /* only non-zero entries for pitch quaternion */
  float pitch2 = (ANGLE_FLOAT_OF_BFP(theta_offset) + radio_control.values[RADIO_PITCH] * STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ) / 2;
  float qy_pitch = sinf(pitch2);
  float qi_pitch = cosf(pitch2);

  /* only multiply non-zero entries of FLOAT_QUAT_COMP(*q, q_roll, q_pitch) */
  q->qi = qi_roll * qi_pitch;
  q->qx = qx_roll * qi_pitch;
  q->qy = qi_roll * qy_pitch;
  q->qz = qx_roll * qy_pitch;
}

static inline void stabilization_attitude_read_rc_setpoint_quat_f(struct FloatQuat* q_sp, bool_t in_flight) {

  // FIXME: remove me, do in quaternion directly
  // is currently still needed, since the yaw setpoint integration is done in eulers
#if defined STABILIZATION_ATTITUDE_TYPE_INT
  stabilization_attitude_read_rc_setpoint_eulers(&stab_att_sp_euler, in_flight);
#else
  stabilization_attitude_read_rc_setpoint_eulers_f(&stab_att_sp_euler, in_flight);
#endif

  struct FloatQuat q_rp_cmd;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_roll_pitch_earth_quat_f(&q_rp_cmd);
    const struct FloatVect3 zaxis = {0., 0., 1.};
  if (in_flight) {
    /* get current heading setpoint */
    struct FloatQuat q_yaw_sp;
    FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw_sp, zaxis, stab_att_sp_euler.psi);

#if defined STABILIZATION_ATTITUDE_TYPE_INT
    FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw_sp, zaxis, ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.psi));
#else
    FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw_sp, zaxis, stab_att_sp_euler.psi);
#endif

  FLOAT_QUAT_COMP(*q_sp, q_yaw_sp, q_rp_cmd);
  }
  else {
    struct FloatQuat q_yaw;
    FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw, zaxis, stateGetNedToBodyEulers_f()->psi);

    /* roll/pitch commands applied to to current heading */
    struct FloatQuat q_rp_sp;
    FLOAT_QUAT_COMP(q_rp_sp, q_yaw, q_rp_cmd);
    FLOAT_QUAT_NORMALIZE(q_rp_sp);

    QUAT_COPY(*q_sp, q_rp_sp);
  }

#else
  stabilization_attitude_read_rc_roll_pitch_quat_f(&q_rp_cmd);

  /* get current heading */
  const struct FloatVect3 zaxis = {0., 0., 1.};
  struct FloatQuat q_yaw;

  //Care Free mode
  if(autopilot_mode == AP_MODE_CARE_FREE) {
    //mem_psi has been set to current psi when entering care free mode.
    FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw, zaxis, mem_psi);
  }
  else {
    FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw, zaxis, stateGetNedToBodyEulers_f()->psi);
  }

  /* roll/pitch commands applied to to current heading */
  struct FloatQuat q_rp_sp;
  FLOAT_QUAT_COMP(q_rp_sp, q_yaw, q_rp_cmd);
  FLOAT_QUAT_NORMALIZE(q_rp_sp);

  if (in_flight)
  {
    /* get current heading setpoint */
    struct FloatQuat q_yaw_sp;
#if defined STABILIZATION_ATTITUDE_TYPE_INT
    FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw_sp, zaxis, ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.psi));
#else
    FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw_sp, zaxis, stab_att_sp_euler.psi);
#endif

    /* rotation between current yaw and yaw setpoint */
    struct FloatQuat q_yaw_diff;
    FLOAT_QUAT_COMP_INV(q_yaw_diff, q_yaw_sp, q_yaw);

    /* compute final setpoint with yaw */
    FLOAT_QUAT_COMP_NORM_SHORTEST(*q_sp, q_rp_sp, q_yaw_diff);
  } else {
    QUAT_COPY(*q_sp, q_rp_sp);
  }
#endif
}
#endif /* STABILIZATION_ATTITUDE_RC_SETPOINT_H */
