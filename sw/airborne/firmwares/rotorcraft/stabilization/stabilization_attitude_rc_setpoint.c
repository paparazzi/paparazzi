/*
 * Copyright (C) 2012-2013 Felix Ruess <felix.ruess@gmail.com>
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

/** @file stabilization_attitude_rc_setpoint.c
 *  Read an attitude setpoint from the RC.
 */

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"

#include "subsystems/radio_control.h"
#include "state.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref.h"

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

#ifndef RC_UPDATE_FREQ
#define RC_UPDATE_FREQ 40
#endif

float care_free_heading = 0;

/// reset the heading for care-free mode to current heading
void stabilization_attitude_reset_care_free_heading(void) {
  care_free_heading = stateGetNedToBodyEulers_f()->psi;
}

/** Read attitude setpoint from RC as euler angles.
 * @param[in]  in_flight  true if in flight
 * @param[out] sp         attitude setpoint as euler angles
 */
void stabilization_attitude_read_rc_setpoint_eulers(struct Int32Eulers *sp, bool_t in_flight) {

  sp->phi = ((int32_t) radio_control.values[RADIO_ROLL]  * SP_MAX_PHI / MAX_PPRZ);
  sp->theta = ((int32_t) radio_control.values[RADIO_PITCH] * SP_MAX_THETA / MAX_PPRZ);

  if (in_flight) {
    if (YAW_DEADBAND_EXCEEDED()) {
      sp->psi += ((int32_t) radio_control.values[RADIO_YAW] * SP_MAX_R / MAX_PPRZ / RC_UPDATE_FREQ);
      INT32_ANGLE_NORMALIZE(sp->psi);
    }
#ifdef STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT
    // Make sure the yaw setpoint does not differ too much from the real yaw
    // to prevent a sudden switch at 180 deg
    int32_t delta_psi = sp->psi - stateGetNedToBodyEulers_i()->psi;
    int32_t delta_limit = ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT);
    INT32_ANGLE_NORMALIZE(delta_psi);
    if (delta_psi > delta_limit){
      sp->psi = stateGetNedToBodyEulers_i()->psi + delta_limit;
    }
    else if (delta_psi < -delta_limit){
      sp->psi = stateGetNedToBodyEulers_i()->psi - delta_limit;
    }
    INT32_ANGLE_NORMALIZE(sp->psi);
#endif
    //Care Free mode
    if (guidance_h_mode == GUIDANCE_H_MODE_CARE_FREE) {
      //care_free_heading has been set to current psi when entering care free mode.
      int32_t cos_psi;
      int32_t sin_psi;
      int32_t care_free_heading_i;
      int32_t temp_theta;

      care_free_heading_i = ANGLE_BFP_OF_REAL(care_free_heading);

      PPRZ_ITRIG_SIN(sin_psi, sp->psi - care_free_heading_i);
      PPRZ_ITRIG_COS(cos_psi, sp->psi - care_free_heading_i);

      temp_theta = INT_MULT_RSHIFT(cos_psi, sp->theta, INT32_ANGLE_FRAC) - INT_MULT_RSHIFT(sin_psi, sp->phi, INT32_ANGLE_FRAC);
      sp->phi = INT_MULT_RSHIFT(cos_psi, sp->phi, INT32_ANGLE_FRAC) - INT_MULT_RSHIFT(sin_psi, sp->theta, INT32_ANGLE_FRAC);

      sp->theta = temp_theta;
    }
  }
  else { /* if not flying, use current yaw as setpoint */
    sp->psi = stateGetNedToBodyEulers_i()->psi;
  }
}


void stabilization_attitude_read_rc_setpoint_eulers_f(struct FloatEulers *sp, bool_t in_flight) {
  sp->phi = (radio_control.values[RADIO_ROLL]  * SP_MAX_PHI / MAX_PPRZ);
  sp->theta = (radio_control.values[RADIO_PITCH] * SP_MAX_THETA / MAX_PPRZ);

  if (in_flight) {
    if (YAW_DEADBAND_EXCEEDED()) {
      sp->psi += (radio_control.values[RADIO_YAW] * SP_MAX_R / MAX_PPRZ / RC_UPDATE_FREQ);
      FLOAT_ANGLE_NORMALIZE(sp->psi);
    }
#ifdef STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT
    // Make sure the yaw setpoint does not differ too much from the real yaw
    // to prevent a sudden switch at 180 deg
    float delta_psi = sp->psi - stateGetNedToBodyEulers_f()->psi;
    FLOAT_ANGLE_NORMALIZE(delta_psi);
    if (delta_psi > STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT){
      sp->psi = stateGetNedToBodyEulers_f()->psi + STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT;
    }
    else if (delta_psi < -STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT){
      sp->psi = stateGetNedToBodyEulers_f()->psi - STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT;
    }
    FLOAT_ANGLE_NORMALIZE(sp->psi);
#endif
  }
  //Care Free mode
  if (guidance_h_mode == GUIDANCE_H_MODE_CARE_FREE) {
    //care_free_heading has been set to current psi when entering care free mode.
    float cos_psi;
    float sin_psi;
    float temp_theta;

    sin_psi = sin(sp->psi - care_free_heading);
    cos_psi = cos(sp->psi - care_free_heading);

    temp_theta = cos_psi*sp->theta - sin_psi*sp->phi;
    sp->phi = cos_psi*sp->phi - sin_psi*sp->theta;

    sp->theta = temp_theta;
  }
  else { /* if not flying, use current yaw as setpoint */
    sp->psi = stateGetNedToBodyEulers_f()->psi;
  }
}


/** Read roll/pitch command from RC as quaternion.
 * Interprets the stick positions as axes.
 * @param[out] q quaternion representing the RC roll/pitch input
 */
void stabilization_attitude_read_rc_roll_pitch_quat_f(struct FloatQuat* q) {
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
void stabilization_attitude_read_rc_roll_pitch_earth_quat_f(struct FloatQuat* q) {
  /* only non-zero entries for roll quaternion */
  float roll2 = radio_control.values[RADIO_ROLL] * STABILIZATION_ATTITUDE_SP_MAX_PHI / MAX_PPRZ / 2;
  float qx_roll = sinf(roll2);
  float qi_roll = cosf(roll2);

  /* only non-zero entries for pitch quaternion */
  float pitch2 = radio_control.values[RADIO_PITCH] * STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ / 2;
  float qy_pitch = sinf(pitch2);
  float qi_pitch = cosf(pitch2);

  /* only multiply non-zero entries of FLOAT_QUAT_COMP(*q, q_roll, q_pitch) */
  q->qi = qi_roll * qi_pitch;
  q->qx = qx_roll * qi_pitch;
  q->qy = qi_roll * qy_pitch;
  q->qz = qx_roll * qy_pitch;
}

void stabilization_attitude_read_rc_setpoint_quat_f(struct FloatQuat* q_sp, bool_t in_flight) {

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
#else
  stabilization_attitude_read_rc_roll_pitch_quat_f(&q_rp_cmd);
#endif

  /* get current heading */
  const struct FloatVect3 zaxis = {0., 0., 1.};
  struct FloatQuat q_yaw;

  //Care Free mode
  if (guidance_h_mode == GUIDANCE_H_MODE_CARE_FREE) {
    //care_free_heading has been set to current psi when entering care free mode.
    FLOAT_QUAT_OF_AXIS_ANGLE(q_yaw, zaxis, care_free_heading);
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
}
