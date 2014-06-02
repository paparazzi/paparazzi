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
#include "generated/airframe.h"

#include "subsystems/radio_control.h"
#include "state.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"

#ifndef RC_UPDATE_FREQ
#define RC_UPDATE_FREQ 40
#endif

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

float care_free_heading = 0;

/// reset the heading for care-free mode to current heading
void stabilization_attitude_reset_care_free_heading(void) {
  care_free_heading = stateGetNedToBodyEulers_f()->psi;
}

/*   This is a different way to obtain yaw. It will not switch when going beyond 90 degrees pitch.
     However, when rolling more then 90 degrees in combination with pitch it switches. For a
     transition vehicle this is better as 90 degrees pitch will occur, but more than 90 degrees roll probably not. */
int32_t stabilization_attitude_get_heading_i(void) {
  struct Int32Eulers* att = stateGetNedToBodyEulers_i();

  int32_t heading;

  if(abs(att->phi) < INT32_ANGLE_PI_2) {
    int32_t sin_theta;
    PPRZ_ITRIG_SIN(sin_theta, att->theta);
    heading = att->psi - INT_MULT_RSHIFT(sin_theta, att->phi, INT32_TRIG_FRAC);
  }
  else if(ANGLE_FLOAT_OF_BFP(att->theta) > 0)
    heading = att->psi - att->phi;
  else
    heading = att->psi + att->phi;

  return heading;
}

float stabilization_attitude_get_heading_f(void) {
  struct FloatEulers* att = stateGetNedToBodyEulers_f();

  float heading;

  if(abs(att->phi) < M_PI/2) {
    heading = att->psi - sinf(att->theta)*att->phi;
  }
  else if(att->theta > 0)
    heading = att->psi - att->phi;
  else
    heading = att->psi + att->phi;

  return heading;
}


/** Read attitude setpoint from RC as euler angles
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  in_flight         true if in flight
 * @param[out] sp                attitude setpoint as euler angles
 */
void stabilization_attitude_read_rc_setpoint_eulers(struct Int32Eulers *sp, bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn) {
  const int32_t max_rc_phi = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_PHI);
  const int32_t max_rc_theta = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_THETA);
  const int32_t max_rc_r = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_R);

  sp->phi = (int32_t) ((radio_control.values[RADIO_ROLL] * max_rc_phi) /  MAX_PPRZ);
  sp->theta = (int32_t) ((radio_control.values[RADIO_PITCH] * max_rc_theta) /  MAX_PPRZ);

  if (in_flight) {
    /* do not advance yaw setpoint if within a small deadband around stick center or if throttle is zero */
    if (YAW_DEADBAND_EXCEEDED() && !THROTTLE_STICK_DOWN()) {
      sp->psi += (int32_t) ((radio_control.values[RADIO_YAW] * max_rc_r) /  MAX_PPRZ / RC_UPDATE_FREQ);
      INT32_ANGLE_NORMALIZE(sp->psi);
    }
    if (coordinated_turn) {
      //Coordinated turn
      //feedforward estimate angular rotation omega = g*tan(phi)/v
      //Take v = 9.81/1.3 m/s
      int32_t omega;
      const int32_t max_phi = ANGLE_BFP_OF_REAL(RadOfDeg(85.0));
      if(abs(sp->phi) < max_phi)
        omega = ANGLE_BFP_OF_REAL(1.3*tanf(ANGLE_FLOAT_OF_BFP(sp->phi)));
      else //max 60 degrees roll, then take constant omega
        omega = ANGLE_BFP_OF_REAL(1.3*1.72305* ((sp->phi > 0) - (sp->phi < 0)));

      sp->psi += omega/RC_UPDATE_FREQ;
    }
#ifdef STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT
    // Make sure the yaw setpoint does not differ too much from the real yaw
    // to prevent a sudden switch at 180 deg
    const int32_t delta_limit = ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT);

    int32_t heading = stabilization_attitude_get_heading_i();

    int32_t delta_psi = sp->psi - heading;
    INT32_ANGLE_NORMALIZE(delta_psi);
    if (delta_psi > delta_limit){
      sp->psi = heading + delta_limit;
    }
    else if (delta_psi < -delta_limit){
      sp->psi = heading - delta_limit;
    }
    INT32_ANGLE_NORMALIZE(sp->psi);
#endif
    //Care Free mode
    if (in_carefree) {
      //care_free_heading has been set to current psi when entering care free mode.
      int32_t cos_psi;
      int32_t sin_psi;
      int32_t temp_theta;
      int32_t care_free_delta_psi_i;

      care_free_delta_psi_i = sp->psi - ANGLE_BFP_OF_REAL(care_free_heading);

      INT32_ANGLE_NORMALIZE(care_free_delta_psi_i);

      PPRZ_ITRIG_SIN(sin_psi, care_free_delta_psi_i);
      PPRZ_ITRIG_COS(cos_psi, care_free_delta_psi_i);

      temp_theta = INT_MULT_RSHIFT(cos_psi, sp->theta, INT32_ANGLE_FRAC) - INT_MULT_RSHIFT(sin_psi, sp->phi, INT32_ANGLE_FRAC);
      sp->phi = INT_MULT_RSHIFT(cos_psi, sp->phi, INT32_ANGLE_FRAC) - INT_MULT_RSHIFT(sin_psi, sp->theta, INT32_ANGLE_FRAC);

      sp->theta = temp_theta;
    }
  }
  else { /* if not flying, use current yaw as setpoint */
    sp->psi = stateGetNedToBodyEulers_i()->psi;
  }
}


void stabilization_attitude_read_rc_setpoint_eulers_f(struct FloatEulers *sp, bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn) {
  sp->phi = (radio_control.values[RADIO_ROLL]  * STABILIZATION_ATTITUDE_SP_MAX_PHI / MAX_PPRZ);
  sp->theta = (radio_control.values[RADIO_PITCH] * STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ);

  if (in_flight) {
    /* do not advance yaw setpoint if within a small deadband around stick center or if throttle is zero */
    if (YAW_DEADBAND_EXCEEDED() && !THROTTLE_STICK_DOWN()) {
      sp->psi += (radio_control.values[RADIO_YAW] * STABILIZATION_ATTITUDE_SP_MAX_R / MAX_PPRZ / RC_UPDATE_FREQ);
      FLOAT_ANGLE_NORMALIZE(sp->psi);
    }
    if (coordinated_turn) {
      //Coordinated turn
      //feedforward estimate angular rotation omega = g*tan(phi)/v
      //Take v = 9.81/1.3 m/s
      float omega;
      const float max_phi = RadOfDeg(85.0);
      if(abs(sp->phi) < max_phi)
        omega = 1.3*tanf(sp->phi);
      else //max 60 degrees roll, then take constant omega
        omega = 1.3*1.72305* ((sp->phi > 0) - (sp->phi < 0));

      sp->psi += omega/RC_UPDATE_FREQ;
    }
#ifdef STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT
    // Make sure the yaw setpoint does not differ too much from the real yaw
    // to prevent a sudden switch at 180 deg
    float heading = stabilization_attitude_get_heading_f();

    float delta_psi = sp->psi - heading;
    FLOAT_ANGLE_NORMALIZE(delta_psi);
    if (delta_psi > STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT){
      sp->psi = heading + STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT;
    }
    else if (delta_psi < -STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT){
      sp->psi = heading - STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT;
    }
    FLOAT_ANGLE_NORMALIZE(sp->psi);
#endif
    //Care Free mode
    if (in_carefree) {
      //care_free_heading has been set to current psi when entering care free mode.
      float cos_psi;
      float sin_psi;
      float temp_theta;

      float care_free_delta_psi_f = sp->psi - care_free_heading;

      FLOAT_ANGLE_NORMALIZE(care_free_delta_psi_f);

      sin_psi = sinf(care_free_delta_psi_f);
      cos_psi = cosf(care_free_delta_psi_f);

      temp_theta = cos_psi*sp->theta - sin_psi*sp->phi;
      sp->phi = cos_psi*sp->phi - sin_psi*sp->theta;

      sp->theta = temp_theta;
    }
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
  /* orientation vector describing simultaneous rotation of roll/pitch */
  struct FloatVect3 ov;
  ov.x = radio_control.values[RADIO_ROLL] * STABILIZATION_ATTITUDE_SP_MAX_PHI / MAX_PPRZ;
  ov.y = radio_control.values[RADIO_PITCH] * STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ;
  ov.z = 0.0;

  /* quaternion from that orientation vector */
  FLOAT_QUAT_OF_ORIENTATION_VECT(*q, ov);
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

  //An offset is added if in forward mode
  /* only non-zero entries for pitch quaternion */
  float pitch2 = (ANGLE_FLOAT_OF_BFP(transition_theta_offset) + radio_control.values[RADIO_PITCH] * STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ) / 2;
  float qy_pitch = sinf(pitch2);
  float qi_pitch = cosf(pitch2);

  /* only multiply non-zero entries of FLOAT_QUAT_COMP(*q, q_roll, q_pitch) */
  q->qi = qi_roll * qi_pitch;
  q->qx = qx_roll * qi_pitch;
  q->qy = qi_roll * qy_pitch;
  q->qz = qx_roll * qy_pitch;
}

/** Read attitude setpoint from RC as quaternion
 * Interprets the stick positions as axes.
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  in_flight         true if in flight
 * @param[out] q_sp              attitude setpoint as quaternion
 */
void stabilization_attitude_read_rc_setpoint_quat_f(struct FloatQuat* q_sp, bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn) {

  // FIXME: remove me, do in quaternion directly
  // is currently still needed, since the yaw setpoint integration is done in eulers
#if defined STABILIZATION_ATTITUDE_TYPE_INT
  stabilization_attitude_read_rc_setpoint_eulers(&stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_eulers_f(&stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
#endif

  struct FloatQuat q_rp_cmd;
  stabilization_attitude_read_rc_roll_pitch_quat_f(&q_rp_cmd);

  /* get current heading */
  const struct FloatVect3 zaxis = {0., 0., 1.};
  struct FloatQuat q_yaw;

  //Care Free mode
  if (in_carefree) {
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

//Function that reads the rc setpoint in an earth bound frame
void stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(struct FloatQuat* q_sp, bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn) {
  // FIXME: remove me, do in quaternion directly
  // is currently still needed, since the yaw setpoint integration is done in eulers
  #if defined STABILIZATION_ATTITUDE_TYPE_INT
  stabilization_attitude_read_rc_setpoint_eulers(&stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
  #else
  stabilization_attitude_read_rc_setpoint_eulers_f(&stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
  #endif

  const struct FloatVect3 zaxis = {0., 0., 1.};

  struct FloatQuat q_rp_cmd;
  stabilization_attitude_read_rc_roll_pitch_earth_quat_f(&q_rp_cmd);

  if (in_flight) {
    /* get current heading setpoint */
    struct FloatQuat q_yaw_sp;

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
}
