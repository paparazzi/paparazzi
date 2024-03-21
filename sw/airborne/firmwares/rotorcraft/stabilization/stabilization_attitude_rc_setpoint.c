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

#include "modules/radio_control/radio_control.h"
#include "state.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "mcu_periph/sys_time.h"

#ifndef STABILIZATION_ATTITUDE_DEADBAND_A
#define STABILIZATION_ATTITUDE_DEADBAND_A 0
#endif

#ifndef STABILIZATION_ATTITUDE_DEADBAND_E
#define STABILIZATION_ATTITUDE_DEADBAND_E 0
#endif

/**
 * Airspeed that will be used in the turning speed calculation (m/s).
 *
 * This variable is for calculation of the turn speed, and does not influence the airspeed.
 * With a higher speed, the vehicle will turn less in a turn with the same roll.
 * */
#ifndef COORDINATED_TURN_AIRSPEED
#define COORDINATED_TURN_AIRSPEED 12.0
#endif

#define YAW_DEADBAND_EXCEEDED(_rc)                               \
  (rc->values[RADIO_YAW] >  STABILIZATION_ATTITUDE_DEADBAND_R || \
   rc->values[RADIO_YAW] < -STABILIZATION_ATTITUDE_DEADBAND_R)

static int32_t get_rc_roll(struct RadioControl *rc)
{
  const int32_t max_rc_phi = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_PHI);
  int32_t roll = rc->values[RADIO_ROLL];
  DeadBand(roll, STABILIZATION_ATTITUDE_DEADBAND_A);
  return roll * max_rc_phi / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_A);
}

static int32_t get_rc_pitch(struct RadioControl *rc)
{
  const int32_t max_rc_theta = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_THETA);
  int32_t pitch = rc->values[RADIO_PITCH];
  DeadBand(pitch, STABILIZATION_ATTITUDE_DEADBAND_E);
  return pitch * max_rc_theta / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_E);
}

static int32_t get_rc_yaw(struct RadioControl *rc)
{
  const int32_t max_rc_r = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_R);
  int32_t yaw = rc->values[RADIO_YAW];
  DeadBand(yaw, STABILIZATION_ATTITUDE_DEADBAND_R);
  return yaw * max_rc_r / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_R);
}

static float get_rc_roll_f(struct RadioControl *rc)
{
  int32_t roll = rc->values[RADIO_ROLL];
  DeadBand(roll, STABILIZATION_ATTITUDE_DEADBAND_A);
  return roll * STABILIZATION_ATTITUDE_SP_MAX_PHI / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_A);
}

static float get_rc_pitch_f(struct RadioControl *rc)
{
  int32_t pitch = rc->values[RADIO_PITCH];
  DeadBand(pitch, STABILIZATION_ATTITUDE_DEADBAND_E);
  return pitch * STABILIZATION_ATTITUDE_SP_MAX_THETA / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_E);
}

static inline float get_rc_yaw_f(struct RadioControl *rc)
{
  int32_t yaw = rc->values[RADIO_YAW];
  DeadBand(yaw, STABILIZATION_ATTITUDE_DEADBAND_R);
  return yaw * STABILIZATION_ATTITUDE_SP_MAX_R / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_R);
}

void stabilization_attitude_rc_setpoint_init(struct AttitudeRCInput *rc_sp)
{
  float_quat_identity(&rc_sp->rc_quat);
  FLOAT_EULERS_ZERO(rc_sp->rc_eulers);
  rc_sp->care_free_heading = 0.f;
  rc_sp->transition_theta_offset = 0.f;
  rc_sp->last_ts = 0.f;
}

/** Read attitude setpoint from RC as quaternion
 * Interprets the stick positions as axes.
 * @param[out] rc_sp             pointer to rc input structure
 * @param[in]  in_flight         true if in flight
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  rc                pointer to radio control structure
 */
void stabilization_attitude_read_rc_setpoint(struct AttitudeRCInput *rc_sp, bool in_flight, bool in_carefree,
    bool coordinated_turn, struct RadioControl *rc)
{
  // FIXME: remove me, do in quaternion directly
  // is currently still needed, since the yaw setpoint integration is done in eulers
  stabilization_attitude_read_rc_setpoint_eulers_f(rc_sp, in_flight, in_carefree, coordinated_turn, rc);

  struct FloatQuat q_rp_cmd;
  stabilization_attitude_read_rc_roll_pitch_quat_f(&q_rp_cmd, rc);

  /* get current heading */
  const struct FloatVect3 zaxis = {0., 0., 1.};
  struct FloatQuat q_yaw;

  //Care Free mode
  if (in_carefree) {
    //care_free_heading has been set to current psi when entering care free mode.
    float_quat_of_axis_angle(&q_yaw, &zaxis, rc_sp->care_free_heading);
  } else {
    float_quat_of_axis_angle(&q_yaw, &zaxis, stateGetNedToBodyEulers_f()->psi);
  }

  /* roll/pitch commands applied to to current heading */
  struct FloatQuat q_rp_sp;
  float_quat_comp(&q_rp_sp, &q_yaw, &q_rp_cmd);
  float_quat_normalize(&q_rp_sp);

  if (in_flight) {
    /* get current heading setpoint */
    struct FloatQuat q_yaw_sp;
    float_quat_of_axis_angle(&q_yaw_sp, &zaxis, rc_sp->rc_eulers.psi);

    /* rotation between current yaw and yaw setpoint */
    struct FloatQuat q_yaw_diff;
    float_quat_comp_inv(&q_yaw_diff, &q_yaw_sp, &q_yaw);

    /* compute final setpoint with yaw */
    float_quat_comp_norm_shortest(&rc_sp->rc_quat, &q_rp_sp, &q_yaw_diff);
  } else {
    QUAT_COPY(rc_sp->rc_quat, q_rp_sp);
  }
}

/** Read attitude setpoint from RC as quaternion in earth bound frame
 * @param[out] rc_sp             pointer to rc input structure
 * @param[in]  in_flight         true if in flight
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  rc                pointer to radio control structure
 */
void stabilization_attitude_read_rc_setpoint_earth_bound(struct AttitudeRCInput *rc_sp, bool in_flight,
    bool in_carefree, bool coordinated_turn, struct RadioControl *rc)
{
  // FIXME: remove me, do in quaternion directly
  // is currently still needed, since the yaw setpoint integration is done in eulers
  stabilization_attitude_read_rc_setpoint_eulers_f(rc_sp, in_flight, in_carefree, coordinated_turn, rc);

  const struct FloatVect3 zaxis = {0., 0., 1.};

  struct FloatQuat q_rp_cmd;
  stabilization_attitude_read_rc_roll_pitch_earth_quat_f(&q_rp_cmd, rc_sp->transition_theta_offset, rc);

  if (in_flight) {
    /* get current heading setpoint */
    struct FloatQuat q_yaw_sp;
    float_quat_of_axis_angle(&q_yaw_sp, &zaxis, rc_sp->rc_eulers.psi);
    float_quat_comp(&rc_sp->rc_quat, &q_yaw_sp, &q_rp_cmd);
  } else {
    struct FloatQuat q_yaw;
    float_quat_of_axis_angle(&q_yaw, &zaxis, stateGetNedToBodyEulers_f()->psi);

    /* roll/pitch commands applied to to current heading */
    struct FloatQuat q_rp_sp;
    float_quat_comp(&q_rp_sp, &q_yaw, &q_rp_cmd);
    float_quat_normalize(&q_rp_sp);

    QUAT_COPY(rc_sp->rc_quat, q_rp_sp);
  }
}

/// reset to current state
void stabilization_attitude_reset_rc_setpoint(struct AttitudeRCInput *rc_sp)
{
  rc_sp->rc_eulers = *stateGetNedToBodyEulers_f();
  rc_sp->rc_quat = *stateGetNedToBodyQuat_f();
}

/// reset the heading for care-free mode to current heading
void stabilization_attitude_reset_care_free_heading(struct AttitudeRCInput *rc_sp)
{
  rc_sp->care_free_heading = stateGetNedToBodyEulers_f()->psi;
}

/*   This is a different way to obtain yaw. It will not switch when going beyond 90 degrees pitch.
     However, when rolling more then 90 degrees in combination with pitch it switches. For a
     transition vehicle this is better as 90 degrees pitch will occur, but more than 90 degrees roll probably not. */
int32_t stabilization_attitude_get_heading_i(void)
{
  struct Int32Eulers *att = stateGetNedToBodyEulers_i();
  int32_t heading;
  if (abs(att->phi) < INT32_ANGLE_PI_2) {
    int32_t sin_theta;
    PPRZ_ITRIG_SIN(sin_theta, att->theta);
    heading = att->psi - INT_MULT_RSHIFT(sin_theta, att->phi, INT32_TRIG_FRAC);
  } else if (ANGLE_FLOAT_OF_BFP(att->theta) > 0) {
    heading = att->psi - att->phi;
  } else {
    heading = att->psi + att->phi;
  }
  return heading;
}

float stabilization_attitude_get_heading_f(void)
{
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  float heading;
  if (fabsf(att->phi) < M_PI / 2) {
    heading = att->psi - sinf(att->theta) * att->phi;
  } else if (att->theta > 0) {
    heading = att->psi - att->phi;
  } else {
    heading = att->psi + att->phi;
  }
  return heading;
}


/** Read attitude setpoint from RC as euler angles
 * Only the euler format is updated and returned
 * @param[out] rc_sp             pointer to rc input structure
 * @param[in]  in_flight         true if in flight
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  rc                pointer to radio control structure
 * @return attitude setpoint in eulers (int)
 */
struct Int32Eulers stabilization_attitude_read_rc_setpoint_eulers(struct AttitudeRCInput *rc_sp, bool in_flight,
    bool in_carefree, bool coordinated_turn, struct RadioControl *rc)
{
  struct Int32Eulers sp_i;
  EULERS_BFP_OF_REAL(sp_i, rc_sp->rc_eulers);

  sp_i.phi = get_rc_roll(rc);
  sp_i.theta = get_rc_pitch(rc);

  if (in_flight) {
    /* calculate dt for yaw integration */
    float dt = get_sys_time_float() - rc_sp->last_ts;
    /* make sure nothing drastically weird happens, bound dt to 0.5sec */
    Bound(dt, 0, 0.5);

    /* do not advance yaw setpoint if within a small deadband around stick center or if throttle is zero */
    if (YAW_DEADBAND_EXCEEDED(rc) && !THROTTLE_STICK_DOWN_FROM_RC(rc)) {
      sp_i.psi += get_rc_yaw(rc) * dt;
      INT32_ANGLE_NORMALIZE(sp_i.psi);
    }
    if (coordinated_turn) {
      //Coordinated turn
      //feedforward estimate angular rotation omega = g*tan(phi)/v
      int32_t omega;
      const int32_t max_phi = ANGLE_BFP_OF_REAL(RadOfDeg(60.0));
      if (abs(sp_i.phi) < max_phi) {
        omega = ANGLE_BFP_OF_REAL(9.81 / COORDINATED_TURN_AIRSPEED * tanf(ANGLE_FLOAT_OF_BFP(sp_i.phi)));
      } else { //max 60 degrees roll
        omega = ANGLE_BFP_OF_REAL(9.81 / COORDINATED_TURN_AIRSPEED * 1.72305 * ((sp_i.phi > 0) - (sp_i.phi < 0)));
      }

      sp_i.psi += omega * dt;
    }
#ifdef STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT
    // Make sure the yaw setpoint does not differ too much from the real yaw
    // to prevent a sudden switch at 180 deg
    const int32_t delta_limit = ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT);

    int32_t heading = stabilization_attitude_get_heading_i();

    int32_t delta_psi = sp_i.psi - heading;
    INT32_ANGLE_NORMALIZE(delta_psi);
    if (delta_psi > delta_limit) {
      sp_i.psi = heading + delta_limit;
    } else if (delta_psi < -delta_limit) {
      sp_i.psi = heading - delta_limit;
    }
    INT32_ANGLE_NORMALIZE(sp_i.psi);
#endif
    //Care Free mode
    if (in_carefree) {
      //care_free_heading has been set to current psi when entering care free mode.
      int32_t cos_psi;
      int32_t sin_psi;
      int32_t temp_theta;
      int32_t care_free_delta_psi_i;

      care_free_delta_psi_i = sp_i.psi - ANGLE_BFP_OF_REAL(rc_sp->care_free_heading);

      INT32_ANGLE_NORMALIZE(care_free_delta_psi_i);

      PPRZ_ITRIG_SIN(sin_psi, care_free_delta_psi_i);
      PPRZ_ITRIG_COS(cos_psi, care_free_delta_psi_i);

      temp_theta = INT_MULT_RSHIFT(cos_psi, sp_i.theta, INT32_ANGLE_FRAC) - INT_MULT_RSHIFT(sin_psi, sp_i.phi,
                   INT32_ANGLE_FRAC);
      sp_i.phi = INT_MULT_RSHIFT(cos_psi, sp_i.phi, INT32_ANGLE_FRAC) - INT_MULT_RSHIFT(sin_psi, sp_i.theta, INT32_ANGLE_FRAC);

      sp_i.theta = temp_theta;
    }
  } else { /* if not flying, use current yaw as setpoint */
    sp_i.psi = stateGetNedToBodyEulers_i()->psi;
  }

  /* update timestamp for dt calculation */
  rc_sp->last_ts = get_sys_time_float();
  EULERS_FLOAT_OF_BFP(rc_sp->rc_eulers, sp_i);
  return sp_i;
}


/** Read attitude setpoint from RC as float euler angles
 * Only the euler format is updated and returned
 * @param[out] rc_sp             pointer to rc input structure
 * @param[in]  in_flight         true if in flight
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  rc                pointer to radio control structure
 * @return attitude setpoint in eulers (float)
 */
struct FloatEulers stabilization_attitude_read_rc_setpoint_eulers_f(struct AttitudeRCInput *rc_sp, bool in_flight,
    bool in_carefree, bool coordinated_turn, struct RadioControl *rc)
{
  rc_sp->rc_eulers.phi = get_rc_roll_f(rc);
  rc_sp->rc_eulers.theta = get_rc_pitch_f(rc);

  if (in_flight) {
    /* calculate dt for yaw integration */
    float dt = get_sys_time_float() - rc_sp->last_ts;
    /* make sure nothing drastically weird happens, bound dt to 0.5sec */
    Bound(dt, 0, 0.5);

    /* do not advance yaw setpoint if within a small deadband around stick center or if throttle is zero */
    if (YAW_DEADBAND_EXCEEDED(rc) && !THROTTLE_STICK_DOWN_FROM_RC(rc)) {
      rc_sp->rc_eulers.psi += get_rc_yaw_f(rc) * dt;
      FLOAT_ANGLE_NORMALIZE(rc_sp->rc_eulers.psi);
    }
    if (coordinated_turn) {
      //Coordinated turn
      //feedforward estimate angular rotation omega = g*tan(phi)/v
      float omega;
      const float max_phi = RadOfDeg(60.0);
      if (fabsf(rc_sp->rc_eulers.phi) < max_phi) {
        omega = 9.81 / COORDINATED_TURN_AIRSPEED * tanf(rc_sp->rc_eulers.phi);
      } else { //max 60 degrees roll
        omega = 9.81 / COORDINATED_TURN_AIRSPEED * 1.72305 * ((rc_sp->rc_eulers.phi > 0) - (rc_sp->rc_eulers.phi < 0));
      }

      rc_sp->rc_eulers.psi += omega * dt;
    }
#ifdef STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT
    // Make sure the yaw setpoint does not differ too much from the real yaw
    // to prevent a sudden switch at 180 deg
    float heading = stabilization_attitude_get_heading_f();

    float delta_psi = rc_sp->rc_eulers.psi - heading;
    FLOAT_ANGLE_NORMALIZE(delta_psi);
    if (delta_psi > STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT) {
      rc_sp->rc_eulers.psi = heading + STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT;
    } else if (delta_psi < -STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT) {
      rc_sp->rc_eulers.psi = heading - STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT;
    }
    FLOAT_ANGLE_NORMALIZE(rc_sp->rc_eulers.psi);
#endif
    //Care Free mode
    if (in_carefree) {
      //care_free_heading has been set to current psi when entering care free mode.
      float cos_psi;
      float sin_psi;
      float temp_theta;

      float care_free_delta_psi_f = rc_sp->rc_eulers.psi - rc_sp->care_free_heading;

      FLOAT_ANGLE_NORMALIZE(care_free_delta_psi_f);

      sin_psi = sinf(care_free_delta_psi_f);
      cos_psi = cosf(care_free_delta_psi_f);

      temp_theta = cos_psi * rc_sp->rc_eulers.theta - sin_psi * rc_sp->rc_eulers.phi;
      rc_sp->rc_eulers.phi = cos_psi * rc_sp->rc_eulers.phi - sin_psi * rc_sp->rc_eulers.theta;

      rc_sp->rc_eulers.theta = temp_theta;
    }
  } else { /* if not flying, use current yaw as setpoint */
    rc_sp->rc_eulers.psi = stateGetNedToBodyEulers_f()->psi;
  }

  /* update timestamp for dt calculation */
  rc_sp->last_ts = get_sys_time_float();
  return rc_sp->rc_eulers;
}


/** Read roll/pitch command from RC as quaternion.
 * Interprets the stick positions as axes.
 * @param[out] q quaternion representing the RC roll/pitch input
 * @param[in] rc pointer to radio control structure
 */
void stabilization_attitude_read_rc_roll_pitch_quat_f(struct FloatQuat *q, struct RadioControl *rc)
{
  /* orientation vector describing simultaneous rotation of roll/pitch */
  struct FloatVect3 ov;
  ov.x = get_rc_roll_f(rc);
  ov.y = get_rc_pitch_f(rc);
  ov.z = 0.0;

  /* quaternion from that orientation vector */
  float_quat_of_orientation_vect(q, &ov);
}

/** Read roll/pitch command from RC as quaternion.
 * Both angles are are interpreted relative to to the horizontal plane (earth bound).
 * @param[out] q quaternion representing the RC roll/pitch input
 * @param[in] theta_offset pitch offset for forward flight
 * @param[in] rc pointer to radio control structure
 */
void stabilization_attitude_read_rc_roll_pitch_earth_quat_f(struct FloatQuat *q, float theta_offset, struct RadioControl *rc)
{
  /* only non-zero entries for roll quaternion */
  float roll2 = get_rc_roll_f(rc) / 2.0f;
  float qx_roll = sinf(roll2);
  float qi_roll = cosf(roll2);

  //An offset is added if in forward mode
  /* only non-zero entries for pitch quaternion */
  float pitch2 = (theta_offset + get_rc_pitch_f(rc)) / 2.0f;
  float qy_pitch = sinf(pitch2);
  float qi_pitch = cosf(pitch2);

  /* only multiply non-zero entries of float_quat_comp(q, &q_roll, &q_pitch) */
  q->qi = qi_roll * qi_pitch;
  q->qx = qx_roll * qi_pitch;
  q->qy = qi_roll * qy_pitch;
  q->qz = qx_roll * qy_pitch;
}

