/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

/** @file firmwares/rotorcraft/guidance/guidance_h.c
 *  Horizontal guidance for rotorcrafts.
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/navigation.h"

#include "state.h"

#include "generated/airframe.h"

uint8_t guidance_h_mode;

struct Int32Vect2 guidance_h_pos_sp;
struct Int32Vect2 guidance_h_pos_ref;
struct Int32Vect2 guidance_h_speed_ref;
struct Int32Vect2 guidance_h_accel_ref;

struct Int32Vect2 guidance_h_pos_err;
struct Int32Vect2 guidance_h_speed_err;
struct Int32Vect2 guidance_h_pos_err_sum;
struct Int32Vect2 guidance_h_nav_err;

struct Int32Eulers guidance_h_rc_sp;
struct Int32Vect2  guidance_h_command_earth;
struct Int32Vect2  guidance_h_stick_earth_sp;
struct Int32Eulers guidance_h_command_body;

int32_t guidance_h_pgain;
int32_t guidance_h_dgain;
int32_t guidance_h_igain;
int32_t guidance_h_again;

/* warn if some gains are still negative */
#if (GUIDANCE_H_PGAIN < 0) ||                   \
  (GUIDANCE_H_DGAIN < 0)   ||                   \
  (GUIDANCE_H_IGAIN < 0)
#warning "ALL control gains are now positive!!!"
#endif


#ifndef GUIDANCE_H_AGAIN
#define GUIDANCE_H_AGAIN 0
#else
#if (GUIDANCE_H_AGAIN < 0)
#warning "ALL control gains are now positive!!!"
#endif
#endif

#ifndef GUIDANCE_H_MAX_BANK
#define GUIDANCE_H_MAX_BANK RadOfDeg(20)
#endif

static inline void guidance_h_update_reference(bool_t use_ref);
static inline void guidance_h_traj_run(bool_t in_flight);
static inline void guidance_h_hover_enter(void);
static inline void guidance_h_nav_enter(void);

#define GuidanceHSetRef(_pos, _speed, _accel) { \
    gh_set_ref(_pos, _speed, _accel);        \
    VECT2_COPY(guidance_h_pos_ref,   _pos);     \
    VECT2_COPY(guidance_h_speed_ref, _speed);   \
    VECT2_COPY(guidance_h_accel_ref, _accel);   \
  }


void guidance_h_init(void) {

  guidance_h_mode = GUIDANCE_H_MODE_KILL;
  INT_VECT2_ZERO(guidance_h_pos_sp);
  INT_VECT2_ZERO(guidance_h_pos_err_sum);
  INT_EULERS_ZERO(guidance_h_rc_sp);
  INT_EULERS_ZERO(guidance_h_command_body);
  guidance_h_pgain = GUIDANCE_H_PGAIN;
  guidance_h_igain = GUIDANCE_H_IGAIN;
  guidance_h_dgain = GUIDANCE_H_DGAIN;
  guidance_h_again = GUIDANCE_H_AGAIN;

}


void guidance_h_mode_changed(uint8_t new_mode) {
  if (new_mode == guidance_h_mode)
    return;

  switch (new_mode) {
    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_enter();
      break;

    case GUIDANCE_H_MODE_RATE:
      stabilization_rate_enter();
      break;

    case GUIDANCE_H_MODE_CARE_FREE:
      stabilization_attitude_reset_care_free_heading();
    case GUIDANCE_H_MODE_ATTITUDE:
      stabilization_attitude_enter();
      break;

    case GUIDANCE_H_MODE_HOVER:
      guidance_h_hover_enter();
      break;

    case GUIDANCE_H_MODE_NAV:
      guidance_h_nav_enter();
      break;
    default:
      break;
  }

  guidance_h_mode = new_mode;

}


void guidance_h_read_rc(bool_t  in_flight) {

  switch ( guidance_h_mode ) {

    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_read_rc();
      break;

    case GUIDANCE_H_MODE_RATE:
      stabilization_rate_read_rc();
      break;

    case GUIDANCE_H_MODE_CARE_FREE:
    case GUIDANCE_H_MODE_ATTITUDE:
      stabilization_attitude_read_rc(in_flight);
      break;

    case GUIDANCE_H_MODE_HOVER:
      stabilization_attitude_read_rc_setpoint_eulers(&guidance_h_rc_sp, in_flight);
      break;

    case GUIDANCE_H_MODE_NAV:
      if (radio_control.status == RC_OK) {
        stabilization_attitude_read_rc_setpoint_eulers(&guidance_h_rc_sp, in_flight);
      }
      else {
        INT_EULERS_ZERO(guidance_h_rc_sp);
      }
      break;
    default:
      break;
  }

}


void guidance_h_run(bool_t  in_flight) {
  switch ( guidance_h_mode ) {

    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_run(in_flight);
      break;

    case GUIDANCE_H_MODE_RATE:
      stabilization_rate_run(in_flight);
      break;

    case GUIDANCE_H_MODE_CARE_FREE:
    case GUIDANCE_H_MODE_ATTITUDE:
      stabilization_attitude_run(in_flight);
      break;

    case GUIDANCE_H_MODE_HOVER:
      guidance_h_update_reference(FALSE);

      /* set psi command */
      guidance_h_command_body.psi = guidance_h_rc_sp.psi;
      /* compute roll and pitch commands and set final attitude setpoint */
      guidance_h_traj_run(in_flight);

      stabilization_attitude_run(in_flight);
      break;

    case GUIDANCE_H_MODE_NAV:
      if (!in_flight) guidance_h_nav_enter();

      if (horizontal_mode == HORIZONTAL_MODE_ATTITUDE) {
        stab_att_sp_euler.phi = nav_roll;
        stab_att_sp_euler.theta = nav_pitch;
        /* FIXME: heading can't be set via attitude block yet, use current heading for now */
        stab_att_sp_euler.psi = stateGetNedToBodyEulers_i()->psi;
#ifdef STABILIZATION_ATTITUDE_TYPE_QUAT
        INT32_QUAT_OF_EULERS(stab_att_sp_quat, stab_att_sp_euler);
        INT32_QUAT_WRAP_SHORTEST(stab_att_sp_quat);
#endif
      }
      else {
        INT32_VECT2_NED_OF_ENU(guidance_h_pos_sp, navigation_carrot);

#if GUIDANCE_H_USE_REF
        guidance_h_update_reference(TRUE);
#else
        guidance_h_update_reference(FALSE);
#endif
        /* set psi command */
        guidance_h_command_body.psi = nav_heading;
        /* compute roll and pitch commands and set final attitude setpoint */
        guidance_h_traj_run(in_flight);
      }
      stabilization_attitude_run(in_flight);
      break;

    default:
      break;
  }

}

static inline void guidance_h_update_reference(bool_t use_ref) {
  /* convert our reference to generic representation */
  if (use_ref) {
    gh_update_ref_from_pos_sp(guidance_h_pos_sp);
    INT32_VECT2_RSHIFT(guidance_h_pos_ref,   gh_pos_ref,   (GH_POS_REF_FRAC - INT32_POS_FRAC));
    INT32_VECT2_LSHIFT(guidance_h_speed_ref, gh_speed_ref, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
    INT32_VECT2_LSHIFT(guidance_h_accel_ref, gh_accel_ref, (INT32_ACCEL_FRAC - GH_ACCEL_REF_FRAC));
  } else {
    VECT2_COPY(guidance_h_pos_ref, guidance_h_pos_sp);
    INT_VECT2_ZERO(guidance_h_speed_ref);
    INT_VECT2_ZERO(guidance_h_accel_ref);
  }
}


#define MAX_POS_ERR   POS_BFP_OF_REAL(16.)
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(16.)
#define MAX_POS_ERR_SUM ((int32_t)(MAX_POS_ERR)<< 12)

/* with a pgain of 100 and a scale of 2,
 * you get an angle of 5.6 degrees for 1m pos error */
#define GH_GAIN_SCALE 2

/** maximum bank angle: default 20 deg */
#define TRAJ_MAX_BANK BFP_OF_REAL(GUIDANCE_H_MAX_BANK, INT32_ANGLE_FRAC)

static inline void guidance_h_traj_run(bool_t in_flight) {

  /* compute position error    */
  VECT2_DIFF(guidance_h_pos_err, guidance_h_pos_ref, *stateGetPositionNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_h_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  /* compute speed error    */
  VECT2_DIFF(guidance_h_speed_err, guidance_h_speed_ref, *stateGetSpeedNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_h_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

  /* update pos error integral, zero it if not in_flight */
  if (in_flight) {
    VECT2_ADD(guidance_h_pos_err_sum, guidance_h_pos_err);
    /* saturate it               */
    VECT2_STRIM(guidance_h_pos_err_sum, -MAX_POS_ERR_SUM, MAX_POS_ERR_SUM);
  } else {
    INT_VECT2_ZERO(guidance_h_pos_err_sum);
  }

  /* run PID */
  guidance_h_command_earth.x =
    ((guidance_h_pgain * guidance_h_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_dgain * (guidance_h_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2)) +
    ((guidance_h_igain * (guidance_h_pos_err_sum.x >> 12)) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_again * guidance_h_accel_ref.x) >> 8); /* feedforward gain */
  guidance_h_command_earth.y =
    ((guidance_h_pgain * guidance_h_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_dgain * (guidance_h_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2)) +
    ((guidance_h_igain * (guidance_h_pos_err_sum.y >> 12)) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_again * guidance_h_accel_ref.y) >> 8); /* feedforward gain */

  VECT2_STRIM(guidance_h_command_earth, -TRAJ_MAX_BANK, TRAJ_MAX_BANK);

  /* Rotate to body frame */
  int32_t s_psi, c_psi;
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);

  // Restore angle ref resolution after rotation
  guidance_h_command_body.phi =
    ( - s_psi * guidance_h_command_earth.x + c_psi * guidance_h_command_earth.y) >> INT32_TRIG_FRAC;
  guidance_h_command_body.theta =
    - ( c_psi * guidance_h_command_earth.x + s_psi * guidance_h_command_earth.y) >> INT32_TRIG_FRAC;


  /* Add RC roll and pitch setpoints for emergency corrections */
  guidance_h_command_body.phi += guidance_h_rc_sp.phi;
  guidance_h_command_body.theta += guidance_h_rc_sp.theta;

  /* Set attitude setpoint in eulers and as quaternion */
  EULERS_COPY(stab_att_sp_euler, guidance_h_command_body);

#ifdef STABILIZATION_ATTITUDE_TYPE_QUAT
  INT32_QUAT_OF_EULERS(stab_att_sp_quat, stab_att_sp_euler);
  INT32_QUAT_WRAP_SHORTEST(stab_att_sp_quat);
#endif /* STABILIZATION_ATTITUDE_TYPE_QUAT */

}

static inline void guidance_h_hover_enter(void) {

  VECT2_COPY(guidance_h_pos_sp, *stateGetPositionNed_i());

  guidance_h_rc_sp.psi = stateGetNedToBodyEulers_i()->psi;
  reset_psi_ref_from_body();

  INT_VECT2_ZERO(guidance_h_pos_err_sum);

}

static inline void guidance_h_nav_enter(void) {

  INT32_VECT2_NED_OF_ENU(guidance_h_pos_sp, navigation_carrot);
  struct Int32Vect2 pos,speed,zero;
  INT_VECT2_ZERO(zero);
  VECT2_COPY(pos, *stateGetPositionNed_i());
  VECT2_COPY(speed, *stateGetSpeedNed_i());
  GuidanceHSetRef(pos, speed, zero);

  /* reset psi reference, set psi setpoint to current psi */
  reset_psi_ref_from_body();
  nav_heading = stateGetNedToBodyEulers_i()->psi;

  INT_VECT2_ZERO(guidance_h_pos_err_sum);

}
