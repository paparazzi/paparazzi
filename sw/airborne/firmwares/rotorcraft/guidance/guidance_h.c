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

#include "generated/airframe.h"

#include "firmwares/rotorcraft/guidance/guidance_hybrid.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_flip.h"
#include "firmwares/rotorcraft/guidance/guidance_module.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/radio_control/radio_control.h"

#include "firmwares/rotorcraft/stabilization/stabilization_none.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

/* for guidance_v.thrust_coeff */
#include "firmwares/rotorcraft/guidance/guidance_v.h"

#include "state.h"

PRINT_CONFIG_VAR(GUIDANCE_H_USE_REF)
PRINT_CONFIG_VAR(GUIDANCE_H_USE_SPEED_REF)

struct HorizontalGuidance guidance_h;

int32_t transition_percentage;

/** horizontal guidance command.
 * In north/east with #INT32_ANGLE_FRAC
 */
struct StabilizationSetpoint guidance_h_cmd;

static void guidance_h_update_reference(void);
static inline void transition_run(bool to_forward);
static void read_rc_setpoint_speed_i(struct Int32Vect2 *speed_sp, bool in_flight);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_gh(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_i *pos = stateGetPositionNed_i();
  pprz_msg_send_GUIDANCE_H_INT(trans, dev, AC_ID,
                               &guidance_h.sp.pos.x, &guidance_h.sp.pos.y,
                               &guidance_h.ref.pos.x, &guidance_h.ref.pos.y,
                               &(pos->x), &(pos->y));
}

static void send_href(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE_H_REF_INT(trans, dev, AC_ID,
                                   &guidance_h.sp.pos.x, &guidance_h.ref.pos.x,
                                   &guidance_h.sp.speed.x, &guidance_h.ref.speed.x,
                                   &guidance_h.ref.accel.x,
                                   &guidance_h.sp.pos.y, &guidance_h.ref.pos.y,
                                   &guidance_h.sp.speed.y, &guidance_h.ref.speed.y,
                                   &guidance_h.ref.accel.y);
}

static void send_tune_hover(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ROTORCRAFT_TUNE_HOVER(trans, dev, AC_ID,
                                      &radio_control.values[RADIO_ROLL],
                                      &radio_control.values[RADIO_PITCH],
                                      &radio_control.values[RADIO_YAW],
                                      &stabilization_cmd[COMMAND_ROLL],
                                      &stabilization_cmd[COMMAND_PITCH],
                                      &stabilization_cmd[COMMAND_YAW],
                                      &stabilization_cmd[COMMAND_THRUST],
                                      &(stateGetNedToBodyEulers_i()->phi),
                                      &(stateGetNedToBodyEulers_i()->theta),
                                      &(stateGetNedToBodyEulers_i()->psi));
}

#endif

void guidance_h_init(void)
{

  guidance_h.mode = GUIDANCE_H_MODE_KILL;
  guidance_h.use_ref = GUIDANCE_H_USE_REF;

  INT_VECT2_ZERO(guidance_h.sp.pos);
  FLOAT_EULERS_ZERO(guidance_h.rc_sp);
  guidance_h.sp.heading = 0.0;
  guidance_h.sp.heading_rate = 0.0;
  transition_percentage = 0;
  transition_theta_offset = 0;

  gh_ref_init();

#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE
  guidance_h_module_init();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_H_INT, send_gh);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_H_REF_INT, send_href);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_TUNE_HOVER, send_tune_hover);
#endif

}


static inline void reset_guidance_reference_from_current_position(void)
{
  VECT2_COPY(guidance_h.ref.pos, *stateGetPositionNed_i());
  VECT2_COPY(guidance_h.ref.speed, *stateGetSpeedNed_i());
  struct FloatVect2 ref_speed;
  ref_speed.x = SPEED_FLOAT_OF_BFP(guidance_h.ref.speed.x);
  ref_speed.y = SPEED_FLOAT_OF_BFP(guidance_h.ref.speed.y);

  INT_VECT2_ZERO(guidance_h.ref.accel);
  struct FloatVect2 ref_accel;
  FLOAT_VECT2_ZERO(ref_accel);
  gh_set_ref(guidance_h.ref.pos, ref_speed, ref_accel);
}

void guidance_h_mode_changed(uint8_t new_mode)
{
  if (new_mode == guidance_h.mode) {
    return;
  }

  switch (new_mode) {
    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_enter();
      break;

#if USE_STABILIZATION_RATE
    case GUIDANCE_H_MODE_RATE:
      stabilization_rate_enter();
      break;
#endif

    case GUIDANCE_H_MODE_CARE_FREE:
      stabilization_attitude_reset_care_free_heading();
      /* Falls through. */
    case GUIDANCE_H_MODE_FORWARD:
    case GUIDANCE_H_MODE_ATTITUDE:
#if NO_ATTITUDE_RESET_ON_MODE_CHANGE
      /* reset attitude stabilization if previous mode was not using it */
      if (guidance_h.mode == GUIDANCE_H_MODE_KILL ||
          guidance_h.mode == GUIDANCE_H_MODE_RATE ||
          guidance_h.mode == GUIDANCE_H_MODE_RC_DIRECT)
#endif
        stabilization_attitude_enter();
      break;

    case GUIDANCE_H_MODE_GUIDED:
    case GUIDANCE_H_MODE_HOVER:
      guidance_h_hover_enter();
#if NO_ATTITUDE_RESET_ON_MODE_CHANGE
      /* reset attitude stabilization if previous mode was not using it */
      if (guidance_h.mode == GUIDANCE_H_MODE_KILL ||
          guidance_h.mode == GUIDANCE_H_MODE_RATE ||
          guidance_h.mode == GUIDANCE_H_MODE_RC_DIRECT)
#endif
        stabilization_attitude_enter();
      break;

#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE
    case GUIDANCE_H_MODE_MODULE:
      guidance_h_module_enter();
      break;
#endif

    case GUIDANCE_H_MODE_NAV:
      guidance_h_nav_enter();
#if NO_ATTITUDE_RESET_ON_MODE_CHANGE
      /* reset attitude stabilization if previous mode was not using it */
      if (guidance_h.mode == GUIDANCE_H_MODE_KILL ||
          guidance_h.mode == GUIDANCE_H_MODE_RATE ||
          guidance_h.mode == GUIDANCE_H_MODE_RC_DIRECT)
#endif
        stabilization_attitude_enter();
      break;

    case GUIDANCE_H_MODE_FLIP:
      guidance_flip_enter();
      break;

    default:
      break;
  }

  guidance_h.mode = new_mode;

}


void guidance_h_read_rc(bool  in_flight)
{

  switch (guidance_h.mode) {

    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_read_rc();
      break;

#if USE_STABILIZATION_RATE
    case GUIDANCE_H_MODE_RATE:
#if SWITCH_STICKS_FOR_RATE_CONTROL
      stabilization_rate_read_rc_switched_sticks();
#else
      stabilization_rate_read_rc();
#endif
      break;
#endif

    case GUIDANCE_H_MODE_CARE_FREE:
      stabilization_attitude_read_rc(in_flight, TRUE, FALSE);
      break;
    case GUIDANCE_H_MODE_FORWARD:
      stabilization_attitude_read_rc(in_flight, FALSE, TRUE);
      break;
    case GUIDANCE_H_MODE_ATTITUDE:
      stabilization_attitude_read_rc(in_flight, FALSE, FALSE);
      break;
    case GUIDANCE_H_MODE_HOVER:
      stabilization_attitude_read_rc_setpoint_eulers_f(&guidance_h.rc_sp, in_flight, FALSE, FALSE);
#if GUIDANCE_H_USE_SPEED_REF
      read_rc_setpoint_speed_i(&guidance_h.sp.speed, in_flight);
      /* enable x,y velocity setpoints */
      SetBit(guidance_h.sp.mask, 5);
#endif
      break;

#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE
    case GUIDANCE_H_MODE_MODULE:
      guidance_h_module_read_rc();
      break;
#endif

    case GUIDANCE_H_MODE_NAV:
      if (radio_control.status == RC_OK) {
        stabilization_attitude_read_rc_setpoint_eulers_f(&guidance_h.rc_sp, in_flight, FALSE, FALSE);
      } else {
        FLOAT_EULERS_ZERO(guidance_h.rc_sp);
      }
      break;
    case GUIDANCE_H_MODE_FLIP:
      stabilization_attitude_read_rc(in_flight, FALSE, FALSE);
      break;
    default:
      break;
  }

}

void guidance_h_run(bool  in_flight)
{
  switch (guidance_h.mode) {

    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_run(in_flight);
      break;

#if USE_STABILIZATION_RATE
    case GUIDANCE_H_MODE_RATE:
      stabilization_rate_run(in_flight);
      break;
#endif

    case GUIDANCE_H_MODE_FORWARD:
      if (transition_percentage < (100 << INT32_PERCENTAGE_FRAC)) {
        transition_run(true);
      }
      /* Falls through. */
    case GUIDANCE_H_MODE_CARE_FREE:
    case GUIDANCE_H_MODE_ATTITUDE:
      if ((!(guidance_h.mode == GUIDANCE_H_MODE_FORWARD)) && transition_percentage > 0) {
        transition_run(false);
      }
      stabilization_attitude_run(in_flight);
#if (STABILIZATION_FILTER_CMD_ROLL_PITCH || STABILIZATION_FILTER_CMD_YAW)
      if (in_flight) {
        stabilization_filter_commands();
      }
#endif

      break;

    case GUIDANCE_H_MODE_HOVER:
      /* set psi command from RC */
      guidance_h.sp.heading = guidance_h.rc_sp.psi;
      /* fall trough to GUIDED to update ref, run traj and set final attitude setpoint */

      /* Falls through. */
    case GUIDANCE_H_MODE_GUIDED:
      guidance_h_guided_run(in_flight);
      break;

    case GUIDANCE_H_MODE_NAV:
      guidance_h_from_nav(in_flight);
      break;

#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE
    case GUIDANCE_H_MODE_MODULE:
      guidance_h_module_run(in_flight);
      break;
#endif

    case GUIDANCE_H_MODE_FLIP:
      guidance_flip_run();
      break;

    default:
      break;
  }
}


static void guidance_h_update_reference(void)
{
  /* compute reference even if usage temporarily disabled via guidance_h_use_ref */
#if GUIDANCE_H_USE_REF
  if (bit_is_set(guidance_h.sp.mask, 5)) {
    struct FloatVect2 sp_speed;
    sp_speed.x = SPEED_FLOAT_OF_BFP(guidance_h.sp.speed.x);
    sp_speed.y = SPEED_FLOAT_OF_BFP(guidance_h.sp.speed.y);
    gh_update_ref_from_speed_sp(sp_speed);
  } else {
    gh_update_ref_from_pos_sp(guidance_h.sp.pos);
  }
#endif

  /* either use the reference or simply copy the pos setpoint */
  if (guidance_h.use_ref) {
    /* convert our reference to generic representation */
    INT32_VECT2_RSHIFT(guidance_h.ref.pos,   gh_ref.pos, (GH_POS_REF_FRAC - INT32_POS_FRAC));
    guidance_h.ref.speed.x = SPEED_BFP_OF_REAL(gh_ref.speed.x);
    guidance_h.ref.speed.y = SPEED_BFP_OF_REAL(gh_ref.speed.y);
    guidance_h.ref.accel.x = ACCEL_BFP_OF_REAL(gh_ref.accel.x);
    guidance_h.ref.accel.y = ACCEL_BFP_OF_REAL(gh_ref.accel.y);
  } else {
    VECT2_COPY(guidance_h.ref.pos, guidance_h.sp.pos);
    INT_VECT2_ZERO(guidance_h.ref.speed);
    INT_VECT2_ZERO(guidance_h.ref.accel);
  }

#if GUIDANCE_H_USE_SPEED_REF
  if (guidance_h.mode == GUIDANCE_H_MODE_HOVER) {
    VECT2_COPY(guidance_h.sp.pos, guidance_h.ref.pos); // for display only
  }
#endif

  /* update heading setpoint from rate */
  if (bit_is_set(guidance_h.sp.mask, 7)) {
    guidance_h.sp.heading += guidance_h.sp.heading_rate / PERIODIC_FREQUENCY;
    FLOAT_ANGLE_NORMALIZE(guidance_h.sp.heading);
  }
}

void guidance_h_hover_enter(void)
{
  /* reset speed setting */
  guidance_h.sp.speed.x = 0;
  guidance_h.sp.speed.y = 0;

  /* set horizontal setpoint to current position */
  guidance_h_set_pos(
      stateGetPositionNed_f()->x,
      stateGetPositionNed_f()->y);
  /* reset guidance reference */
  reset_guidance_reference_from_current_position();

  /* set guidance to current heading and position */
  guidance_h.rc_sp.psi = stateGetNedToBodyEulers_f()->psi;
  guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);

  /* call specific implementation */
  guidance_h_run_enter();
}

void guidance_h_nav_enter(void)
{
  /* horizontal position setpoint from navigation/flightplan */
  guidance_h_set_pos(nav.carrot.y, nav.carrot.x);
  reset_guidance_reference_from_current_position();
  /* set nav_heading to current heading */
  nav.heading = stateGetNedToBodyEulers_f()->psi;
  guidance_h_set_heading(nav.heading);
  /* call specific implementation */
  guidance_h_run_enter();
}

void guidance_h_from_nav(bool in_flight)
{
  if (!in_flight) {
    guidance_h_nav_enter();
  }

  if (nav.horizontal_mode == NAV_HORIZONTAL_MODE_MANUAL) {
    stabilization_cmd[COMMAND_ROLL]  = nav.cmd_roll;
    stabilization_cmd[COMMAND_PITCH] = nav.cmd_pitch;
    stabilization_cmd[COMMAND_YAW]   = nav.cmd_yaw;
  } else if (nav.horizontal_mode == NAV_HORIZONTAL_MODE_ATTITUDE) {
    if (nav.setpoint_mode == NAV_SETPOINT_MODE_QUAT) {
      // directly apply quat setpoint
      struct Int32Quat quat_i;
      QUAT_BFP_OF_REAL(quat_i, nav.quat);
      stabilization_attitude_set_quat_setpoint_i(&quat_i);
      stabilization_attitude_run(in_flight);
    }
    else {
      // it should be nav.setpoint_mode == NAV_SETPOINT_MODE_ATTITUDE
      // TODO error handling ?
      struct Int32Eulers sp_cmd_i;
      sp_cmd_i.phi = ANGLE_BFP_OF_REAL(nav.roll);
      sp_cmd_i.theta = ANGLE_BFP_OF_REAL(nav.pitch);
      sp_cmd_i.psi = ANGLE_BFP_OF_REAL(nav.heading);
      stabilization_attitude_set_rpy_setpoint_i(&sp_cmd_i);
      stabilization_attitude_run(in_flight);
    }
  } else if (nav.horizontal_mode == NAV_HORIZONTAL_MODE_GUIDED) {
    guidance_h_guided_run(in_flight);
  } else {
    // update carrot for display, even if sp is changed in speed mode
    guidance_h_set_pos(nav.carrot.y, nav.carrot.x);
    switch (nav.setpoint_mode) {
      case NAV_SETPOINT_MODE_POS:
        // set guidance in NED
        guidance_h_update_reference();
        guidance_h_set_heading(nav.heading);
        guidance_h_cmd = guidance_h_run_pos(in_flight, &guidance_h);
        break;

      case NAV_SETPOINT_MODE_SPEED:
        guidance_h_set_vel(nav.speed.y, nav.speed.x); // nav speed is in ENU frame, convert to NED
        guidance_h_update_reference();
        guidance_h_set_heading(nav.heading);
        guidance_h_cmd = guidance_h_run_speed(in_flight, &guidance_h);
        break;

      case NAV_SETPOINT_MODE_ACCEL:
        // TODO set_accel ref
        guidance_h_set_heading(nav.heading);
        guidance_h_cmd = guidance_h_run_accel(in_flight, &guidance_h);
        break;

      default:
        // nothing to do for other cases at the moment
        break;
    }
    /* set final attitude setpoint */
    stabilization_attitude_set_stab_sp(&guidance_h_cmd);
    stabilization_attitude_run(in_flight);

  }
}

static inline void transition_run(bool to_forward)
{
  if (to_forward) {
    //Add 0.00625%
    transition_percentage += 1 << (INT32_PERCENTAGE_FRAC - 4);
  } else {
    //Subtract 0.00625%
    transition_percentage -= 1 << (INT32_PERCENTAGE_FRAC - 4);
  }

#ifdef TRANSITION_MAX_OFFSET
  const int32_t max_offset = ANGLE_BFP_OF_REAL(TRANSITION_MAX_OFFSET);
  transition_theta_offset = INT_MULT_RSHIFT((transition_percentage << (INT32_ANGLE_FRAC - INT32_PERCENTAGE_FRAC)) / 100,
                            max_offset, INT32_ANGLE_FRAC);
#endif
}

/// read speed setpoint from RC
static void read_rc_setpoint_speed_i(struct Int32Vect2 *speed_sp, bool in_flight)
{
  if (in_flight) {
    // negative pitch is forward
    int64_t rc_x = -radio_control.values[RADIO_PITCH];
    int64_t rc_y = radio_control.values[RADIO_ROLL];
    DeadBand(rc_x, MAX_PPRZ / 20);
    DeadBand(rc_y, MAX_PPRZ / 20);

    // convert input from MAX_PPRZ range to SPEED_BFP
    int32_t max_speed = SPEED_BFP_OF_REAL(GUIDANCE_H_REF_MAX_SPEED);
    /// @todo calc proper scale while making sure a division by zero can't occur
    //int32_t rc_norm = sqrtf(rc_x * rc_x + rc_y * rc_y);
    //int32_t max_pprz = rc_norm * MAX_PPRZ / Max(abs(rc_x), abs(rc_y);
    rc_x = rc_x * max_speed / MAX_PPRZ;
    rc_y = rc_y * max_speed / MAX_PPRZ;

    /* Rotate from body to NED frame by negative psi angle */
    int32_t psi = -stateGetNedToBodyEulers_i()->psi;
    int32_t s_psi, c_psi;
    PPRZ_ITRIG_SIN(s_psi, psi);
    PPRZ_ITRIG_COS(c_psi, psi);
    speed_sp->x = (int32_t)(((int64_t)c_psi * rc_x + (int64_t)s_psi * rc_y) >> INT32_TRIG_FRAC);
    speed_sp->y = (int32_t)((-(int64_t)s_psi * rc_x + (int64_t)c_psi * rc_y) >> INT32_TRIG_FRAC);
  } else {
    speed_sp->x = 0;
    speed_sp->y = 0;
  }
}

void guidance_h_guided_run(bool in_flight)
{
  /* guidance_h.sp.pos and guidance_h.sp.heading need to be set from external source */
  if (!in_flight) {
    guidance_h_hover_enter();
  }

  guidance_h_update_reference();

  guidance_h_cmd = guidance_h_run_pos(in_flight, &guidance_h);
  /* set final attitude setpoint */
  stabilization_attitude_set_stab_sp(&guidance_h_cmd);
  stabilization_attitude_run(in_flight);
}

void guidance_h_set_pos(float x, float y)
{
  ClearBit(guidance_h.sp.mask, 5);
  guidance_h.sp.pos.x = POS_BFP_OF_REAL(x);
  guidance_h.sp.pos.y = POS_BFP_OF_REAL(y);
}

void guidance_h_set_heading(float heading)
{
  ClearBit(guidance_h.sp.mask, 7);
  guidance_h.sp.heading = heading;
  FLOAT_ANGLE_NORMALIZE(guidance_h.sp.heading);
}

void guidance_h_set_body_vel(float vx, float vy)
{
  float psi = stateGetNedToBodyEulers_f()->psi;
  float newvx =  cosf(-psi) * vx + sinf(-psi) * vy;
  float newvy = -sinf(-psi) * vx + cosf(-psi) * vy;
  guidance_h_set_vel(newvx, newvy);
}

void guidance_h_set_vel(float vx, float vy)
{
  SetBit(guidance_h.sp.mask, 5);
  guidance_h.sp.speed.x = SPEED_BFP_OF_REAL(vx);
  guidance_h.sp.speed.y = SPEED_BFP_OF_REAL(vy);
}

void guidance_h_set_heading_rate(float rate)
{
  SetBit(guidance_h.sp.mask, 7);
  guidance_h.sp.heading_rate = rate;
}

