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

#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_module.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "modules/radio_control/radio_control.h"
#include "modules/core/abi.h"

/* for guidance_v.thrust_coeff */
#include "firmwares/rotorcraft/guidance/guidance_v.h"

#include "state.h"

PRINT_CONFIG_VAR(GUIDANCE_H_USE_REF)
PRINT_CONFIG_VAR(GUIDANCE_H_USE_SPEED_REF)

struct HorizontalGuidance guidance_h;

/** horizontal guidance command.
 * In north/east with #INT32_ANGLE_FRAC
 */
struct StabilizationSetpoint guidance_h_cmd;

static void guidance_h_update_reference(void);

#ifndef GUIDANCE_H_RC_ID
#define GUIDANCE_H_RC_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(GUIDANCE_H_RC_ID)
static abi_event rc_ev;
static void rc_cb(uint8_t sender_id UNUSED, struct RadioControl *rc);

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

#endif

void guidance_h_init(void)
{

  guidance_h.mode = GUIDANCE_H_MODE_NONE;
  guidance_h.use_ref = GUIDANCE_H_USE_REF;

  INT_VECT2_ZERO(guidance_h.sp.pos);
  guidance_h.sp.heading = 0.f;
  guidance_h.sp.heading_rate = 0.f;
  guidance_h.sp.h_mask = GUIDANCE_H_SP_POS;
  guidance_h.sp.yaw_mask = GUIDANCE_H_SP_YAW;
  INT_VECT2_ZERO(guidance_h.rc_sp.vect);
  guidance_h.rc_sp.heading = 0.f;
  guidance_h.rc_sp.last_ts = 0.f;

  gh_ref_init();

  // bind ABI messages
  AbiBindMsgRADIO_CONTROL(GUIDANCE_H_RC_ID, &rc_ev, rc_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_H_INT, send_gh);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_H_REF_INT, send_href);
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
    case GUIDANCE_H_MODE_HOVER:
    case GUIDANCE_H_MODE_GUIDED:
      guidance_h_hover_enter();
      break;
    case GUIDANCE_H_MODE_NAV:
      guidance_h_nav_enter();
      break;
    default:
      break;
  }

  guidance_h.mode = new_mode;

}

// If not defined, use attitude max yaw setpoint (or 60 deg/s) by default
#ifndef GUIDANCE_H_SP_MAX_R
#ifdef STABILIZATION_ATTITUDE_SP_MAX_R
#define GUIDANCE_H_SP_MAX_R STABILIZATION_ATTITUDE_SP_MAX_R
#else
#define GUIDANCE_H_SP_MAX_R 60.f
#endif
#endif

#ifndef GUIDANCE_H_DEADBAND_R
#define GUIDANCE_H_DEADBAND_R 200
#endif

#define YAW_DEADBAND_EXCEEDED(_rc)                               \
  (rc->values[RADIO_YAW] >  GUIDANCE_H_DEADBAND_R || \
   rc->values[RADIO_YAW] < -GUIDANCE_H_DEADBAND_R)

static void read_rc_setpoint_heading(struct HorizontalGuidanceRCInput *rc_sp, bool in_flight, struct RadioControl *rc)
{
  if (in_flight) {
    /* calculate dt for yaw integration */
    float dt = get_sys_time_float() - rc_sp->last_ts;
    /* make sure nothing drastically weird happens, bound dt to 0.5sec */
    Bound(dt, 0, 0.5);

    /* do not advance yaw setpoint if within a small deadband around stick center or if throttle is zero */
    if (YAW_DEADBAND_EXCEEDED(rc) && !THROTTLE_STICK_DOWN_FROM_RC(rc)) {
      float heading_rate = (float) rc->values[RADIO_YAW] * GUIDANCE_H_SP_MAX_R / MAX_PPRZ;
      rc_sp->heading += heading_rate * dt;
      FLOAT_ANGLE_NORMALIZE(rc_sp->heading);
    }
  } else { /* if not flying, use current yaw as setpoint */
    rc_sp->heading = stateGetNedToBodyEulers_f()->psi;
  }
  /* update timestamp for dt calculation */
  rc_sp->last_ts = get_sys_time_float();
}

/// read speed setpoint from RC
static void read_rc_setpoint_speed_i(struct Int32Vect2 *speed_sp, bool in_flight, struct RadioControl *rc)
{
  if (in_flight) {
    // negative pitch is forward
    int64_t rc_x = -rc->values[RADIO_PITCH];
    int64_t rc_y = rc->values[RADIO_ROLL];
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

static void rc_cb(uint8_t sender_id UNUSED, struct RadioControl *rc)
{
  switch (guidance_h.mode) {

    case GUIDANCE_H_MODE_HOVER:
      read_rc_setpoint_heading(&guidance_h.rc_sp, autopilot_in_flight(), rc);
      read_rc_setpoint_speed_i(&guidance_h.rc_sp.vect, autopilot_in_flight(), rc);
#if GUIDANCE_H_USE_SPEED_REF
      /* enable x,y velocity setpoints */
      guidance_h.sp.speed = guidance_h.rc_sp.vect;
      guidance_h.sp.h_mask = GUIDANCE_H_SP_SPEED;
#endif
      break;
    case GUIDANCE_H_MODE_NAV:
      INT_VECT2_ZERO(guidance_h.rc_sp.vect);
      if (radio_control.status == RC_OK) {
        read_rc_setpoint_heading(&guidance_h.rc_sp, autopilot_in_flight(), rc);
      }
      break;
    default:
      break;
  }

}

struct StabilizationSetpoint guidance_h_run(bool in_flight)
{
  struct StabilizationSetpoint sp;
  STAB_SP_SET_EULERS_ZERO(sp);

  switch (guidance_h.mode) {

    case GUIDANCE_H_MODE_HOVER:
      /* set psi command from RC */
      guidance_h.sp.heading = guidance_h.rc_sp.heading;
      /* fall trough to GUIDED to update ref, run traj and set final attitude setpoint */

      /* Falls through. */
    case GUIDANCE_H_MODE_GUIDED:
      sp = guidance_h_guided_run(in_flight);
      break;

    case GUIDANCE_H_MODE_NAV:
      sp = guidance_h_from_nav(in_flight);
      break;

    default:
      break;
  }

  return sp;
}


static void guidance_h_update_reference(void)
{
  /* compute reference even if usage temporarily disabled via guidance_h_use_ref */
#if GUIDANCE_H_USE_REF
  if (guidance_h.sp.h_mask == GUIDANCE_H_SP_ACCEL) {
    struct FloatVect2 sp_accel_local;
    sp_accel_local.x = ACCEL_FLOAT_OF_BFP(guidance_h.sp.accel.x);
    sp_accel_local.y = ACCEL_FLOAT_OF_BFP(guidance_h.sp.accel.y);
    gh_update_ref_from_accel_sp(sp_accel_local);
  }
  else if (guidance_h.sp.h_mask == GUIDANCE_H_SP_SPEED) {
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
    switch (nav.setpoint_mode) {
      case NAV_SETPOINT_MODE_SPEED:
        guidance_h.ref.pos.x = stateGetPositionNed_i()->x;
        guidance_h.ref.pos.y = stateGetPositionNed_i()->y;
        guidance_h.ref.speed.x = guidance_h.sp.speed.x;
        guidance_h.ref.speed.y = guidance_h.sp.speed.y;
        guidance_h.ref.accel.x = 0;
        guidance_h.ref.accel.y = 0;
        break;

      case NAV_SETPOINT_MODE_ACCEL:
        guidance_h.ref.pos.x = stateGetPositionNed_i()->x;
        guidance_h.ref.pos.y = stateGetPositionNed_i()->y;
        guidance_h.ref.speed.x = stateGetSpeedNed_i()->x;
        guidance_h.ref.speed.y = stateGetSpeedNed_i()->y;
        guidance_h.ref.accel.x = guidance_h.sp.accel.x;
        guidance_h.ref.accel.y = guidance_h.sp.accel.y;
        break;

      case NAV_SETPOINT_MODE_POS:
      default: // Fallback is guidance by pos
        VECT2_COPY(guidance_h.ref.pos, guidance_h.sp.pos);
        INT_VECT2_ZERO(guidance_h.ref.speed);
        INT_VECT2_ZERO(guidance_h.ref.accel);
        break;
    }
  }

#if GUIDANCE_H_USE_SPEED_REF
  if (guidance_h.mode == GUIDANCE_H_MODE_HOVER) {
    VECT2_COPY(guidance_h.sp.pos, guidance_h.ref.pos); // for display only
  }
#endif

  /* update heading setpoint from rate */
  if (guidance_h.sp.yaw_mask == GUIDANCE_H_SP_YAW_RATE) {
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
  guidance_h.rc_sp.heading = stateGetNedToBodyEulers_f()->psi;
  guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);

  /* call specific implementation */
  guidance_h_run_enter();
}

void guidance_h_nav_enter(void)
{
  /* horizontal position setpoint from navigation/flightplan */
  guidance_h_set_pos(nav.carrot.y, nav.carrot.x);
  reset_guidance_reference_from_current_position();

  /* call specific implementation */
  guidance_h_run_enter();
  guidance_h_set_heading(nav.heading);
}

struct StabilizationSetpoint guidance_h_from_nav(bool in_flight)
{
  if (!in_flight) {
    guidance_h_nav_enter();
  }

  if (nav.fp_max_speed > 0.f) {
    guidance_h_SetMaxSpeed(nav.fp_max_speed);
  }
  if (nav.horizontal_mode == NAV_HORIZONTAL_MODE_NONE) {
    struct StabilizationSetpoint sp;
    STAB_SP_SET_EULERS_ZERO(sp);
    return sp; // don't call guidance, still return attitude zero
  } else if (nav.horizontal_mode == NAV_HORIZONTAL_MODE_ATTITUDE) {
    if (nav.setpoint_mode == NAV_SETPOINT_MODE_QUAT) {
      return stab_sp_from_quat_f(&nav.quat);
    }
    else {
      // it should be nav.setpoint_mode == NAV_SETPOINT_MODE_ATTITUDE
      // TODO error handling ?
      struct FloatEulers sp_cmd_f = {
        .phi = nav.roll,
        .theta = nav.pitch,
        .psi = nav.heading
      };
      return stab_sp_from_eulers_f(&sp_cmd_f);
    }
  } else if (nav.horizontal_mode == NAV_HORIZONTAL_MODE_GUIDED) {
    return guidance_h_guided_run(in_flight);
  } else {
    // update carrot for GCS display and convert ENU float -> NED int
    // even if sp is changed later
    guidance_h.sp.pos.x = POS_BFP_OF_REAL(nav.carrot.y);
    guidance_h.sp.pos.y = POS_BFP_OF_REAL(nav.carrot.x);

    switch (nav.setpoint_mode) {
      case NAV_SETPOINT_MODE_POS:
        guidance_h_set_pos(nav.carrot.y, nav.carrot.x); // nav pos is in ENU frame, convert to NED
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
        guidance_h_set_acc(nav.accel.y, nav.accel.x); // nav acc is in ENU frame, convert to NED
        guidance_h_update_reference();
        guidance_h_set_heading(nav.heading);
        guidance_h_cmd = guidance_h_run_accel(in_flight, &guidance_h);
        break;

      default:
        // nothing to do for other cases at the moment
        break;
    }
    /* return final attitude setpoint */
    return guidance_h_cmd;
  }
}

struct StabilizationSetpoint guidance_h_guided_run(bool in_flight)
{
  /* guidance_h.sp.pos and guidance_h.sp.heading need to be set from external source */
  if (!in_flight) {
    guidance_h_hover_enter();
  }

  guidance_h_update_reference();

  guidance_h_cmd = guidance_h_run_pos(in_flight, &guidance_h);
  /* return final attitude setpoint */
  return guidance_h_cmd;
}

void guidance_h_set_pos(float x, float y)
{
  if (guidance_h.sp.h_mask != GUIDANCE_H_SP_POS) {
    reset_guidance_reference_from_current_position();
  }
  guidance_h.sp.h_mask = GUIDANCE_H_SP_POS;
  guidance_h.sp.pos.x = POS_BFP_OF_REAL(x);
  guidance_h.sp.pos.y = POS_BFP_OF_REAL(y);
}

void guidance_h_set_heading(float heading)
{
  guidance_h.sp.yaw_mask = GUIDANCE_H_SP_YAW;
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
  if (guidance_h.sp.h_mask != GUIDANCE_H_SP_SPEED) {
    reset_guidance_reference_from_current_position();
  }
  guidance_h.sp.h_mask = GUIDANCE_H_SP_SPEED;
  guidance_h.sp.speed.x = SPEED_BFP_OF_REAL(vx);
  guidance_h.sp.speed.y = SPEED_BFP_OF_REAL(vy);
}

void guidance_h_set_body_acc(float ax, float ay)
{
  float psi = stateGetNedToBodyEulers_f()->psi;
  float newax =  cosf(-psi) * ax + sinf(-psi) * ay;
  float neway = -sinf(-psi) * ax + cosf(-psi) * ay;
  guidance_h_set_acc(newax, neway);
}

void guidance_h_set_acc(float ax, float ay)
{
  if (guidance_h.sp.h_mask != GUIDANCE_H_SP_ACCEL) {
    reset_guidance_reference_from_current_position();
  }
  guidance_h.sp.h_mask = GUIDANCE_H_SP_ACCEL;
  guidance_h.sp.accel.x = ACCEL_BFP_OF_REAL(ax);
  guidance_h.sp.accel.y = ACCEL_BFP_OF_REAL(ay);
}

void guidance_h_set_heading_rate(float rate)
{
  guidance_h.sp.yaw_mask = GUIDANCE_H_SP_YAW_RATE;
  guidance_h.sp.heading_rate = rate;
}

