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
#if GUIDANCE_INDI_HYBRID
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#else
#include "firmwares/rotorcraft/guidance/guidance_indi.h"
#endif

#include "firmwares/rotorcraft/stabilization/stabilization_none.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

/* for guidance_v_thrust_coeff */
#include "firmwares/rotorcraft/guidance/guidance_v.h"

#include "state.h"

#ifndef GUIDANCE_H_AGAIN
#define GUIDANCE_H_AGAIN 0
#endif

#ifndef GUIDANCE_H_VGAIN
#define GUIDANCE_H_VGAIN 0
#endif

/* error if some gains are negative */
#if (GUIDANCE_H_PGAIN < 0) ||                   \
  (GUIDANCE_H_DGAIN < 0)   ||                   \
  (GUIDANCE_H_IGAIN < 0)   ||                   \
  (GUIDANCE_H_AGAIN < 0)   ||                   \
  (GUIDANCE_H_VGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

#ifndef GUIDANCE_H_MAX_BANK
#define GUIDANCE_H_MAX_BANK RadOfDeg(20)
#endif

PRINT_CONFIG_VAR(GUIDANCE_H_USE_REF)
PRINT_CONFIG_VAR(GUIDANCE_H_USE_SPEED_REF)

#ifndef GUIDANCE_H_APPROX_FORCE_BY_THRUST
#define GUIDANCE_H_APPROX_FORCE_BY_THRUST FALSE
#endif

#ifndef GUIDANCE_INDI
#define GUIDANCE_INDI FALSE
#endif

// Navigation can set heading freely
// This is false if sideslip is a problem
#ifndef GUIDANCE_HEADING_IS_FREE
#define GUIDANCE_HEADING_IS_FREE TRUE
#endif

struct HorizontalGuidance guidance_h;

int32_t transition_percentage;

/*
 * internal variables
 */
struct Int32Vect2 guidance_h_pos_err;
struct Int32Vect2 guidance_h_speed_err;
struct Int32Vect2 guidance_h_trim_att_integrator;

/** horizontal guidance command.
 * In north/east with #INT32_ANGLE_FRAC
 * @todo convert to real force command
 */
struct Int32Vect2  guidance_h_cmd_earth;

static void guidance_h_update_reference(void);
#if !GUIDANCE_INDI
static void guidance_h_traj_run(bool in_flight);
#endif
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

static void send_hover_loop(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_i *pos = stateGetPositionNed_i();
  struct NedCoor_i *speed = stateGetSpeedNed_i();
  struct NedCoor_i *accel = stateGetAccelNed_i();
  pprz_msg_send_HOVER_LOOP(trans, dev, AC_ID,
                           &guidance_h.sp.pos.x,
                           &guidance_h.sp.pos.y,
                           &(pos->x), &(pos->y),
                           &(speed->x), &(speed->y),
                           &(accel->x), &(accel->y),
                           &guidance_h_pos_err.x,
                           &guidance_h_pos_err.y,
                           &guidance_h_speed_err.x,
                           &guidance_h_speed_err.y,
                           &guidance_h_trim_att_integrator.x,
                           &guidance_h_trim_att_integrator.y,
                           &guidance_h_cmd_earth.x,
                           &guidance_h_cmd_earth.y,
                           &guidance_h.sp.heading);
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
  guidance_h.approx_force_by_thrust = GUIDANCE_H_APPROX_FORCE_BY_THRUST;

  INT_VECT2_ZERO(guidance_h.sp.pos);
  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  FLOAT_EULERS_ZERO(guidance_h.rc_sp);
  guidance_h.sp.heading = 0.0;
  guidance_h.sp.heading_rate = 0.0;
  guidance_h.gains.p = GUIDANCE_H_PGAIN;
  guidance_h.gains.i = GUIDANCE_H_IGAIN;
  guidance_h.gains.d = GUIDANCE_H_DGAIN;
  guidance_h.gains.a = GUIDANCE_H_AGAIN;
  guidance_h.gains.v = GUIDANCE_H_VGAIN;
  transition_percentage = 0;
  transition_theta_offset = 0;

  gh_ref_init();

#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE
  guidance_h_module_init();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_H_INT, send_gh);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_HOVER_LOOP, send_hover_loop);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_H_REF_INT, send_href);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_TUNE_HOVER, send_tune_hover);
#endif

}


static inline void reset_guidance_reference_from_current_position(void)
{
  VECT2_COPY(guidance_h.ref.pos, *stateGetPositionNed_i());
  VECT2_COPY(guidance_h.ref.speed, *stateGetSpeedNed_i());
  INT_VECT2_ZERO(guidance_h.ref.accel);
  gh_set_ref(guidance_h.ref.pos, guidance_h.ref.speed, guidance_h.ref.accel);

  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
}

void guidance_h_mode_changed(uint8_t new_mode)
{
  if (new_mode == guidance_h.mode) {
    return;
  }

#if HYBRID_NAVIGATION
  guidance_hybrid_norm_ref_airspeed = 0;
#endif

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
#if GUIDANCE_INDI
      guidance_indi_enter();
#endif
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
    gh_update_ref_from_speed_sp(guidance_h.sp.speed);
  } else {
    gh_update_ref_from_pos_sp(guidance_h.sp.pos);
  }
#endif

  /* either use the reference or simply copy the pos setpoint */
  if (guidance_h.use_ref) {
    /* convert our reference to generic representation */
    INT32_VECT2_RSHIFT(guidance_h.ref.pos,   gh_ref.pos, (GH_POS_REF_FRAC - INT32_POS_FRAC));
    INT32_VECT2_LSHIFT(guidance_h.ref.speed, gh_ref.speed, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
    INT32_VECT2_LSHIFT(guidance_h.ref.accel, gh_ref.accel, (INT32_ACCEL_FRAC - GH_ACCEL_REF_FRAC));
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

#define MAX_POS_ERR   POS_BFP_OF_REAL(16.)
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(16.)

#ifndef GUIDANCE_H_THRUST_CMD_FILTER
#define GUIDANCE_H_THRUST_CMD_FILTER 10
#endif

/* with a pgain of 100 and a scale of 2,
 * you get an angle of 5.6 degrees for 1m pos error */
#define GH_GAIN_SCALE 2

#if !GUIDANCE_INDI
static void guidance_h_traj_run(bool in_flight)
{
  /* maximum bank angle: default 20 deg, max 40 deg*/
  static const int32_t traj_max_bank = Min(BFP_OF_REAL(GUIDANCE_H_MAX_BANK, INT32_ANGLE_FRAC),
                                       BFP_OF_REAL(RadOfDeg(40), INT32_ANGLE_FRAC));
  static const int32_t total_max_bank = BFP_OF_REAL(RadOfDeg(45), INT32_ANGLE_FRAC);

  /* compute position error    */
  VECT2_DIFF(guidance_h_pos_err, guidance_h.ref.pos, *stateGetPositionNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_h_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  /* compute speed error    */
  VECT2_DIFF(guidance_h_speed_err, guidance_h.ref.speed, *stateGetSpeedNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_h_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

  /* run PID */
  int32_t pd_x =
    ((guidance_h.gains.p * guidance_h_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h.gains.d * (guidance_h_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  int32_t pd_y =
    ((guidance_h.gains.p * guidance_h_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h.gains.d * (guidance_h_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  guidance_h_cmd_earth.x = pd_x +
                           ((guidance_h.gains.v * guidance_h.ref.speed.x) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE)) + /* speed feedforward gain */
                           ((guidance_h.gains.a * guidance_h.ref.accel.x) >> (INT32_ACCEL_FRAC -
                               GH_GAIN_SCALE));   /* acceleration feedforward gain */
  guidance_h_cmd_earth.y = pd_y +
                           ((guidance_h.gains.v * guidance_h.ref.speed.y) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE)) + /* speed feedforward gain */
                           ((guidance_h.gains.a * guidance_h.ref.accel.y) >> (INT32_ACCEL_FRAC -
                               GH_GAIN_SCALE));   /* acceleration feedforward gain */

  /* trim max bank angle from PD */
  VECT2_STRIM(guidance_h_cmd_earth, -traj_max_bank, traj_max_bank);

  /* Update pos & speed error integral, zero it if not in_flight.
   * Integrate twice as fast when not only POS but also SPEED are wrong,
   * but do not integrate POS errors when the SPEED is already catching up.
   */
  if (in_flight) {
    /* ANGLE_FRAC (12) * GAIN (8) * LOOP_FREQ (9) -> INTEGRATOR HIGH RES ANGLE_FRAX (28) */
    guidance_h_trim_att_integrator.x += (guidance_h.gains.i * pd_x);
    guidance_h_trim_att_integrator.y += (guidance_h.gains.i * pd_y);
    /* saturate it  */
    VECT2_STRIM(guidance_h_trim_att_integrator, -(traj_max_bank << (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2)),
                (traj_max_bank << (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2)));
    /* add it to the command */
    guidance_h_cmd_earth.x += (guidance_h_trim_att_integrator.x >> (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2));
    guidance_h_cmd_earth.y += (guidance_h_trim_att_integrator.y >> (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2));
  } else {
    INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  }

  /* compute a better approximation of force commands by taking thrust into account */
  if (guidance_h.approx_force_by_thrust && in_flight) {
    static int32_t thrust_cmd_filt;
    int32_t vertical_thrust = (stabilization_cmd[COMMAND_THRUST] * guidance_v_thrust_coeff) >> INT32_TRIG_FRAC;
    thrust_cmd_filt = (thrust_cmd_filt * GUIDANCE_H_THRUST_CMD_FILTER + vertical_thrust) /
                      (GUIDANCE_H_THRUST_CMD_FILTER + 1);
    guidance_h_cmd_earth.x = ANGLE_BFP_OF_REAL(atan2f((guidance_h_cmd_earth.x * MAX_PPRZ / INT32_ANGLE_PI_2),
                             thrust_cmd_filt));
    guidance_h_cmd_earth.y = ANGLE_BFP_OF_REAL(atan2f((guidance_h_cmd_earth.y * MAX_PPRZ / INT32_ANGLE_PI_2),
                             thrust_cmd_filt));
  }

  VECT2_STRIM(guidance_h_cmd_earth, -total_max_bank, total_max_bank);
}
#endif

void guidance_h_hover_enter(void)
{
  /* reset speed setting */
  guidance_h.sp.speed.x = 0;
  guidance_h.sp.speed.y = 0;

  /* disable horizontal velocity setpoints,
   * might still be activated in guidance_h_read_rc if GUIDANCE_H_USE_SPEED_REF
   */
  ClearBit(guidance_h.sp.mask, 5);
  ClearBit(guidance_h.sp.mask, 7);

  /* set horizontal setpoint to current position */
  VECT2_COPY(guidance_h.sp.pos, *stateGetPositionNed_i());

  /* reset guidance reference */
  reset_guidance_reference_from_current_position();

  /* set guidance to current heading and position */
  guidance_h.rc_sp.psi = stateGetNedToBodyEulers_f()->psi;
  guidance_h.sp.heading = guidance_h.rc_sp.psi;
}

void guidance_h_nav_enter(void)
{
  ClearBit(guidance_h.sp.mask, 5);
  ClearBit(guidance_h.sp.mask, 7);

  /* horizontal position setpoint from navigation/flightplan */
  INT32_VECT2_NED_OF_ENU(guidance_h.sp.pos, navigation_carrot);

  reset_guidance_reference_from_current_position();

  nav_heading = stateGetNedToBodyEulers_i()->psi;
  guidance_h.sp.heading = stateGetNedToBodyEulers_f()->psi;
}

void guidance_h_from_nav(bool in_flight)
{
  if (!in_flight) {
    guidance_h_nav_enter();
  }

  if (horizontal_mode == HORIZONTAL_MODE_MANUAL) {
    stabilization_cmd[COMMAND_ROLL]  = nav_cmd_roll;
    stabilization_cmd[COMMAND_PITCH] = nav_cmd_pitch;
    stabilization_cmd[COMMAND_YAW]   = nav_cmd_yaw;
  } else if (horizontal_mode == HORIZONTAL_MODE_ATTITUDE) {
    struct Int32Eulers sp_cmd_i;
    sp_cmd_i.phi = nav_roll;
    sp_cmd_i.theta = nav_pitch;
    sp_cmd_i.psi = nav_heading;
    stabilization_attitude_set_rpy_setpoint_i(&sp_cmd_i);
    stabilization_attitude_run(in_flight);

#if HYBRID_NAVIGATION
    //make sure the heading is right before leaving horizontal_mode attitude
    guidance_hybrid_reset_heading(&sp_cmd_i);
#endif
  } else {

#if HYBRID_NAVIGATION
    INT32_VECT2_NED_OF_ENU(guidance_h.sp.pos, navigation_target);
    guidance_hybrid_run();
#else
    INT32_VECT2_NED_OF_ENU(guidance_h.sp.pos, navigation_carrot);

    guidance_h_update_reference();

#if GUIDANCE_HEADING_IS_FREE
    /* set psi command */
    guidance_h.sp.heading = ANGLE_FLOAT_OF_BFP(nav_heading);
    FLOAT_ANGLE_NORMALIZE(guidance_h.sp.heading);
#endif

#if GUIDANCE_INDI
    guidance_indi_run(&guidance_h.sp.heading);
#else
    /* compute x,y earth commands */
    guidance_h_traj_run(in_flight);
    /* set final attitude setpoint */
    int32_t heading_sp_i = ANGLE_BFP_OF_REAL(guidance_h.sp.heading);
    stabilization_attitude_set_earth_cmd_i(&guidance_h_cmd_earth,
                                           heading_sp_i);
#endif

#endif
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

void guidance_h_set_igain(uint32_t igain)
{
  guidance_h.gains.i = igain;
  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
}


void guidance_h_guided_run(bool in_flight)
{
  /* guidance_h.sp.pos and guidance_h.sp.heading need to be set from external source */
  if (!in_flight) {
    guidance_h_hover_enter();
  }

  guidance_h_update_reference();

#if GUIDANCE_INDI
  guidance_indi_run(&guidance_h.sp.heading);
#else
  /* compute x,y earth commands */
  guidance_h_traj_run(in_flight);
  /* set final attitude setpoint */
  int32_t heading_sp_i = ANGLE_BFP_OF_REAL(guidance_h.sp.heading);
  stabilization_attitude_set_earth_cmd_i(&guidance_h_cmd_earth, heading_sp_i);
#endif
  stabilization_attitude_run(in_flight);
}

bool guidance_h_set_guided_pos(float x, float y)
{
  if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED) {
    ClearBit(guidance_h.sp.mask, 5);
    guidance_h.sp.pos.x = POS_BFP_OF_REAL(x);
    guidance_h.sp.pos.y = POS_BFP_OF_REAL(y);
    return true;
  }
  return false;
}

bool guidance_h_set_guided_heading(float heading)
{
  if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED) {
    ClearBit(guidance_h.sp.mask, 7);
    guidance_h.sp.heading = heading;
    FLOAT_ANGLE_NORMALIZE(guidance_h.sp.heading);
    return true;
  }
  return false;
}

bool guidance_h_set_guided_body_vel(float vx, float vy)
{
  float psi = stateGetNedToBodyEulers_f()->psi;
  float newvx =  cosf(-psi) * vx + sinf(-psi) * vy;
  float newvy = -sinf(-psi) * vx + cosf(-psi) * vy;
  return guidance_h_set_guided_vel(newvx, newvy);
}

bool guidance_h_set_guided_vel(float vx, float vy)
{
  if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED) {
    SetBit(guidance_h.sp.mask, 5);
    guidance_h.sp.speed.x = SPEED_BFP_OF_REAL(vx);
    guidance_h.sp.speed.y = SPEED_BFP_OF_REAL(vy);
    return true;
  }
  return false;
}

bool guidance_h_set_guided_heading_rate(float rate)
{
  if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED) {
    SetBit(guidance_h.sp.mask, 7);
    guidance_h.sp.heading_rate = rate;
    return true;
  }
  return false;
}

const struct Int32Vect2 *guidance_h_get_pos_err(void)
{
  return &guidance_h_pos_err;
}
