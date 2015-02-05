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
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/radio_control.h"

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


uint8_t guidance_h_mode;
bool_t guidance_h_use_ref;
bool_t guidance_h_approx_force_by_thrust;

struct Int32Vect2 guidance_h_pos_sp;
struct Int32Vect2 guidance_h_pos_ref;
struct Int32Vect2 guidance_h_speed_ref;
struct Int32Vect2 guidance_h_accel_ref;
#if GUIDANCE_H_USE_SPEED_REF
struct Int32Vect2 guidance_h_speed_sp;
#endif
struct Int32Vect2 guidance_h_pos_err;
struct Int32Vect2 guidance_h_speed_err;
struct Int32Vect2 guidance_h_trim_att_integrator;

struct Int32Vect2  guidance_h_cmd_earth;
struct Int32Eulers guidance_h_rc_sp;
int32_t guidance_h_heading_sp;

int32_t guidance_h_pgain;
int32_t guidance_h_dgain;
int32_t guidance_h_igain;
int32_t guidance_h_again;
int32_t guidance_h_vgain;

int32_t transition_percentage;
int32_t transition_theta_offset;


static void guidance_h_update_reference(void);
static void guidance_h_traj_run(bool_t in_flight);
static void guidance_h_hover_enter(void);
static void guidance_h_nav_enter(void);
static inline void transition_run(void);
static void read_rc_setpoint_speed_i(struct Int32Vect2 *speed_sp, bool_t in_flight);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_gh(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_i *pos = stateGetPositionNed_i();
  pprz_msg_send_GUIDANCE_H_INT(trans, dev, AC_ID,
                               &guidance_h_pos_sp.x, &guidance_h_pos_sp.y,
                               &guidance_h_pos_ref.x, &guidance_h_pos_ref.y,
                               &(pos->x), &(pos->y));
}

static void send_hover_loop(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_i *pos = stateGetPositionNed_i();
  struct NedCoor_i *speed = stateGetSpeedNed_i();
  struct NedCoor_i *accel = stateGetAccelNed_i();
  pprz_msg_send_HOVER_LOOP(trans, dev, AC_ID,
                           &guidance_h_pos_sp.x,
                           &guidance_h_pos_sp.y,
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
                           &guidance_h_heading_sp);
}

static void send_href(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GUIDANCE_H_REF_INT(trans, dev, AC_ID,
                                   &guidance_h_pos_sp.x, &guidance_h_pos_ref.x,
                                   &guidance_h_speed_sp.x, &guidance_h_speed_ref.x,
                                   &guidance_h_accel_ref.x,
                                   &guidance_h_pos_sp.y, &guidance_h_pos_ref.y,
                                   &guidance_h_speed_sp.y, &guidance_h_speed_ref.y,
                                   &guidance_h_accel_ref.y);
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

  guidance_h_mode = GUIDANCE_H_MODE_KILL;
  guidance_h_use_ref = GUIDANCE_H_USE_REF;
  guidance_h_approx_force_by_thrust = GUIDANCE_H_APPROX_FORCE_BY_THRUST;

  INT_VECT2_ZERO(guidance_h_pos_sp);
  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  INT_EULERS_ZERO(guidance_h_rc_sp);
  guidance_h_heading_sp = 0;
  guidance_h_pgain = GUIDANCE_H_PGAIN;
  guidance_h_igain = GUIDANCE_H_IGAIN;
  guidance_h_dgain = GUIDANCE_H_DGAIN;
  guidance_h_again = GUIDANCE_H_AGAIN;
  guidance_h_vgain = GUIDANCE_H_VGAIN;
  transition_percentage = 0;
  transition_theta_offset = 0;

  gh_ref_init();

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "GUIDANCE_H_INT", send_gh);
  register_periodic_telemetry(DefaultPeriodic, "HOVER_LOOP", send_hover_loop);
  register_periodic_telemetry(DefaultPeriodic, "GUIDANCE_H_REF", send_href);
  register_periodic_telemetry(DefaultPeriodic, "ROTORCRAFT_TUNE_HOVER", send_tune_hover);
#endif
}


static inline void reset_guidance_reference_from_current_position(void)
{
  VECT2_COPY(guidance_h_pos_ref, *stateGetPositionNed_i());
  VECT2_COPY(guidance_h_speed_ref, *stateGetSpeedNed_i());
  INT_VECT2_ZERO(guidance_h_accel_ref);
  gh_set_ref(guidance_h_pos_ref, guidance_h_speed_ref, guidance_h_accel_ref);

  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
}

void guidance_h_mode_changed(uint8_t new_mode)
{
  if (new_mode == guidance_h_mode) {
    return;
  }

  if (new_mode != GUIDANCE_H_MODE_FORWARD && new_mode != GUIDANCE_H_MODE_RATE) {
    transition_percentage = 0;
    transition_theta_offset = 0;
  }

  switch (new_mode) {
    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_enter();
      break;

    case GUIDANCE_H_MODE_RATE:
      stabilization_rate_enter();
      break;

    case GUIDANCE_H_MODE_CARE_FREE:
      stabilization_attitude_reset_care_free_heading();
    case GUIDANCE_H_MODE_FORWARD:
    case GUIDANCE_H_MODE_ATTITUDE:
#if NO_ATTITUDE_RESET_ON_MODE_CHANGE
      /* reset attitude stabilization if previous mode was not using it */
      if (guidance_h_mode == GUIDANCE_H_MODE_KILL ||
          guidance_h_mode == GUIDANCE_H_MODE_RATE ||
          guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT)
#endif
        stabilization_attitude_enter();
      break;

    case GUIDANCE_H_MODE_HOVER:
      guidance_h_hover_enter();
#if NO_ATTITUDE_RESET_ON_MODE_CHANGE
      /* reset attitude stabilization if previous mode was not using it */
      if (guidance_h_mode == GUIDANCE_H_MODE_KILL ||
          guidance_h_mode == GUIDANCE_H_MODE_RATE ||
          guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT)
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
      if (guidance_h_mode == GUIDANCE_H_MODE_KILL ||
          guidance_h_mode == GUIDANCE_H_MODE_RATE ||
          guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT)
#endif
        stabilization_attitude_enter();
      break;

    default:
      break;
  }

  guidance_h_mode = new_mode;

}


void guidance_h_read_rc(bool_t  in_flight)
{

  switch (guidance_h_mode) {

    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_read_rc();
      break;

    case GUIDANCE_H_MODE_RATE:
#if SWITCH_STICKS_FOR_RATE_CONTROL
      stabilization_rate_read_rc_switched_sticks();
#else
      stabilization_rate_read_rc();
#endif
      break;
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
      stabilization_attitude_read_rc_setpoint_eulers(&guidance_h_rc_sp, in_flight, FALSE, FALSE);
#if GUIDANCE_H_USE_SPEED_REF
      read_rc_setpoint_speed_i(&guidance_h_speed_sp, in_flight);
#endif
      break;

#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE
    case GUIDANCE_H_MODE_MODULE:
      guidance_h_module_read_rc();
      break;
#endif

    case GUIDANCE_H_MODE_NAV:
      if (radio_control.status == RC_OK) {
        stabilization_attitude_read_rc_setpoint_eulers(&guidance_h_rc_sp, in_flight, FALSE, FALSE);
      } else {
        INT_EULERS_ZERO(guidance_h_rc_sp);
      }
      break;
    default:
      break;
  }

}


void guidance_h_run(bool_t  in_flight)
{
  switch (guidance_h_mode) {

    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_run(in_flight);
      break;

    case GUIDANCE_H_MODE_RATE:
      stabilization_rate_run(in_flight);
      break;

    case GUIDANCE_H_MODE_FORWARD:
      if (transition_percentage < (100 << INT32_PERCENTAGE_FRAC)) {
        transition_run();
      }
    case GUIDANCE_H_MODE_CARE_FREE:
    case GUIDANCE_H_MODE_ATTITUDE:
      stabilization_attitude_run(in_flight);
      break;

    case GUIDANCE_H_MODE_HOVER:
      if (!in_flight) {
        guidance_h_hover_enter();
      }

      guidance_h_update_reference();

      /* set psi command */
      guidance_h_heading_sp = guidance_h_rc_sp.psi;
      /* compute x,y earth commands */
      guidance_h_traj_run(in_flight);
      /* set final attitude setpoint */
      stabilization_attitude_set_earth_cmd_i(&guidance_h_cmd_earth,
                                             guidance_h_heading_sp);
      stabilization_attitude_run(in_flight);
      break;

    case GUIDANCE_H_MODE_NAV:
      if (!in_flight) {
        guidance_h_nav_enter();
      }

      if (horizontal_mode == HORIZONTAL_MODE_ATTITUDE) {
        struct Int32Eulers sp_cmd_i;
        sp_cmd_i.phi = nav_roll;
        sp_cmd_i.theta = nav_pitch;
        /** @todo: heading can't be set via attitude block yet.
         * use current euler psi for now, should be real heading
         */
        sp_cmd_i.psi = stateGetNedToBodyEulers_i()->psi;
        stabilization_attitude_set_rpy_setpoint_i(&sp_cmd_i);
      } else {
        INT32_VECT2_NED_OF_ENU(guidance_h_pos_sp, navigation_carrot);

        guidance_h_update_reference();

        /* set psi command */
        guidance_h_heading_sp = nav_heading;
        INT32_ANGLE_NORMALIZE(guidance_h_heading_sp);
        /* compute x,y earth commands */
        guidance_h_traj_run(in_flight);
        /* set final attitude setpoint */
        stabilization_attitude_set_earth_cmd_i(&guidance_h_cmd_earth,
                                               guidance_h_heading_sp);
      }
      stabilization_attitude_run(in_flight);
      break;

#if GUIDANCE_H_MODE_MODULE_SETTING == GUIDANCE_H_MODE_MODULE
    case GUIDANCE_H_MODE_MODULE:
      guidance_h_module_run(in_flight);
      break;
#endif

    default:
      break;
  }
}


static void guidance_h_update_reference(void)
{
  /* compute reference even if usage temporarily disabled via guidance_h_use_ref */
#if GUIDANCE_H_USE_REF
#if GUIDANCE_H_USE_SPEED_REF
  if (guidance_h_mode == GUIDANCE_H_MODE_HOVER) {
    gh_update_ref_from_speed_sp(guidance_h_speed_sp);
  } else
#endif
    gh_update_ref_from_pos_sp(guidance_h_pos_sp);
#endif

  /* either use the reference or simply copy the pos setpoint */
  if (guidance_h_use_ref) {
    /* convert our reference to generic representation */
    INT32_VECT2_RSHIFT(guidance_h_pos_ref,   gh_ref.pos, (GH_POS_REF_FRAC - INT32_POS_FRAC));
    INT32_VECT2_LSHIFT(guidance_h_speed_ref, gh_ref.speed, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
    INT32_VECT2_LSHIFT(guidance_h_accel_ref, gh_ref.accel, (INT32_ACCEL_FRAC - GH_ACCEL_REF_FRAC));
  } else {
    VECT2_COPY(guidance_h_pos_ref, guidance_h_pos_sp);
    INT_VECT2_ZERO(guidance_h_speed_ref);
    INT_VECT2_ZERO(guidance_h_accel_ref);
  }

#if GUIDANCE_H_USE_SPEED_REF
  if (guidance_h_mode == GUIDANCE_H_MODE_HOVER) {
    VECT2_COPY(guidance_h_pos_sp, guidance_h_pos_ref); // for display only
  }
#endif
}


#define MAX_POS_ERR   POS_BFP_OF_REAL(16.)
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(16.)

#ifndef GUIDANCE_H_THRUST_CMD_FILTER
#define GUIDANCE_H_THRUST_CMD_FILTER 10
#endif

/* with a pgain of 100 and a scale of 2,
 * you get an angle of 5.6 degrees for 1m pos error */
#define GH_GAIN_SCALE 2

static void guidance_h_traj_run(bool_t in_flight)
{
  /* maximum bank angle: default 20 deg, max 40 deg*/
  static const int32_t traj_max_bank = Max(BFP_OF_REAL(GUIDANCE_H_MAX_BANK, INT32_ANGLE_FRAC),
                                       BFP_OF_REAL(RadOfDeg(40), INT32_ANGLE_FRAC));
  static const int32_t total_max_bank = BFP_OF_REAL(RadOfDeg(45), INT32_ANGLE_FRAC);

  /* compute position error    */
  VECT2_DIFF(guidance_h_pos_err, guidance_h_pos_ref, *stateGetPositionNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_h_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  /* compute speed error    */
  VECT2_DIFF(guidance_h_speed_err, guidance_h_speed_ref, *stateGetSpeedNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_h_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

  /* run PID */
  int32_t pd_x =
    ((guidance_h_pgain * guidance_h_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_dgain * (guidance_h_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  int32_t pd_y =
    ((guidance_h_pgain * guidance_h_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_dgain * (guidance_h_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  guidance_h_cmd_earth.x =
    pd_x +
    ((guidance_h_vgain * guidance_h_speed_ref.x) >> 17) + /* speed feedforward gain */
    ((guidance_h_again * guidance_h_accel_ref.x) >> 8);   /* acceleration feedforward gain */
  guidance_h_cmd_earth.y =
    pd_y +
    ((guidance_h_vgain * guidance_h_speed_ref.y) >> 17) + /* speed feedforward gain */
    ((guidance_h_again * guidance_h_accel_ref.y) >> 8);   /* acceleration feedforward gain */

  /* trim max bank angle from PD */
  VECT2_STRIM(guidance_h_cmd_earth, -traj_max_bank, traj_max_bank);

  /* Update pos & speed error integral, zero it if not in_flight.
   * Integrate twice as fast when not only POS but also SPEED are wrong,
   * but do not integrate POS errors when the SPEED is already catching up.
   */
  if (in_flight) {
    /* ANGLE_FRAC (12) * GAIN (8) * LOOP_FREQ (9) -> INTEGRATOR HIGH RES ANGLE_FRAX (28) */
    guidance_h_trim_att_integrator.x += (guidance_h_igain * pd_x);
    guidance_h_trim_att_integrator.y += (guidance_h_igain * pd_y);
    /* saturate it  */
    VECT2_STRIM(guidance_h_trim_att_integrator, -(traj_max_bank << 16), (traj_max_bank << 16));
    /* add it to the command */
    guidance_h_cmd_earth.x += (guidance_h_trim_att_integrator.x >> 16);
    guidance_h_cmd_earth.y += (guidance_h_trim_att_integrator.y >> 16);
  } else {
    INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  }

  /* compute a better approximation of force commands by taking thrust into account */
  if (guidance_h_approx_force_by_thrust && in_flight) {
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

static void guidance_h_hover_enter(void)
{

  /* set horizontal setpoint to current position */
  VECT2_COPY(guidance_h_pos_sp, *stateGetPositionNed_i());

  reset_guidance_reference_from_current_position();

  guidance_h_rc_sp.psi = stateGetNedToBodyEulers_i()->psi;
}

static void guidance_h_nav_enter(void)
{

  /* horizontal position setpoint from navigation/flightplan */
  INT32_VECT2_NED_OF_ENU(guidance_h_pos_sp, navigation_carrot);

  reset_guidance_reference_from_current_position();

  nav_heading = stateGetNedToBodyEulers_i()->psi;
}

static inline void transition_run(void)
{
  //Add 0.00625%
  transition_percentage += 1 << (INT32_PERCENTAGE_FRAC - 4);

#ifdef TRANSITION_MAX_OFFSET
  const int32_t max_offset = ANGLE_BFP_OF_REAL(TRANSITION_MAX_OFFSET);
  transition_theta_offset = INT_MULT_RSHIFT((transition_percentage << (INT32_ANGLE_FRAC - INT32_PERCENTAGE_FRAC)) / 100,
                            max_offset, INT32_ANGLE_FRAC);
#endif
}

/// read speed setpoint from RC
static void read_rc_setpoint_speed_i(struct Int32Vect2 *speed_sp, bool_t in_flight)
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
