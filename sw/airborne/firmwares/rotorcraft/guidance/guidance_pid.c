/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file firmwares/rotorcraft/guidance/guidance_pid.c
 *  Guidance controller with PID for rotorcrafts.
 *
 */

#include "firmwares/rotorcrafts/guidance/guidance_pid.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "generated/airframe.h"
#include "state.h"


// Keep GUIDANCE_H_XXX format for backward compatibility

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

#ifndef GUIDANCE_H_THRUST_CMD_FILTER
#define GUIDANCE_H_THRUST_CMD_FILTER 10
#endif

#ifndef GUIDANCE_H_APPROX_FORCE_BY_THRUST
#define GUIDANCE_H_APPROX_FORCE_BY_THRUST FALSE
#endif

#define MAX_POS_ERR   POS_BFP_OF_REAL(16.)
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(16.)

/** The PID controller is used by default
 */
#ifndef GUIDANCE_PID_USE_AS_DEFAULT
#define GUIDANCE_PID_USE_AS_DEFAULT TRUE
#endif

/*
 * external variables
 */

struct GuidancePIDGains guidance_pid_gains;
struct Int32Vect2 guidance_pid_cmd_earth;
bool guidance_pid_approx_force_by_thrust;

/*
 * internal variables
 */

struct Int32Vect2 guidance_pid_pos_err;
struct Int32Vect2 guidance_pid_speed_err;
struct Int32Vect2 guidance_pid_trim_att_integrator;


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

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
                           &guidance_pid_pos_err.x,
                           &guidance_pid_pos_err.y,
                           &guidance_pid_speed_err.x,
                           &guidance_pid_speed_err.y,
                           &guidance_pid_trim_att_integrator.x,
                           &guidance_pid_trim_att_integrator.y,
                           &guidance_pid_cmd_earth.x,
                           &guidance_pid_cmd_earth.y,
                           &guidance_h.sp.heading);
}

#endif

void guidance_pid_init(void)
{
  INT_VECT2_ZERO(guidance_pid_cmd_earth);
  INT_VECT2_ZERO(guidance_pid_pos_err);
  INT_VECT2_ZERO(guidance_pid_speed_err);
  INT_VECT2_ZERO(guidance_pid_trim_att_integrator);
  guidance_pid_gains.p = GUIDANCE_H_PGAIN;
  guidance_pid_gains.i = GUIDANCE_H_IGAIN;
  guidance_pid_gains.d = GUIDANCE_H_DGAIN;
  guidance_pid_gains.a = GUIDANCE_H_AGAIN;
  guidance_pid_gains.v = GUIDANCE_H_VGAIN;
  guidance_pid_approx_force_by_thrust = GUIDANCE_H_APPROX_FORCE_BY_THRUST;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_HOVER_LOOP, send_hover_loop);
#endif
}

/* with a pgain of 100 and a scale of 2,
 * you get an angle of 5.6 degrees for 1m pos error */
#define GH_GAIN_SCALE 2

/**
 * run control loop for position and speed control
 */
static struct Int32Vect2 guidance_pid_run_pid(bool in_flight, struct HorizontalGuidance *gh)
{
  /* maximum bank angle: default 20 deg, max 40 deg*/
  static const int32_t traj_max_bank = Min(BFP_OF_REAL(GUIDANCE_H_MAX_BANK, INT32_ANGLE_FRAC),
                                       BFP_OF_REAL(RadOfDeg(40), INT32_ANGLE_FRAC));
  static const int32_t total_max_bank = BFP_OF_REAL(RadOfDeg(45), INT32_ANGLE_FRAC);

  /* run PID */
  int32_t pd_x =
    ((guidance_pid_gains.p * guidance_pid_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_pid_gains.d * (guidance_pid_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  int32_t pd_y =
    ((guidance_pid_gains.p * guidance_pid_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_pid_gains.d * (guidance_pid_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  guidance_pid_cmd_earth.x = pd_x +
                           ((guidance_pid_gains.v * gh->ref.speed.x) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE)) + /* speed feedforward gain */
                           ((guidance_pid_gains.a * gh->ref.accel.x) >> (INT32_ACCEL_FRAC - GH_GAIN_SCALE));  /* acceleration feedforward gain */
  guidance_pid_cmd_earth.y = pd_y +
                           ((guidance_pid_gains.v * gh->ref.speed.y) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE)) + /* speed feedforward gain */
                           ((guidance_pid_gains.a * gh->ref.accel.y) >> (INT32_ACCEL_FRAC - GH_GAIN_SCALE));  /* acceleration feedforward gain */

  /* trim max bank angle from PD */
  VECT2_STRIM(guidance_pid_cmd_earth, -traj_max_bank, traj_max_bank);

  /* Update pos & speed error integral, zero it if not in_flight.
   * Integrate twice as fast when not only POS but also SPEED are wrong,
   * but do not integrate POS errors when the SPEED is already catching up.
   */
  if (in_flight) {
    /* ANGLE_FRAC (12) * GAIN (8) * LOOP_FREQ (9) -> INTEGRATOR HIGH RES ANGLE_FRAX (28) */
    guidance_pid_trim_att_integrator.x += (guidance_pid_gains.i * pd_x);
    guidance_pid_trim_att_integrator.y += (guidance_pid_gains.i * pd_y);
    /* saturate it  */
    VECT2_STRIM(guidance_pid_trim_att_integrator, -(traj_max_bank << (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2)),
                (traj_max_bank << (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2)));
    /* add it to the command */
    guidance_pid_cmd_earth.x += (guidance_pid_trim_att_integrator.x >> (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2));
    guidance_pid_cmd_earth.y += (guidance_pid_trim_att_integrator.y >> (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2));
  } else {
    INT_VECT2_ZERO(guidance_pid_trim_att_integrator);
  }

  /* compute a better approximation of force commands by taking thrust into account */
  if (guidance_pid_approx_force_by_thrust && in_flight) {
    static int32_t thrust_cmd_filt;
    int32_t vertical_thrust = (stabilization_cmd[COMMAND_THRUST] * guidance_v_thrust_coeff) >> INT32_TRIG_FRAC;
    thrust_cmd_filt = (thrust_cmd_filt * GUIDANCE_H_THRUST_CMD_FILTER + vertical_thrust) /
                      (GUIDANCE_H_THRUST_CMD_FILTER + 1);
    guidance_pid_cmd_earth.x = ANGLE_BFP_OF_REAL(atan2f((guidance_pid_cmd_earth.x * MAX_PPRZ / INT32_ANGLE_PI_2),
                             thrust_cmd_filt));
    guidance_pid_cmd_earth.y = ANGLE_BFP_OF_REAL(atan2f((guidance_pid_cmd_earth.y * MAX_PPRZ / INT32_ANGLE_PI_2),
                             thrust_cmd_filt));
  }

  VECT2_STRIM(guidance_pid_cmd_earth, -total_max_bank, total_max_bank);
  return guidance_pid_cmd_earth;
}

struct Int32Vect2 guidance_pid_run_pos(bool in_flight, struct HorizontalGuidance *gh)
{
  /* compute position error    */
  VECT2_DIFF(guidance_pid_pos_err, gh->ref.pos, *stateGetPositionNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_pid_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  /* compute speed error    */
  VECT2_DIFF(guidance_pid_speed_err, gh->ref.speed, *stateGetSpeedNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_pid_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

  /* run PID */
  return guidance_pid_run_pid(in_flight, gh);
}

struct Int32Vect2 guidance_pid_run_speed(bool in_flight, struct HorizontalGuidance *gh)
{
  /* cancel position error */
  INT_VECT2_ZERO(guidance_pid_pos_err);

  /* compute speed error    */
  VECT2_DIFF(guidance_pid_speed_err, gh->ref.speed, *stateGetSpeedNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_pid_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

  /* run PID */
  return guidance_pid_run_pid(in_flight, gh);
}

struct Int32Vect2 guidance_pid_run_accel(bool in_flight UNUSED, struct HorizontalGuidance *gh UNUSED)
{
  struct Int32Vect2 cmd = { 0, 0 };
  // TODO
  return cmd;
}

void guidance_pid_set_igain(uint32_t igain)
{
  guidance_pid_gains.i = igain;
  INT_VECT2_ZERO(guidance_pid_trim_att_integrator);
}


const struct Int32Vect2 *guidance_pid_get_pos_err(void)
{
  return &guidance_pid_pos_err;
}

#if GUIDANCE_PID_USE_AS_DEFAULT
// guidance pid control function is implementing the default functions of guidance

struct Int32Vect2 guidance_h_run_pos(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_pid_run_pos(in_flight, gh);
}

struct Int32Vect2 guidance_h_run_speed(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_pid_run_speed(in_flight, gh);
}

struct Int32Vect2 guidance_h_run_accel(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_pid_run_accel(in_flight, gh);
}

#endif

