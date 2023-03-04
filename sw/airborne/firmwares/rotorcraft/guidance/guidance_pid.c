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

#include "firmwares/rotorcraft/guidance/guidance_pid.h"
#include "firmwares/rotorcraft/guidance/guidance_v_adapt.h"
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
#error "ALL PID_H control gains have to be positive!!!"
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

// TODO configurable
#define MAX_POS_ERR   POS_BFP_OF_REAL(16.)
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(16.)

// Keep GUIDANCE_V_XXX format for backward compatibility

/* error if some gains are negative */
#if (GUIDANCE_V_HOVER_KP < 0) ||                   \
  (GUIDANCE_V_HOVER_KD < 0)   ||                   \
  (GUIDANCE_V_HOVER_KI < 0)
#error "ALL PID_V control gains must be positive!!!"
#endif

/* If only GUIDANCE_V_NOMINAL_HOVER_THROTTLE is defined,
 * disable the adaptive throttle estimation by default.
 * Otherwise enable adaptive estimation by default.
 */
#ifdef GUIDANCE_V_NOMINAL_HOVER_THROTTLE
#  ifndef GUIDANCE_V_ADAPT_THROTTLE_ENABLED
#    define GUIDANCE_V_ADAPT_THROTTLE_ENABLED FALSE
#  endif
#else
#  ifndef GUIDANCE_V_ADAPT_THROTTLE_ENABLED
#    define GUIDANCE_V_ADAPT_THROTTLE_ENABLED TRUE
#  endif
#endif
PRINT_CONFIG_VAR(GUIDANCE_V_ADAPT_THROTTLE_ENABLED)

#ifndef GUIDANCE_V_MIN_ERR_Z
#define GUIDANCE_V_MIN_ERR_Z POS_BFP_OF_REAL(-10.)
#endif

#ifndef GUIDANCE_V_MAX_ERR_Z
#define GUIDANCE_V_MAX_ERR_Z POS_BFP_OF_REAL(10.)
#endif

#ifndef GUIDANCE_V_MIN_ERR_ZD
#define GUIDANCE_V_MIN_ERR_ZD SPEED_BFP_OF_REAL(-10.)
#endif

#ifndef GUIDANCE_V_MAX_ERR_ZD
#define GUIDANCE_V_MAX_ERR_ZD SPEED_BFP_OF_REAL(10.)
#endif

#ifndef GUIDANCE_V_MAX_SUM_ERR
#define GUIDANCE_V_MAX_SUM_ERR 2000000
#endif

#ifndef GUIDANCE_V_MAX_CMD
#define GUIDANCE_V_MAX_CMD 0.9*MAX_PPRZ
#endif



/** The PID controller is used by default
 */
#ifndef GUIDANCE_PID_USE_AS_DEFAULT
#define GUIDANCE_PID_USE_AS_DEFAULT TRUE
#endif

/*
 * external variables
 */

struct GuidancePID guidance_pid;

/*
 * internal variables
 */

struct Int32Vect2 guidance_pid_pos_err;
struct Int32Vect2 guidance_pid_speed_err;
struct Int32Vect2 guidance_pid_trim_att_integrator;

int32_t guidance_pid_z_sum_err; ///< accumulator for I-gain
int32_t guidance_pid_v_ff_cmd;  ///< feed-forward command
int32_t guidance_pid_v_fb_cmd;  ///< feed-back command


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
                           &guidance_pid.cmd_earth.x,
                           &guidance_pid.cmd_earth.y,
                           &guidance_h.sp.heading);
}

static void send_vert_loop(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_VERT_LOOP(trans, dev, AC_ID,
                          &guidance_v.z_sp, &guidance_v.zd_sp,
                          &(stateGetPositionNed_i()->z),
                          &(stateGetSpeedNed_i()->z),
                          &(stateGetAccelNed_i()->z),
                          &guidance_v.z_ref, &guidance_v.zd_ref,
                          &guidance_v.zdd_ref,
                          &gv_adapt_X,
                          &gv_adapt_P,
                          &gv_adapt_Xmeas,
                          &guidance_pid_z_sum_err,
                          &guidance_pid_v_ff_cmd,
                          &guidance_pid_v_fb_cmd,
                          &guidance_pid.cmd_thrust);
}

#endif

void guidance_pid_init(void)
{
  INT_VECT2_ZERO(guidance_pid.cmd_earth);
  INT_VECT2_ZERO(guidance_pid_pos_err);
  INT_VECT2_ZERO(guidance_pid_speed_err);
  INT_VECT2_ZERO(guidance_pid_trim_att_integrator);
  guidance_pid.cmd_thrust = 0;
  guidance_pid_z_sum_err = 0;
  guidance_pid_v_ff_cmd = 0;
  guidance_pid_v_fb_cmd = 0;
  guidance_pid.kp = GUIDANCE_H_PGAIN;
  guidance_pid.ki = GUIDANCE_H_IGAIN;
  guidance_pid.kd = GUIDANCE_H_DGAIN;
  guidance_pid.ka = GUIDANCE_H_AGAIN;
  guidance_pid.kv = GUIDANCE_H_VGAIN;
  guidance_pid.v_kp = GUIDANCE_V_HOVER_KP;
  guidance_pid.v_kd = GUIDANCE_V_HOVER_KD;
  guidance_pid.v_ki = GUIDANCE_V_HOVER_KI;
  guidance_pid.approx_force_by_thrust = GUIDANCE_H_APPROX_FORCE_BY_THRUST;
  guidance_pid.adapt_throttle_enabled = GUIDANCE_V_ADAPT_THROTTLE_ENABLED;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_HOVER_LOOP, send_hover_loop);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VERT_LOOP, send_vert_loop);
#endif
}

/* with a pgain of 100 and a scale of 2,
 * you get an angle of 5.6 degrees for 1m pos error */
#define GH_GAIN_SCALE 2

/**
 * run horizontal control loop for position and speed control
 */
static struct StabilizationSetpoint guidance_pid_h_run(bool in_flight, struct HorizontalGuidance *gh)
{
  /* maximum bank angle: default 20 deg, max 40 deg*/
  static const int32_t traj_max_bank = Min(BFP_OF_REAL(GUIDANCE_H_MAX_BANK, INT32_ANGLE_FRAC),
                                       BFP_OF_REAL(RadOfDeg(40), INT32_ANGLE_FRAC));
  static const int32_t total_max_bank = BFP_OF_REAL(RadOfDeg(45), INT32_ANGLE_FRAC);

  /* run PID */
  int32_t pd_x =
    ((guidance_pid.kp * guidance_pid_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_pid.kd * (guidance_pid_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  int32_t pd_y =
    ((guidance_pid.kp * guidance_pid_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_pid.kd * (guidance_pid_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  guidance_pid.cmd_earth.x = pd_x +
                           ((guidance_pid.kv * gh->ref.speed.x) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE)) + /* speed feedforward gain */
                           ((guidance_pid.ka * gh->ref.accel.x) >> (INT32_ACCEL_FRAC - GH_GAIN_SCALE));  /* acceleration feedforward gain */
  guidance_pid.cmd_earth.y = pd_y +
                           ((guidance_pid.kv * gh->ref.speed.y) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE)) + /* speed feedforward gain */
                           ((guidance_pid.ka * gh->ref.accel.y) >> (INT32_ACCEL_FRAC - GH_GAIN_SCALE));  /* acceleration feedforward gain */

  /* trim max bank angle from PD */
  VECT2_STRIM(guidance_pid.cmd_earth, -traj_max_bank, traj_max_bank);

  /* Update pos & speed error integral, zero it if not in_flight.
   * Integrate twice as fast when not only POS but also SPEED are wrong,
   * but do not integrate POS errors when the SPEED is already catching up.
   */
  if (in_flight) {
    /* ANGLE_FRAC (12) * GAIN (8) * LOOP_FREQ (9) -> INTEGRATOR HIGH RES ANGLE_FRAX (28) */
    guidance_pid_trim_att_integrator.x += (guidance_pid.ki * pd_x);
    guidance_pid_trim_att_integrator.y += (guidance_pid.ki * pd_y);
    /* saturate it  */
    VECT2_STRIM(guidance_pid_trim_att_integrator, -(traj_max_bank << (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2)),
                (traj_max_bank << (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2)));
    /* add it to the command */
    guidance_pid.cmd_earth.x += (guidance_pid_trim_att_integrator.x >> (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2));
    guidance_pid.cmd_earth.y += (guidance_pid_trim_att_integrator.y >> (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2));
  } else {
    INT_VECT2_ZERO(guidance_pid_trim_att_integrator);
  }

  /* compute a better approximation of force commands by taking thrust into account */
  if (guidance_pid.approx_force_by_thrust && in_flight) {
    static int32_t thrust_cmd_filt;
    // FIXME strong coupling with guidance_v here !!!
    int32_t vertical_thrust = (stabilization_cmd[COMMAND_THRUST] * guidance_v.thrust_coeff) >> INT32_TRIG_FRAC;
    thrust_cmd_filt = (thrust_cmd_filt * GUIDANCE_H_THRUST_CMD_FILTER + vertical_thrust) /
                      (GUIDANCE_H_THRUST_CMD_FILTER + 1);
    guidance_pid.cmd_earth.x = ANGLE_BFP_OF_REAL(atan2f((guidance_pid.cmd_earth.x * MAX_PPRZ / INT32_ANGLE_PI_2),
                             thrust_cmd_filt));
    guidance_pid.cmd_earth.y = ANGLE_BFP_OF_REAL(atan2f((guidance_pid.cmd_earth.y * MAX_PPRZ / INT32_ANGLE_PI_2),
                             thrust_cmd_filt));
  }

  VECT2_STRIM(guidance_pid.cmd_earth, -total_max_bank, total_max_bank);

  return stab_sp_from_ltp_i(&guidance_pid.cmd_earth, ANGLE_BFP_OF_REAL(guidance_h.sp.heading));
}

struct StabilizationSetpoint guidance_pid_h_run_pos(bool in_flight, struct HorizontalGuidance *gh)
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
  return guidance_pid_h_run(in_flight, gh);
}

struct StabilizationSetpoint guidance_pid_h_run_speed(bool in_flight, struct HorizontalGuidance *gh)
{
  /* cancel position error */
  INT_VECT2_ZERO(guidance_pid_pos_err);

  /* compute speed error    */
  VECT2_DIFF(guidance_pid_speed_err, gh->ref.speed, *stateGetSpeedNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_pid_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

  /* run PID */
  return guidance_pid_h_run(in_flight, gh);
}

struct StabilizationSetpoint guidance_pid_h_run_accel(bool in_flight UNUSED, struct HorizontalGuidance *gh UNUSED)
{
  struct StabilizationSetpoint sp = { 0 };
  // TODO
  return sp;
}

/**
 * run vertical control loop for position and speed control
 */

#define FF_CMD_FRAC 18

static int32_t guidance_pid_v_run(bool in_flight, struct VerticalGuidance *gv)
{
  /* compute the error to our reference */
  int32_t err_z  = gv->z_ref - stateGetPositionNed_i()->z;
  Bound(err_z, GUIDANCE_V_MIN_ERR_Z, GUIDANCE_V_MAX_ERR_Z);
  int32_t err_zd = gv->zd_ref - stateGetSpeedNed_i()->z;
  Bound(err_zd, GUIDANCE_V_MIN_ERR_ZD, GUIDANCE_V_MAX_ERR_ZD);

  if (in_flight) {
    guidance_pid_z_sum_err += err_z;
    Bound(guidance_pid_z_sum_err, -GUIDANCE_V_MAX_SUM_ERR, GUIDANCE_V_MAX_SUM_ERR);
  } else {
    guidance_pid_z_sum_err = 0;
  }

  /* our nominal command : (g + zdd)*m   */
  int32_t inv_m;
  if (guidance_pid.adapt_throttle_enabled) {
    inv_m = gv_adapt_X >> (GV_ADAPT_X_FRAC - FF_CMD_FRAC);
  } else {
    /* use the fixed nominal throttle */
    inv_m = BFP_OF_REAL(9.81 / (gv->nominal_throttle * MAX_PPRZ), FF_CMD_FRAC);
  }

  const int32_t g_m_zdd = (int32_t)BFP_OF_REAL(9.81, FF_CMD_FRAC) -
                          (gv->zdd_ref << (FF_CMD_FRAC - INT32_ACCEL_FRAC));

  guidance_pid_v_ff_cmd = g_m_zdd / inv_m;
  /* feed forward command */
  guidance_pid_v_ff_cmd = (guidance_pid_v_ff_cmd << INT32_TRIG_FRAC) / gv->thrust_coeff;

  /* bound the nominal command to GUIDANCE_V_MAX_CMD */
  Bound(guidance_pid_v_ff_cmd, 0, GUIDANCE_V_MAX_CMD);


  /* our error feed back command                   */
  /* z-axis pointing down -> positive error means we need less thrust */
  guidance_pid_v_fb_cmd =
    ((-guidance_pid.v_kp * err_z)  >> 7) +
    ((-guidance_pid.v_kd * err_zd) >> 16) +
    ((-guidance_pid.v_ki * guidance_pid_z_sum_err) >> 16);

  guidance_pid.cmd_thrust = guidance_pid_v_ff_cmd + guidance_pid_v_fb_cmd;

  /* bound the result */
  Bound(guidance_pid.cmd_thrust, 0, MAX_PPRZ);

  return guidance_pid.cmd_thrust;
}

int32_t guidance_pid_v_run_pos(bool in_flight, struct VerticalGuidance *gv)
{
  return guidance_pid_v_run(in_flight, gv);
}

int32_t guidance_pid_v_run_speed(bool in_flight, struct VerticalGuidance *gv)
{
  return guidance_pid_v_run(in_flight, gv);
}

int32_t guidance_pid_v_run_accel(bool in_flight UNUSED, struct VerticalGuidance *gv UNUSED)
{
  // TODO
  return 0;
}

void guidance_pid_h_enter(void)
{
}

void guidance_pid_v_enter(void)
{
  guidance_pid_z_sum_err = 0;
}

/**
 * settings handler
 */

void guidance_pid_set_h_igain(uint32_t igain)
{
  guidance_pid.ki = igain;
  INT_VECT2_ZERO(guidance_pid_trim_att_integrator);
}

void guidance_pid_set_v_igain(uint32_t igain)
{
  guidance_pid.v_ki = igain;
  guidance_pid_z_sum_err = 0;
}


const struct Int32Vect2 *guidance_pid_get_h_pos_err(void)
{
  return &guidance_pid_pos_err;
}

#if GUIDANCE_PID_USE_AS_DEFAULT
// guidance pid control function is implementing the default functions of guidance

void guidance_h_run_enter(void)
{
  guidance_pid_h_enter();
}

void guidance_v_run_enter(void)
{
  guidance_pid_v_enter();
}

struct StabilizationSetpoint guidance_h_run_pos(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_pid_h_run_pos(in_flight, gh);
}

struct StabilizationSetpoint guidance_h_run_speed(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_pid_h_run_speed(in_flight, gh);
}

struct StabilizationSetpoint guidance_h_run_accel(bool in_flight, struct HorizontalGuidance *gh)
{
  return guidance_pid_h_run_accel(in_flight, gh);
}

int32_t guidance_v_run_pos(bool in_flight, struct VerticalGuidance *gv)
{
  return guidance_pid_v_run_pos(in_flight, gv);
}

int32_t guidance_v_run_speed(bool in_flight, struct VerticalGuidance *gv)
{
  return guidance_pid_v_run_speed(in_flight, gv);
}

int32_t guidance_v_run_accel(bool in_flight, struct VerticalGuidance *gv)
{
  return guidance_pid_v_run_accel(in_flight, gv);
}

#endif

