/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file firmwares/rover/guidance/rover_guidance.c
 *  Basic guidance for rover.
 *  Implement standard PID control loop to track a navigation target.
 *  Guidance "modes" are using the autopilot generation with the "guidance"
 *  state machine.
 */

#define AUTOPILOT_CORE_GUIDANCE_C

#include "firmwares/rover/guidance/rover_guidance.h"
#include "generated/airframe.h"
#include "generated/autopilot_core_guidance.h"
#include "state.h"

struct RoverGuidance rover_guidance;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
// TODO rover guidance messages
#endif

void rover_guidance_init(void)
{
  FLOAT_VECT2_ZERO(rover_guidance.sp.pos);
  rover_guidance.sp.heading = 0.0f;
  rover_guidance.speed_pid.p = ROVER_GUIDANCE_SPEED_PGAIN;
  rover_guidance.speed_pid.i = ROVER_GUIDANCE_SPEED_IGAIN;
  rover_guidance.speed_pid.d = ROVER_GUIDANCE_SPEED_DGAIN;
  rover_guidance.speed_pid.err = 0.f;
  rover_guidance.speed_pid.d_err = 0.f;
  rover_guidance.speed_pid.sum_err = 0.f;
  rover_guidance.turn_pid.p = ROVER_GUIDANCE_TURN_PGAIN;
  rover_guidance.turn_pid.i = ROVER_GUIDANCE_TURN_IGAIN;
  rover_guidance.turn_pid.d = ROVER_GUIDANCE_TURN_DGAIN;
  rover_guidance.turn_pid.err = 0.f;
  rover_guidance.turn_pid.d_err = 0.f;
  rover_guidance.turn_pid.sum_err = 0.f;

#if PERIODIC_TELEMETRY
  // TODO register messages
#endif

  // from code generation
  autopilot_core_guidance_init();
}

void rover_guidance_periodic(void)
{
  // from code generation
  autopilot_core_guidance_periodic_task();
}

static float compute_pid(struct RoverGuidancePID *pid)
{
  return pid->p * pid->err + pid->d * pid->d_err + pid->i * pid->sum_err;
}

#define MAX_POS_ERR       10.f // max position error
#define MAX_SPEED_ERR     10.f // max speed error
#define MAX_INTEGRAL_CMD  (MAX_PPRZ / 10.f) // 10% of max command
#define PROXIMITY_DIST    0.5f // proximity distance TODO improve

void rover_guidance_run(float *heading_sp)
{
  // compute position error
  struct FloatVect2 pos_err;
  VECT2_DIFF(pos_err, rover_guidance.sp.pos, *stateGetPositionEnu_f());
  rover_guidance.speed_pid.err = float_vect2_norm(&pos_err);
  BoundAbs(rover_guidance.speed_pid.err, MAX_POS_ERR);

  // speed and heading update when far enough
  if (rover_guidance.speed_pid.err > PROXIMITY_DIST) {
    // compute speed error
    rover_guidance.speed_pid.d_err = - stateGetHorizontalSpeedNorm_f();
    BoundAbs(rover_guidance.speed_pid.d_err, MAX_SPEED_ERR);
    // integral
    rover_guidance.speed_pid.sum_err = 0.f; // nothing for now
    // run PID
    rover_guidance.cmd.motor_speed = MAX_PPRZ * compute_pid(&rover_guidance.speed_pid);
    rover_guidance.cmd.motor_speed = TRIM_PPRZ(rover_guidance.cmd.motor_speed);

    // if not close to WP, compute desired heading
    rover_guidance.sp.heading = atan2f(pos_err.x, pos_err.y);
    *heading_sp = rover_guidance.sp.heading; // update nav sp
  }
  else {
    rover_guidance.cmd.motor_speed = 0.f;
    // use nav heading ref and run angular control
    rover_guidance.sp.heading = *heading_sp;
  }

  // angular error
  rover_guidance.turn_pid.err = rover_guidance.sp.heading - stateGetNedToBodyEulers_f()->psi;
  NormRadAngle(rover_guidance.turn_pid.err);
  // turn rate error
  rover_guidance.turn_pid.d_err = - stateGetBodyRates_f()->r;
  // integral
  rover_guidance.turn_pid.sum_err = 0.f; // nothing for now
  // run PID
  rover_guidance.cmd.motor_turn = MAX_PPRZ * compute_pid(&rover_guidance.turn_pid);
  rover_guidance.cmd.motor_turn = TRIM_PPRZ(rover_guidance.cmd.motor_turn);
}

void rover_guidance_enter(void)
{
  ClearBit(rover_guidance.sp.mask, 5);
  ClearBit(rover_guidance.sp.mask, 7);

  /* horizontal position setpoint from navigation/flightplan */
  //INT32_VECT2_NED_OF_ENU(rover_guidance.sp.pos, navigation_carrot); TODO move to AP xml

  //nav_heading = stateGetNedToBodyEulers_i()->psi; TODO move to AP xml
  rover_guidance.sp.heading = stateGetNedToBodyEulers_f()->psi;

  // TODO reset integral part ?
}

// TODO from AP xml
//void rover_guidance_from_nav(bool in_flight)
//{
//  if (!in_flight) {
//    rover_guidance_nav_enter();
//  }
//
//  if (horizontal_mode == HORIZONTAL_MODE_MANUAL) {
//    stabilization_cmd[COMMAND_ROLL]  = nav_cmd_roll;
//    stabilization_cmd[COMMAND_PITCH] = nav_cmd_pitch;
//    stabilization_cmd[COMMAND_YAW]   = nav_cmd_yaw;
//  } else if (horizontal_mode == HORIZONTAL_MODE_ATTITUDE) {
//    struct Int32Eulers sp_cmd_i;
//    sp_cmd_i.phi = nav_roll;
//    sp_cmd_i.theta = nav_pitch;
//    sp_cmd_i.psi = nav_heading;
//    stabilization_attitude_set_rpy_setpoint_i(&sp_cmd_i);
//    stabilization_attitude_run(in_flight);
//  } else {
//    INT32_VECT2_NED_OF_ENU(rover_guidance.sp.pos, navigation_carrot);
//
//    /* set psi command */
//    rover_guidance.sp.heading = ANGLE_FLOAT_OF_BFP(nav_heading);
//    FLOAT_ANGLE_NORMALIZE(rover_guidance.sp.heading);
//
//    /* compute x,y earth commands */
//    rover_guidance_traj_run(in_flight);
//    /* set final attitude setpoint */
//    int32_t heading_sp_i = ANGLE_BFP_OF_REAL(rover_guidance.sp.heading);
//    stabilization_attitude_set_earth_cmd_i(&rover_guidance_cmd_earth,
//                                           heading_sp_i);
//    stabilization_attitude_run(in_flight);
//  }
//}

void rover_guidance_set_speed_igain(uint32_t igain)
{
  rover_guidance.speed_pid.i = igain;
  rover_guidance.speed_pid.sum_err = 0.f;
}

void rover_guidance_set_turn_igain(uint32_t igain)
{
  rover_guidance.turn_pid.i = igain;
  rover_guidance.turn_pid.sum_err = 0.f;
}


#ifdef ROVER_GUIDANCE_MODE_GUIDED
void rover_guidance_guided_run(bool in_flight)
{
  /* rover_guidance.sp.pos and rover_guidance.sp.heading need to be set from external source */
  if (!in_flight) {
    rover_guidance_hover_enter();
  }

  /* compute x,y earth commands */
  rover_guidance_traj_run(in_flight);
}

bool rover_guidance_set_guided_pos(float x, float y)
{
  if (rover_guidance.mode == ROVER_GUIDANCE_MODE_GUIDED) {
    ClearBit(rover_guidance.sp.mask, 5);
    rover_guidance.sp.pos.x = x;
    rover_guidance.sp.pos.y = y;
    return true;
  }
  return false;
}

bool rover_guidance_set_guided_heading(float heading)
{
  if (rover_guidance.mode == ROVER_GUIDANCE_MODE_GUIDED) {
    ClearBit(rover_guidance.sp.mask, 7);
    rover_guidance.sp.heading = heading;
    FLOAT_ANGLE_NORMALIZE(rover_guidance.sp.heading);
    return true;
  }
  return false;
}

bool rover_guidance_set_guided_body_vel(float vx, float vy)
{
  float psi = stateGetNedToBodyEulers_f()->psi;
  float newvx =  cosf(-psi) * vx + sinf(-psi) * vy;
  float newvy = -sinf(-psi) * vx + cosf(-psi) * vy;
  return rover_guidance_set_guided_vel(newvx, newvy);
}

bool rover_guidance_set_guided_vel(float vx, float vy)
{
  if (rover_guidance.mode == ROVER_GUIDANCE_MODE_GUIDED) {
    SetBit(rover_guidance.sp.mask, 5);
    rover_guidance.sp.speed.x = vx;
    rover_guidance.sp.speed.y = vy;
    return true;
  }
  return false;
}

bool rover_guidance_set_guided_heading_rate(float rate)
{
  if (rover_guidance.mode == ROVER_GUIDANCE_MODE_GUIDED) {
    SetBit(rover_guidance.sp.mask, 7);
    rover_guidance.sp.heading_rate = rate;
    return true;
  }
  return false;
}

#endif

