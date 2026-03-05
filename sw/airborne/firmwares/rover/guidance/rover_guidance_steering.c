/*
 * Copyright (C) 2021 Jesús Bautista <jesusbautistavillar@gmail.com>
 *                    Hector García  <noeth3r@gmail.com>
 *               2025 Gautier Hattenberger <gautier.hattenberger@gmail.com>
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

#define AUTOPILOT_CORE_GUIDANCE_C

/** Mandatory dependencies header **/
#include "firmwares/rover/guidance/rover_guidance_steering.h"

#include "generated/airframe.h"
#include "generated/autopilot_core_guidance.h"

#include "autopilot.h"
#include "state.h"

#include "filters/pid.h" // Used for p+i speed controller

#ifndef ROVER_GUIDANCE_POS_KP
#define ROVER_GUIDANCE_POS_KP 1.f
#endif

#ifndef ROVER_GUIDANCE_HEADING_KP
#define ROVER_GUIDANCE_HEADING_KP 1.f
#endif

#ifndef ROVER_GUIDANCE_SPEED_KP
#define ROVER_GUIDANCE_SPEED_KP 10.f
#endif

#ifndef ROVER_GUIDANCE_SPEED_KI
#define ROVER_GUIDANCE_SPEED_KI 100.f
#endif

#ifndef ROVER_GUIDANCE_MAX_POS_ERR
#define ROVER_GUIDANCE_MAX_POS_ERR 10.f // max position error
#endif

#ifndef ROVER_GUIDANCE_MAX_SPEED
#define ROVER_GUIDANCE_MAX_SPEED 10.f
#endif

#ifndef ROVER_GUIDANCE_PROXIMITY_DISTANCE
#define ROVER_GUIDANCE_PROXIMITY_DISTANCE 2.f // proximity distance TODO improve with hysteresis ?
#endif

// Guidance control main variables
rover_ctrl guidance_control;

static struct PID_f rover_pid;
static float time_step;

/** INIT function **/
void rover_guidance_steering_init(void)
{
  guidance_control.cmd.delta = 0.f;
  guidance_control.cmd.speed = 0.f;
  guidance_control.throttle  = 0.f;

  guidance_control.pos_kp = ROVER_GUIDANCE_POS_KP;
  guidance_control.heading_kp = ROVER_GUIDANCE_HEADING_KP;
  guidance_control.speed_error = 0.f;
  guidance_control.kf = SR_MEASURED_KF;
  guidance_control.kp = ROVER_GUIDANCE_SPEED_KP;
  guidance_control.ki = ROVER_GUIDANCE_SPEED_KI;

  init_pid_f(&rover_pid, guidance_control.kp, 0.f, guidance_control.ki, MAX_PPRZ*0.2f);

  // Based on autopilot state machine frequency
  time_step = 1.f/PERIODIC_FREQUENCY;
}

void rover_guidance_steering_periodic(void)
{
  // from code generation
  autopilot_core_guidance_periodic_task();
}

/** CTRL functions **/
// Steering control (GVF)
void rover_guidance_steering_heading_ctrl(float omega) //GVF give us this omega
{
  // Speed is bounded to avoid GPS noise while driving at small velocity
  float speed = BoundSpeed(stateGetHorizontalSpeedNorm_f());
  // Compute steering angle
  float delta = DegOfRad(atanf(omega*DRIVE_SHAFT_DISTANCE/speed));

  guidance_control.cmd.delta = BoundDelta(delta);
}

// Speed control (feed feed forward + propotional + integral controler) (PID)
void rover_guidance_steering_speed_ctrl(void)
{
  // Updating PID
  guidance_control.speed_error = (guidance_control.cmd.speed - stateGetHorizontalSpeedNorm_f());
  update_pid_f(&rover_pid, guidance_control.speed_error, time_step);

  guidance_control.throttle = BoundThrottle(guidance_control.kf*guidance_control.cmd.speed + get_pid_f(&rover_pid));
}

void rover_guidance_steering_setpoints(struct EnuCoor_f pos_sp, float *heading_sp)
{
  // compute position error
  struct FloatVect2 pos_2d;
  VECT2_COPY(pos_2d, pos_sp);
  struct FloatVect2 pos_err;
  VECT2_DIFF(pos_err, pos_2d, *stateGetPositionEnu_f());
  float pos_err_norm = float_vect2_norm(&pos_err);
  BoundAbs(pos_err_norm, ROVER_GUIDANCE_MAX_POS_ERR);

  // speed and heading update when far enough
  if (pos_err_norm > ROVER_GUIDANCE_PROXIMITY_DISTANCE) {
    // compute speed error
    guidance_control.cmd.speed = guidance_control.pos_kp * pos_err_norm;
    Bound(guidance_control.cmd.speed, 0.f, ROVER_GUIDANCE_MAX_SPEED);
    // if not close to WP, compute desired heading
    guidance_control.heading_sp = atan2f(pos_err.x, pos_err.y);
    // update nav sp
    *heading_sp = guidance_control.heading_sp;
    // angular error
    float heading_err = guidance_control.heading_sp - stateGetNedToBodyEulers_f()->psi;
    NormRadAngle(heading_err);
    // compute omega setpoint
    guidance_control.omega_sp = guidance_control.heading_kp * heading_err;
  }
  else {
    guidance_control.cmd.speed = 0.f;
    guidance_control.heading_sp = *heading_sp;
    guidance_control.omega_sp = 0.f;
  }
}

/** PID RESET function**/
void rover_guidance_steering_pid_reset(void)
{
  reset_pid_f(&rover_pid);
}

void rover_guidance_steering_kill(void)
{
  guidance_control.cmd.delta = 0.0;
  guidance_control.cmd.speed = 0.0;
}

void rover_guidance_steering_set_speed_pgain(float pgain)
{
  guidance_control.kp = pgain;
  set_gains_pid_f(&rover_pid, guidance_control.kp, 0.f, guidance_control.ki);
}

void rover_guidance_steering_set_speed_igain(float igain)
{
  guidance_control.ki = igain;
  set_gains_pid_f(&rover_pid, guidance_control.kp, 0.f, guidance_control.ki);
  rover_pid.sum = 0.f;
}

