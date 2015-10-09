/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2015 Ewoud Smeur
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

/**
 * @file firmwares/rotorcraft/guidance/guidance_flip.c
 *
 * Open Loop guidance for making a flip. You need to tune this before using.
 * When entering this mode it saves the previous guidance mode and changes AUTO2 back to
 * the previous mode after finishing the flip.
 * Use it with caution!
 */

#include "guidance_flip.h"

#include "autopilot.h"
#include "guidance_h.h"
#include "stabilization/stabilization_attitude_rc_setpoint.h"
#include "stabilization/stabilization_attitude.h"

#ifndef STOP_ROLL_CMD_ANGLE
#define STOP_ROLL_CMD_ANGLE 25.0
#endif
#ifndef FIRST_THRUST_DURATION
#define FIRST_THRUST_DURATION 0.3
#endif
#ifndef FINAL_THRUST_LEVEL
#define FINAL_THRUST_LEVEL 6000
#endif

uint32_t flip_counter;
uint8_t flip_state;
bool_t flip_rollout;
int32_t heading_save;
uint8_t autopilot_mode_old;
struct Int32Vect2 flip_cmd_earth;

void guidance_flip_enter(void)
{
  flip_counter = 0;
  flip_state = 0;
  flip_rollout = false;
  heading_save = stabilization_attitude_get_heading_i();
  autopilot_mode_old = autopilot_mode;
}

void guidance_flip_run(void)
{
  uint32_t timer;
  int32_t phi;
  static uint32_t timer_save = 0;

  timer = (flip_counter++ << 12) / PERIODIC_FREQUENCY;
  phi = stateGetNedToBodyEulers_i()->phi;

  switch (flip_state) {
    case 0:
      flip_cmd_earth.x = 0;
      flip_cmd_earth.y = 0;
      stabilization_attitude_set_earth_cmd_i(&flip_cmd_earth,
                                             heading_save);
      stabilization_attitude_run(autopilot_in_flight);
      stabilization_cmd[COMMAND_THRUST] = 8000; //Thrust to go up first
      timer_save = 0;

      if (timer > BFP_OF_REAL(FIRST_THRUST_DURATION, 12)) {
        flip_state++;
      }
      break;

    case 1:
      stabilization_cmd[COMMAND_ROLL]   = 9000; // Rolling command
      stabilization_cmd[COMMAND_PITCH]  = 0;
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = 1000; //Min thrust?

      if (phi > ANGLE_BFP_OF_REAL(RadOfDeg(STOP_ROLL_CMD_ANGLE))) {
        flip_state++;
      }
      break;

    case 2:
      stabilization_cmd[COMMAND_ROLL]   = 0;
      stabilization_cmd[COMMAND_PITCH]  = 0;
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = 1000; //Min thrust?

      if (phi > ANGLE_BFP_OF_REAL(RadOfDeg(-110.0)) && phi < ANGLE_BFP_OF_REAL(RadOfDeg(STOP_ROLL_CMD_ANGLE))) {
        timer_save = timer;
        flip_state++;
      }
      break;

    case 3:
      flip_cmd_earth.x = 0;
      flip_cmd_earth.y = 0;
      stabilization_attitude_set_earth_cmd_i(&flip_cmd_earth,
                                             heading_save);
      stabilization_attitude_run(autopilot_in_flight);

      stabilization_cmd[COMMAND_THRUST] = FINAL_THRUST_LEVEL; //Thrust to stop falling

      if ((timer - timer_save) > BFP_OF_REAL(0.5, 12)) {
        flip_state++;
      }
      break;

    default:
      autopilot_mode_auto2 = autopilot_mode_old;
      autopilot_set_mode(autopilot_mode_old);
      stab_att_sp_euler.psi = heading_save;
      flip_rollout = false;
      flip_counter = 0;
      timer_save = 0;
      flip_state = 0;

      stabilization_cmd[COMMAND_ROLL]   = 0;
      stabilization_cmd[COMMAND_PITCH]  = 0;
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = 8000; //Some thrust to come out of the roll?
      break;
  }
}
