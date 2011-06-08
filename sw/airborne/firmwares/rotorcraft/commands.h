/*  $Id$
 *
 * (c) 2009 Antoine Drouin <poinix@gmail.com>
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
 *
 */

#ifndef COMMANDS_H
#define COMMANDS_H

#include "paparazzi.h"
#include "generated/airframe.h"

extern int32_t commands[COMMANDS_NB];
extern const int32_t commands_failsafe[COMMANDS_NB];

#ifndef ROTORCRAFT_COMMANDS_YAW_ALWAYS_ENABLED
#define SetCommands(_in_cmd, _in_flight, _motors_on) {			\
    commands[COMMAND_PITCH]  = _in_cmd[COMMAND_PITCH];		\
    commands[COMMAND_ROLL]   = _in_cmd[COMMAND_ROLL];		\
    commands[COMMAND_YAW]    = (_in_flight) ? _in_cmd[COMMAND_YAW] : 0; \
    commands[COMMAND_THRUST] = (_motors_on) ? _in_cmd[COMMAND_THRUST] : 0; \
  }
#else
#define SetCommands(_in_cmd, _in_flight, _motors_on) {			\
    commands[COMMAND_PITCH]  = _in_cmd[COMMAND_PITCH];		\
    commands[COMMAND_ROLL]   = _in_cmd[COMMAND_ROLL];		\
    commands[COMMAND_YAW]    = _in_cmd[COMMAND_YAW]; \
    commands[COMMAND_THRUST] = (_motors_on) ? _in_cmd[COMMAND_THRUST] : 0; \
  }
#endif

#endif /* COMMANDS_H */
