/*
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file firmwares/rotorcraft/autopilot_utils.h
 *
 * Utility functions and includes for autopilots
 *
 */

#ifndef AUTOPILOT_UTILS_H
#define AUTOPILOT_UTILS_H

#include "std.h"
#include "modules/core/commands.h"

/** Set descent speed in failsafe mode */
#ifndef FAILSAFE_DESCENT_SPEED
#define FAILSAFE_DESCENT_SPEED 1.5
#endif

extern bool ap_ahrs_is_aligned(void);
#if defined RADIO_MODE_2x3
extern uint8_t ap_mode_of_3x2way_switch(void);
#else
extern uint8_t ap_mode_of_3way_switch(void);
#endif
#if defined RADIO_AUTO_MODE || defined(__DOXYGEN__)
extern uint8_t ap_mode_of_two_switches(void)
#endif

/** Set Rotorcraft commands.
 *  Limit thrust and/or yaw depending of the in_flight
 *  and motors_on flag status
 *
 *  A default implementation is provided, but the function can be redefined
 *
 *  @param[out] cmd_out output command vector in pprz_t (usually commands array)
 *  @param[in/out] cmd_in input commands to apply, might be affected by in_flight and motors_on param (FIXME really ?)
 *  @param[in] in_flight tells if rotorcraft is in flight
 *  @param[in] motors_on tells if motors are running
 */
extern void set_rotorcraft_commands(pprz_t *cmd_out, int32_t *cmd_in, bool in_flight, bool motors_on);

// in case, backward compatibility macro
#define SetRotorcraftCommands(_cmd, _in_flight, _motors_on) set_rotorcraft_commands(commands, _cmd, _in_flight, _motors_on)

#endif // AUTOPILOT_UTILS_H

