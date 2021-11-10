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

/**
 * @file firmwares/rover/autopilot_utils.h
 *
 * Utility functions and includes for autopilots
 *
 */

#ifndef AUTOPILOT_UTILS_H
#define AUTOPILOT_UTILS_H

#include "std.h"
#include "modules/core/commands.h"

extern bool ap_ahrs_is_aligned(void);
extern uint8_t ap_mode_of_3way_switch(void);

extern void set_rotorcraft_commands(pprz_t *cmd_out, int32_t *cmd_in, bool in_flight, bool motors_on);

// in case, backward compatibility macro
#define SetRotorcraftCommands(_cmd, _in_flight, _motors_on) set_rotorcraft_commands(commands, _cmd, _in_flight, _motors_on)

#endif // AUTOPILOT_UTILS_H

