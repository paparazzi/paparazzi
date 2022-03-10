/*
 * Copyright (c) 2006 Pascal Brisset, Antoine Drouin
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 *
 */

/** \file modules/core/commands.h
 *  \brief Hardware independent code for commands handling
 *
 */

#ifndef COMMANDS_H
#define COMMANDS_H

#include "paparazzi.h"
#include "generated/airframe.h"

/** Storage of intermediate command values.
 *  These values come from the RC (MANUAL mode), from the autopilot (AUTO mode) or from control loops.
 *  They are asyncronisly used to set the servos
 */
extern pprz_t commands[COMMANDS_NB];
extern const pprz_t commands_failsafe[COMMANDS_NB];

/** Settings to trim roll, pitch and yaw commands if defined
 */
extern pprz_t command_roll_trim;
extern pprz_t command_pitch_trim;
extern pprz_t command_yaw_trim;

// Set all commands from array
#define SetCommands(t) { \
    int i; \
    for(i = 0; i < COMMANDS_NB; i++) commands[i] = t[i]; \
  }

/** Set a command value
 * @param idx command index
 * @param value new value
 */
static inline void command_set(uint8_t idx, pprz_t value)
{
  if (idx < COMMANDS_NB) {
    // Bound value ???
    commands[idx] = value;
  }
}

/** Get a command value
 * @param idx command index
 * @return current value, 0 if index is invalid
 */
static inline pprz_t command_get(uint8_t idx)
{
  if (idx < COMMANDS_NB) {
    return commands[idx];
  }
  return 0; // is it the best value ???
}

extern void commands_init(void);

#endif /*  COMMANDS_H */
