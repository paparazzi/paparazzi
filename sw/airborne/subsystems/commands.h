/*
 * (c) 2006 Pascal Brisset, Antoine Drouin
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

/** \file commands.h
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

#define SetCommands(t) { \
    int i; \
    for(i = 0; i < COMMANDS_NB; i++) commands[i] = t[i]; \
  }

#endif /*  COMMANDS_H */
