/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

/** @file firmwares/rotorcraft/stabilization.h
 *  General stabilization interface for rotorcrafts.
 */

#ifndef STABILIZATION_H
#define STABILIZATION_H

#include "std.h"

#include "generated/airframe.h"

extern void stabilization_init(void);
extern void stabilization_filter_commands(void);

/** Stabilization commands.
 *  Contains the resulting stabilization commands,
 *  regardless of whether rate or attitude is currently used.
 *  Range -MAX_PPRZ:MAX_PPRZ
 */
extern int32_t stabilization_cmd[COMMANDS_NB];

#endif /* STABILIZATION_H */
