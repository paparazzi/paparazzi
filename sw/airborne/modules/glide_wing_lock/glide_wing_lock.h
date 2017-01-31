/*
 * Copyright (C) Kevin van Hecke
 *
 * This file is part of paparazzi
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
 * @file "modules/glide_wing_lock/glide_wing_lock.h"
 * @author Kevin van Hecke
 * Locks the wing of an ornicopter so it can glide.
 */

#ifndef GLIDEWINGLOCK_H
#define GLIDEWINGLOCK_H

#include "std.h"
#include "generated/airframe.h"
#include "paparazzi.h"

#define ROTORCRAFT_COMMANDS_THROUGH_MODULE

extern void init(void);
extern void glide_wing_lock_init(void);
extern void glide_wing_lock_event(void);
extern void glide_wing_lock_periodic(void);

//extern void set_rotorcraft_commands(pprz_t *cmd_out, int32_t *cmd_in, bool in_flight __attribute__((unused)), bool motors_on __attribute__((unused)));

#endif

