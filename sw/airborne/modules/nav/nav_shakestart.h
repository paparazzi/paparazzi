/*
 * Copyright (C) 2025 MAVLab <microuav@gmail.com>
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

/** @file "modules/nav/nav_shakestart.h"
 * @author MAVLab <microuav@gmail.com>
 * Shake the RPAS for 1 second to start engines
 */

#ifndef NAV_SHAKESTART_H
#define NAV_SHAKESTART_H


#include "std.h"

extern void nav_shakestart_init(void);
extern void nav_shakestart_periodic(void);
extern bool nav_shakestart_run(void);
extern void nav_shakestart_reset(void);

#endif  // NAV_SHAKESTART_H
