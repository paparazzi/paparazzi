/*
 * Copyright (C) 2009-2015 ENAC
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
 * @file modules/nav/nav_poles.h
 *
 */

#ifndef NAV_POLES_H
#define NAV_POLES_H

#include <stdbool.h>
#include "std.h"

extern uint8_t nav_poles_count;
extern float nav_poles_time;
extern int8_t nav_poles_land;

bool nav_poles_init(uint8_t wp1, uint8_t wp2,
                    uint8_t wp1c, uint8_t wp2c,
                    float radius);

#define nav_poles_SetLandDir(_d) { if (_d < 0) _d = -1; else _d = 1; }

#endif

