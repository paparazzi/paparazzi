/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/nav/nav_rotorcraft_hybrid.h"
 * Specific navigation functions for hybrid aircraft
 *
 * FIXME for now, build on top of nav_rotorcraft_base
 */

#ifndef NAV_ROTORCRAFT_HYBRID_H
#define NAV_ROTORCRAFT_HYBRID_H

#include "modules/nav/nav_base.h"
#include "modules/nav/nav_rotorcraft_base.h"

// settings
extern float nav_max_speed;

extern void nav_rotorcraft_hybrid_init(void);

#endif

