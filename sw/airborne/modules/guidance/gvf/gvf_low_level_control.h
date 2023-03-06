/*
 * Copyright (C) 2020 Hector Garcia de Marina <hgarciad@ucm.es>
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
 * @file modules/guidance/gvf/gvf_low_level_control.h
 *
 * Firmware dependent file for the guiding vector field algorithm for 2D trajectories.
 */

#ifndef GVF_LOW_LEVEL_CONTROL_H
#define GVF_LOW_LEVEL_CONTROL_H

#ifdef FIXEDWING_FIRMWARE
#include "firmwares/fixedwing/nav.h"
#include "modules/nav/common_nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#define gvf_setNavMode(_navMode) (nav.horizontal_mode = _navMode)
#define GVF_MODE_ROUTE NAV_HORIZONTAL_MODE_ROUTE
#define GVF_MODE_WAYPOINT HORIZONTAL_MODE_WAYPOINT
#define GVF_MODE_CIRCLE HORIZONTAL_MODE_CIRCLE

#elif defined(ROVER_FIRMWARE)
#include "state.h"
#include "firmwares/rover/navigation.h"
#define gvf_setNavMode(_navMode) (nav.mode = _navMode)
#define GVF_MODE_ROUTE NAV_MODE_ROUTE
#define GVF_MODE_WAYPOINT NAV_MODE_WAYPOINT
#define GVF_MODE_CIRCLE NAV_MODE_CIRCLE

#else
#error "GVF does not support your firmware yet!"
#endif

// Low level control functions
extern void gvf_low_level_getState(void);
extern void gvf_low_level_control_2D(float);

#endif // GVF_LOW_LEVEL_CONTROL_H

