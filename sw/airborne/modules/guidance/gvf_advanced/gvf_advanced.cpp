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
 * @file modules/guidance/gvf_advanced/gvf_advanced.cpp
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 */

#include <std.h>

#include "subsystems/datalink/telemetry.h"
#include "gvf_advanced.h"
#include "autopilot.h"
#include "std.h"

uint32_t t0 = 0; // We need it for calculting the time lapse delta_T

void gvf_advanced_init(void)
{

}

void gvf_advanced_control_2D(void)
{

}

void gvf_advanced_control_3D(void)
{

}

bool gvf_advanced_3D_ellipse(void)
{
  uint32_t now = get_sys_time_msec();
  uint32_t delta_T = now - t0;
  t0 = now;

  gvf_advanced_control_3D();

  return true;
}


