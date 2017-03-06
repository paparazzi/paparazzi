/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file firmwares/fixedwing/guidance/guidance_h.c
 * Horizontal guidance logic for fixed wing vehicles.
 *
 * This is a hack for generated autopilot
 */

#include "firmwares/fixedwing/guidance/guidance_h.h"
#include "firmwares/fixedwing/autopilot_firmware.h"
#include "firmwares/fixedwing/guidance/guidance_common.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"

void h_ctl_guidance_loop(void)
{
  if (lateral_mode >= LATERAL_MODE_COURSE) {
    h_ctl_course_loop();  /* aka compute nav_desired_roll */
  }
  // Copy the pitch setpoint from the guidance to the stabilization control
  h_ctl_pitch_setpoint = v_ctl_pitch_setpoint;
}

