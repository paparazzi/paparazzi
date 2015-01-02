/*
 *
 * Copyright (C) 2003-2011 Pascal Brisset, Antoine Drouin
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
/** \file cam.c
 *  \brief Pan/Tilt camera library
 *
 */

#include <math.h>
#include "cam.h"
#include "firmwares/fixedwing/nav.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "state.h"
#include "inter_mcu.h"

#ifndef CAM_PHI_MAX
#define CAM_PHI_MAX RadOfDeg(45)
#endif

float cam_roll_phi; /* radian */
float phi_c; /* radian */
float theta_c; /* have to be defined for telemetry message */

float target_x, target_y, target_alt;

#ifdef MOBILE_CAM

#define MODE_MANUAL     0
#define MODE_STABILIZED 1

#ifndef CAM_ROLL_START_MODE
#define CAM_ROLL_START_MODE MODE_MANUAL
#endif

uint8_t cam_roll_mode;

void cam_init(void)
{
  cam_roll_mode = CAM_ROLL_START_MODE;
}

void cam_periodic(void)
{
  switch (cam_roll_mode) {
    case MODE_STABILIZED:
      phi_c = cam_roll_phi + stateGetNedToBodyEulers_f()->phi;
      break;
    case MODE_MANUAL:
      phi_c = cam_roll_phi;
      break;
    default:
      phi_c = 0;
  }
  ap_state->commands[COMMAND_CAM_ROLL] = TRIM_PPRZ(phi_c * MAX_PPRZ / CAM_PHI_MAX);
}

#endif // MOBILE_CAM
