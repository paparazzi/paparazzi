/*
 * $Id$
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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
#include "nav.h"
#include "autopilot.h"
#include "flight_plan.h"
#include "estimator.h"
#include "inter_mcu.h"
#include "nav.h"

#define MIN_PPRZ_CAM ((int16_t)(MAX_PPRZ * 0.05))
#define DELTA_ALPHA 0.2

#define MAX_CAM_ROLL M_PI/2

float cam_roll_phi; /* radian */
float phi_c; /* radian */
float theta_c; /* have to be defined for telemetry message */

float target_x, target_y, target_alt;

#ifdef MOBILE_CAM

#define MODE_MANUAL     0
#define MODE_STABILIZED 1

uint8_t cam_roll_mode;
bool_t cam_roll_switch;

void cam_init( void ) {
  cam_roll_switch = 0;
#ifdef VIDEO_SWITCH_PIN
  IO0DIR |= _BV(VIDEO_SWITCH_PIN);
  IO0CLR = _BV(VIDEO_SWITCH_PIN);
#endif
}

void cam_periodic( void ) {
  switch (cam_roll_mode) {
  case MODE_STABILIZED:
    phi_c = cam_roll_phi + estimator_phi;
    break;
  case MODE_MANUAL:
    phi_c = cam_roll_phi;
    break;
  default:
    phi_c = 0;
  }
  ap_state->commands[COMMAND_CAM_ROLL] = TRIM_PPRZ(phi_c * MAX_PPRZ / RadOfDeg(CAM_PHI_MAX_DEG));
}

#endif // MOBILE_CAM
