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
 * @file modules/guidance/gvf/gvf_low_level_control.c
 *
 * Firmware dependent file for the guiding vector field algorithm for 2D trajectories.
 */

#include "autopilot.h"
#include "modules/guidance/gvf/gvf_low_level_control.h"
#include "modules/guidance/gvf/gvf.h"

#if defined(FIXEDWING_FIRMWARE)
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#endif

void gvf_low_level_getState(void)
{
  #if defined(FIXEDWING_FIRMWARE)
    float ground_speed = stateGetHorizontalSpeedNorm_f();
    gvf_state.course = stateGetHorizontalSpeedDir_f();
    gvf_state.px_dot = ground_speed * sinf(course);
    gvf_state.py_dot = ground_speed * cosf(course);
    
  #elif defined(ROVER_FIRMWARE)
    // We assume that the course and psi
    // of the rover (steering wheel) are the same
    gvf_state.course = stateGetNedToBodyEulers_f()->psi;
    gvf_state.px_dot = stateGetSpeedEnu_f()->x;
    gvf_state.py_dot = stateGetSpeedEnu_f()->y;
  #endif
}

void gvf_low_level_control_2D(float omega)
{
  
#if defined(FIXEDWING_FIRMWARE)
  if (autopilot_get_mode() == AP_MODE_AUTO2) {

    // Coordinated turn
    struct FloatEulers *att = stateGetNedToBodyEulers_f();
    float ground_speed = stateGetHorizontalSpeedNorm_f();

    lateral_mode = LATERAL_MODE_ROLL;

    h_ctl_roll_setpoint =
      -atanf(omega * ground_speed / GVF_GRAVITY / cosf(att->theta));
    BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);
  }
#endif

}

