/*
 * Copyright (C) 2023 Alfredo Gonzalez Calvin <alfredgo@ucm.es>
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

#include "gvf_common.h"

// Get MODULE_H from loaded modules
#include "generated/modules.h" 

/** ------------------------------------------------------------------------ **/

gvf_common_ctrl  gvf_c_ctrl;
gvf_common_info gvf_c_info;
gvf_common_params gvf_c_params = {1,1};

// State
gvf_common_state gvf_c_state;

/** ------------------------------------------------------------------------ **/

void gvf_low_level_getState(void)
{

  gvf_c_state.px = stateGetPositionEnu_f()->x;
  gvf_c_state.py = stateGetPositionEnu_f()->y;

  #if defined(FIXEDWING_FIRMWARE)
    float ground_speed = stateGetHorizontalSpeedNorm_f();
    gvf_c_state.course = stateGetHorizontalSpeedDir_f();
    gvf_c_state.px_dot = ground_speed * sinf(gvf_c_state.course);
    gvf_c_state.py_dot = ground_speed * cosf(gvf_c_state.course);
    
  #elif defined(ROVER_FIRMWARE) || defined(ROTORCRAFT_FIRMWARE)
    // We assume that the course and psi
    // of the rover (steering wheel) are the same
    gvf_c_state.course = stateGetNedToBodyEulers_f()->psi;
    gvf_c_state.px_dot = stateGetSpeedEnu_f()->x;
    gvf_c_state.py_dot = stateGetSpeedEnu_f()->y;
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
      - gvf_c_params.k_roll * atanf(omega * ground_speed / GVF_GRAVITY / cosf(att->theta));
    BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);
  }
  #endif

  gvf_c_ctrl.omega  = omega;
}

void gvf_low_level_control_3D(float heading_rate, float climbing_rate)
{
  #if defined(FIXEDWING_FIRMWARE)
  if (autopilot_get_mode() == AP_MODE_AUTO2) {
    // Vertical Z coordinate
    v_ctl_mode = V_CTL_MODE_AUTO_CLIMB;
    v_ctl_speed_mode = V_CTL_SPEED_THROTTLE;

    v_ctl_climb_setpoint = gvf_c_params.k_climb * climbing_rate; // Setting point for vertical speed

    // Lateral XY coordinates
    lateral_mode = LATERAL_MODE_ROLL;

    struct FloatEulers *att = stateGetNedToBodyEulers_f();
    float ground_speed = stateGetHorizontalSpeedNorm_f();

    h_ctl_roll_setpoint =
      -gvf_c_params.k_roll * atanf(heading_rate * ground_speed / GVF_GRAVITY / cosf(att->theta));
    BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint); // Setting point for roll angle
  }
  #endif

  gvf_low_level_control_2D(heading_rate);

  (void)(climbing_rate); // Avoid unused parameter warning
}

bool gvf_nav_approaching(float wp_x, float wp_y, float from_x, float from_y, float t) 
{

  #if defined(FIXEDWING_FIRMWARE)
  return nav_approaching_xy(wp_x, wp_y, from_x, from_y, t);

  #else // FIXEDWING & ROVER FIRMWARE
  struct EnuCoor_f wp, from;

  wp.x = wp_x;
  wp.y = wp_y;
  from.x = from_x;
  from.y = from_y;
  
  return nav.nav_approaching(&wp, &from, t);
  #endif

  // Avoid unused parameter warning
}