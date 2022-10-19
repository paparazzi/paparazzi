/*
 * Copyright (C) 2016 The Paparazzi Team
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
 */

/**
 * @file firmwares/rotorcraft/autopilot_guided.c
 *
 * Autopilot guided mode interface.
 *
 */

#include "firmwares/rotorcraft/autopilot_guided.h"
#include "autopilot.h"
#include "firmwares/rotorcraft/guidance.h"
#include "state.h"
#include "pprzlink/dl_protocol.h"


bool autopilot_guided_goto_ned(float x, float y, float z, float heading)
{
  if (autopilot_get_mode() == AP_MODE_GUIDED) {
    guidance_h_set_pos(x, y);
    guidance_h_set_heading(heading);
    guidance_v_set_z(z);
    return true;
  }
  return false;
}

bool autopilot_guided_goto_ned_relative(float dx, float dy, float dz, float dyaw)
{
  if (autopilot_get_mode() == AP_MODE_GUIDED && stateIsLocalCoordinateValid()) {
    float x = stateGetPositionNed_f()->x + dx;
    float y = stateGetPositionNed_f()->y + dy;
    float z = stateGetPositionNed_f()->z + dz;
    float heading = stateGetNedToBodyEulers_f()->psi + dyaw;
    return autopilot_guided_goto_ned(x, y, z, heading);
  }
  return false;
}

bool autopilot_guided_goto_body_relative(float dx, float dy, float dz, float dyaw)
{
  if (autopilot_get_mode() == AP_MODE_GUIDED && stateIsLocalCoordinateValid()) {
    float psi = stateGetNedToBodyEulers_f()->psi;
    float x = stateGetPositionNed_f()->x + cosf(-psi) * dx + sinf(-psi) * dy;
    float y = stateGetPositionNed_f()->y - sinf(-psi) * dx + cosf(-psi) * dy;
    float z = stateGetPositionNed_f()->z + dz;
    float heading = psi + dyaw;
    return autopilot_guided_goto_ned(x, y, z, heading);
  }
  return false;
}

bool autopilot_guided_move_ned(float vx, float vy, float vz, float heading)
{
  if (autopilot_get_mode() == AP_MODE_GUIDED) {
    guidance_h_set_vel(vx, vy);
    guidance_h_set_heading(heading);
    guidance_v_set_vz(vz);
    return true;
  }
  return false;
}

/* Set guided mode setpoint
 * Note: Offset position command in NED frame or body frame will only be implemented if
 * local reference frame has been initialised.
 * Flag definition:
   bit 0: x,y as offset coordinates
   bit 1: x,y in body coordinates
   bit 2: z as offset coordinates
   bit 3: yaw as offset coordinates
   bit 4: free
   bit 5: x,y as vel
   bit 6: z as vel
   bit 7: yaw as rate
 */
void autopilot_guided_update(uint8_t flags, float x, float y, float z, float yaw)
{
  // handle x,y
  struct FloatVect2 setpoint = {.x = x, .y = y};
  if (bit_is_set(flags, 5)) { // velocity setpoint
    if (bit_is_set(flags, 1)) { // set velocity in body frame
      guidance_h_set_body_vel(setpoint.x, setpoint.y);
    } else {
      guidance_h_set_vel(setpoint.x, setpoint.y);
    }
  } else {  // position setpoint
    if (!bit_is_set(flags, 0) && !bit_is_set(flags, 1)) {   // set absolute position setpoint
      guidance_h_set_pos(setpoint.x, setpoint.y);
    } else {
      if (stateIsLocalCoordinateValid()) {
        if (bit_is_set(flags, 1)) {  // set position as offset in body frame
          float psi = stateGetNedToBodyEulers_f()->psi;

          setpoint.x = stateGetPositionNed_f()->x + cosf(-psi) * x + sinf(-psi) * y;
          setpoint.y = stateGetPositionNed_f()->y - sinf(-psi) * x + cosf(-psi) * y;
        } else {                     // set position as offset in NED frame
          setpoint.x += stateGetPositionNed_f()->x;
          setpoint.y += stateGetPositionNed_f()->y;
        }
        guidance_h_set_pos(setpoint.x, setpoint.y);
      }
    }
  }

  //handle z
  if (bit_is_set(flags, 6)) { // speed set-point
    guidance_v_set_vz(z);
  } else {    // position set-point
    if (bit_is_set(flags, 2)) { // set position as offset in NED frame
      if (stateIsLocalCoordinateValid()) {
        z += stateGetPositionNed_f()->z;
        guidance_v_set_z(z);
      }
    } else {
      guidance_v_set_z(z);
    }
  }

  //handle yaw
  if (bit_is_set(flags, 7)) { // speed set-point
    guidance_h_set_heading_rate(yaw);
  } else {    // position set-point
    if (bit_is_set(flags, 3)) { // set yaw as offset
      yaw += stateGetNedToBodyEulers_f()->psi;  // will be wrapped to [-pi,pi] later
    }
    guidance_h_set_heading(yaw);
  }
}

/** Parse GUIDED_SETPOINT_NED messages from datalink
 */
void autopilot_guided_parse_GUIDED(uint8_t *buf) {
  if (DL_GUIDED_SETPOINT_NED_ac_id(buf) != AC_ID || autopilot_get_mode() != AP_MODE_GUIDED) {
    return;
  }

  autopilot_guided_update(
      DL_GUIDED_SETPOINT_NED_flags(buf),
      DL_GUIDED_SETPOINT_NED_x(buf),
      DL_GUIDED_SETPOINT_NED_y(buf),
      DL_GUIDED_SETPOINT_NED_z(buf),
      DL_GUIDED_SETPOINT_NED_yaw(buf));
}

