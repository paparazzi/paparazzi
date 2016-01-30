/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 * @file firmwares/rotorcraft/rotorcraft_datalink.c
 * Handling of messages coming from ground and other A/Cs.
 *
 */

#include "subsystems/datalink/datalink.h"

#include "pprzlink/dl_protocol.h"

#ifdef USE_NAVIGATION
#include "firmwares/rotorcraft/navigation.h"
#include "math/pprz_geodetic_int.h"
#endif

#include "firmwares/rotorcraft/autopilot.h"

void firmware_parse_msg(void)
{
  uint8_t msg_id = IdOfPprzMsg(dl_buffer);

  /* parse telemetry messages coming from ground station */
  switch (msg_id) {

#ifdef USE_NAVIGATION
    case DL_BLOCK : {
      if (DL_BLOCK_ac_id(dl_buffer) != AC_ID) { break; }
      nav_goto_block(DL_BLOCK_block_id(dl_buffer));
    }
    break;

    case DL_MOVE_WP : {
      uint8_t ac_id = DL_MOVE_WP_ac_id(dl_buffer);
      if (ac_id != AC_ID) { break; }
      if (stateIsLocalCoordinateValid()) {
        uint8_t wp_id = DL_MOVE_WP_wp_id(dl_buffer);
        struct LlaCoor_i lla;
        lla.lat = DL_MOVE_WP_lat(dl_buffer);
        lla.lon = DL_MOVE_WP_lon(dl_buffer);
        /* WP_alt from message is alt above MSL in mm
         * lla.alt is above ellipsoid in mm
         */
        lla.alt = DL_MOVE_WP_alt(dl_buffer) - state.ned_origin_i.hmsl +
                  state.ned_origin_i.lla.alt;
        waypoint_move_lla(wp_id, &lla);
      }
    }
    break;
#endif /* USE_NAVIGATION */

    case DL_GUIDED_SETPOINT_NED:
      if (DL_GUIDED_SETPOINT_NED_ac_id(dl_buffer) != AC_ID) { break; }
      uint8_t flags = DL_GUIDED_SETPOINT_NED_flags(dl_buffer);
      float x = DL_GUIDED_SETPOINT_NED_x(dl_buffer);
      float y = DL_GUIDED_SETPOINT_NED_y(dl_buffer);
      float z = DL_GUIDED_SETPOINT_NED_z(dl_buffer);
      float yaw = DL_GUIDED_SETPOINT_NED_yaw(dl_buffer);
      switch (flags) {
        case 0x00:
        case 0x02:
          /* local NED position setpoints */
          autopilot_guided_goto_ned(x, y, z, yaw);
          break;
        case 0x01:
          /* local NED offset position setpoints */
          autopilot_guided_goto_ned_relative(x, y, z, yaw);
          break;
        case 0x03:
          /* body NED offset position setpoints */
          autopilot_guided_goto_body_relative(x, y, z, yaw);
          break;
        case 0x70:
          /* local NED with x/y/z as velocity and yaw as absolute angle */
          autopilot_guided_move_ned(x, y, z, yaw);
          break;
        default:
          /* others not handled yet */
          break;
      }
    default:
      break;
  }
}
