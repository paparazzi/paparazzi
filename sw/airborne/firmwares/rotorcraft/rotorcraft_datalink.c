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

#include "modules/datalink/datalink.h"

#include "pprzlink/dl_protocol.h"

#ifdef USE_NAVIGATION
#include "firmwares/rotorcraft/navigation.h"
#include "math/pprz_geodetic_int.h"
#endif

#include "autopilot.h"
#include "firmwares/rotorcraft/autopilot_guided.h"

void firmware_parse_msg(struct link_device *dev __attribute__((unused)), struct transport_tx *trans __attribute__((unused)), uint8_t *buf)
{
  uint8_t msg_id = IdOfPprzMsg(buf);

  /* parse telemetry messages coming from ground station */
  switch (msg_id) {
#ifndef INTERMCU_FBW

#ifdef USE_NAVIGATION
    case DL_BLOCK : {
      if (DL_BLOCK_ac_id(buf) != AC_ID) { break; }
      nav_goto_block(DL_BLOCK_block_id(buf));
    }
    break;

    case DL_MOVE_WP : {
      uint8_t ac_id = DL_MOVE_WP_ac_id(buf);
      if (ac_id != AC_ID) { break; }
      if (stateIsLocalCoordinateValid()) {
        uint8_t wp_id = DL_MOVE_WP_wp_id(buf);
        struct LlaCoor_i lla;
        lla.lat = DL_MOVE_WP_lat(buf);
        lla.lon = DL_MOVE_WP_lon(buf);
        /* WP_alt from message is alt above MSL in mm
         * lla.alt is above ellipsoid in mm
         */
        lla.alt = DL_MOVE_WP_alt(buf) - state.ned_origin_i.hmsl +
                  state.ned_origin_i.lla.alt;
        waypoint_move_lla(wp_id, &lla);
      }
    }
    break;
#endif /* USE_NAVIGATION */

#ifdef AP_MODE_GUIDED
    case DL_GUIDED_SETPOINT_NED:
      if (DL_GUIDED_SETPOINT_NED_ac_id(buf) != AC_ID) { break; }

      autopilot_guided_update(DL_GUIDED_SETPOINT_NED_flags(buf),
                              DL_GUIDED_SETPOINT_NED_x(buf),
                              DL_GUIDED_SETPOINT_NED_y(buf),
                              DL_GUIDED_SETPOINT_NED_z(buf),
                              DL_GUIDED_SETPOINT_NED_yaw(buf));
      break;
#endif

#endif // INTERMCU_FBW
    default:
      break;
  }
}
