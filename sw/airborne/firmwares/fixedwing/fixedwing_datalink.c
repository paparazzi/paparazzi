/*
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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
 * @file firmwares/fixedwing/fixedwing_datalink.c
 * Handling of messages coming from ground and other A/Cs.
 *
 */

#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"

#include "pprzlink/messages.h"
#include "pprzlink/dl_protocol.h"

#if defined NAV || defined WIND_INFO
#include "state.h"
#endif

#ifdef NAV
#include "firmwares/fixedwing/nav.h"
#include "subsystems/navigation/common_nav.h"
#include "math/pprz_geodetic_float.h"
#endif

#ifdef HITL
#include "subsystems/gps.h"
#endif

#define MOfMM(_x) (((float)(_x))/1000.)

void firmware_parse_msg(struct link_device *dev __attribute__((unused)), struct transport_tx *trans __attribute__((unused)), uint8_t *buf)
{
  uint8_t msg_id = IdOfPprzMsg(buf);

  /* parse telemetry messages coming from ground station */
  switch (msg_id) {

#ifdef NAV
    case DL_BLOCK: {
      if (DL_BLOCK_ac_id(buf) != AC_ID) { break; }
      nav_goto_block(DL_BLOCK_block_id(buf));
      SEND_NAVIGATION(trans, dev);
    }
    break;

    case DL_MOVE_WP: {
      if (DL_MOVE_WP_ac_id(buf) != AC_ID) { break; }
      uint8_t wp_id = DL_MOVE_WP_wp_id(buf);

      /* Computes from (lat, long) in the referenced UTM zone */
      struct LlaCoor_f lla;
      lla.lat = RadOfDeg((float)(DL_MOVE_WP_lat(buf) / 1e7));
      lla.lon = RadOfDeg((float)(DL_MOVE_WP_lon(buf) / 1e7));
      lla.alt = MOfMM(DL_MOVE_WP_alt(buf));
      struct UtmCoor_f utm;
      utm.zone = nav_utm_zone0;
      utm_of_lla_f(&utm, &lla);
      nav_move_waypoint(wp_id, utm.east, utm.north, utm.alt);

      /* Waypoint range is limited. Computes the UTM pos back from the relative
         coordinates */
      utm.east = waypoints[wp_id].x + nav_utm_east0;
      utm.north = waypoints[wp_id].y + nav_utm_north0;
      pprz_msg_send_WP_MOVED(trans, dev, AC_ID, &wp_id, &utm.east, &utm.north, &utm.alt, &nav_utm_zone0);
    }
    break;
#endif /** NAV */

#ifdef WIND_INFO
    case DL_WIND_INFO: {
      if (DL_WIND_INFO_ac_id(buf) != AC_ID) { break; }
      uint8_t flags = DL_WIND_INFO_flags(buf);
      struct FloatVect2 wind = { 0.f, 0.f };
      float upwind = 0.f;
      if (bit_is_set(flags, 0)) {
        wind.x = DL_WIND_INFO_north(buf);
        wind.y = DL_WIND_INFO_east(buf);
        stateSetHorizontalWindspeed_f(&wind);
      }
      if (bit_is_set(flags, 1)) {
        upwind = DL_WIND_INFO_up(buf);
        stateSetVerticalWindspeed_f(upwind);
      }
#if !USE_AIRSPEED
      if (bit_is_set(flags, 2)) {
        stateSetAirspeed_f(DL_WIND_INFO_airspeed(buf));
      }
#endif
#ifdef WIND_INFO_RET
      float airspeed = stateGetAirspeed_f();
      pprz_msg_send_WIND_INFO_RET(trans, dev, AC_ID, &flags, &wind.y, &wind.x, &upwind, &airspeed);
#endif
    }
    break;
#endif /** WIND_INFO */

#ifdef HITL
    /** Infrared and GPS sensors are replaced by messages on the datalink */
    case DL_HITL_INFRARED: {
      /** This code simulates infrared.c:ir_update() */
      infrared.roll = DL_HITL_INFRARED_roll(buf);
      infrared.pitch = DL_HITL_INFRARED_pitch(buf);
      infrared.top = DL_HITL_INFRARED_top(buf);
    }
    break;

    case DL_HITL_UBX: {
      /** This code simulates gps_ubx.c:parse_ubx() */
      if (gps_msg_received) {
        gps_nb_ovrn++;
      } else {
        ubx_class = DL_HITL_UBX_class(buf);
        ubx_id = DL_HITL_UBX_id(buf);
        uint8_t l = DL_HITL_UBX_ubx_payload_length(buf);
        uint8_t *ubx_payload = DL_HITL_UBX_ubx_payload(buf);
        memcpy(ubx_msg_buf, ubx_payload, l);
        gps_msg_received = true;
      }
    }
    break;
#endif /* HITL */

    default:
      break;
  }
}
