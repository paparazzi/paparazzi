/*
 * Copyright (C) 2016  2017 Michal Podhradsky <http://github.com/podhrmic>
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
/**
 * @file "modules/mission/copilot_fixedwing.c"
 *
 *  Mission Computer module, interfacing the mission computer (also known as Copilot),
 *  based losely on
 *  ISaAC: The Intelligent Safety and Airworthiness Co-Pilot module
 *  Based on paper "A Payload Verification and Management Framework
 *  for Small UAV-based Personal Remote Sensing Systems" by Cal Coopmans
 *  and Chris Coffin. Link: http://ieeexplore.ieee.org/abstract/document/6309316/
 *
 *  More info can be found on http://wiki.paparazziuav.org/wiki/Mission_computer
 *
 *  Copilot is intended mainly for mapping applications.
 *
 *   This module processes messages from Copilot, and either forwards them to the GCS
 *  (such as CAMERA_SNAPSHOT or CAMERA_PAYLOAD messages), or responds to them as necessary
 *  (such as MOVE_WP).
 *
 *  The module assumes the source of the messages is trusted (i.e. not authentication besides
 *  AC_ID check is performed).
 */

#include "modules/mission/copilot.h"
#include "modules/datalink/telemetry.h"

// needed for WP_MOVED confirmation
#include "firmwares/fixedwing/nav.h"
#include "modules/nav/common_nav.h"
#include "math/pprz_geodetic_float.h"

/**
 * If MOVE_WP from GCS
 *  - processed in  firmware_parse_msg(dev, trans, buf); with regular buffer
 *  - reponse over telemetry (regular buffer)
 *  - here send WP_MOVED over extra_dl
 *
 *  If MOVE_WP from extra_dl
 *  - processed in firmware_parse_msg(dev, trans, buf); with extra buffer
 *  - response over extra_dl
 *  - send an update to GCS
 *
 *  In both cases, the MOVE_WP message was already processed in firmware_parse
 *  here we are taking care only about propagating the change
 *
 */
void copilot_parse_move_wp_dl(uint8_t *buf)
{
  if (DL_MOVE_WP_ac_id(buf) == AC_ID) {
    uint8_t wp_id = DL_MOVE_WP_wp_id(buf);

    if (wp_id >= nb_waypoint) {
      return;
    }

    /* Computes from (lat, long) in the referenced UTM zone */
    struct LlaCoor_f lla;
    lla.lat = RadOfDeg((float)(DL_MOVE_WP_lat(buf) / 1e7));
    lla.lon = RadOfDeg((float)(DL_MOVE_WP_lon(buf) / 1e7));
    lla.alt = ((float)(DL_MOVE_WP_alt(buf)))/1000.;
    struct UtmCoor_f utm;
    utm.zone = nav_utm_zone0;
    utm_of_lla_f(&utm, &lla);

    // Waypoint range is limited. Computes the UTM pos back from the relative
    // coordinates */
    utm.east = waypoints[wp_id].x + nav_utm_east0;
    utm.north = waypoints[wp_id].y + nav_utm_north0;

    if (buf == extra_dl_buffer) {
       // MOVE_WP came from extra_dl, respond over telemetry
      DOWNLINK_SEND_WP_MOVED(DefaultChannel, DefaultDevice,
                             &wp_id, &utm.east, &utm.north, &utm.alt, &nav_utm_zone0);
    }

    if (buf == dl_buffer) {
      // MOVE_WP came over telemetry, respond over extra_dl
      DOWNLINK_SEND_WP_MOVED(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE,
                             &wp_id, &utm.east, &utm.north, &utm.alt, &nav_utm_zone0);
    }

  }
}
