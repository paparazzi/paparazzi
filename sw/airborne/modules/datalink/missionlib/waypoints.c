/*
 * Copyright (C) 2015 Lodewijk Sikkel <l.n.c.sikkel@tudelft.nl>
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

/** @file modules/datalink/missionlib/waypoints.c
 *  @brief Improvement of the missionlib implementation of the waypoint protocol,
 *  truly global waypoints are used such that they will not be relocated after you
 *  run GeoInit.
 */

// Include own header
#include "modules/datalink/missionlib/waypoints.h"

#include <stdio.h>
#include <string.h>

#include "subsystems/navigation/waypoints.h"
//#include "subsystems/navigation/common_nav.h" // for fixed-wing aircraft

#include "modules/datalink/mavlink.h"
#include "modules/datalink/missionlib/mission_manager.h"

static void mavlink_update_wp_list(void)
{
  // Store the waypoints in a mission item
  if (NB_WAYPOINT > 0) {
//        for (uint8_t i = 0; i < NB_WAYPOINT; i++) {
//            mavlink_mission_item_t mission_item;
//
//            /*
//             * Convert ENU waypoint to UTM
//             * May be removed, but nav_move_waypoint() uses UTM coordinates in case of fixed-wing aircraft
//             */
//            struct UtmCoor_f utm;
//            utm.east = waypoints[i].x + nav_utm_east0;
//            utm.north = waypoints[i].y + nav_utm_north0;
//            utm.alt = waypoints[i].a;
//            utm.zone = nav_utm_zone0;
//
//            // Convert UTM waypoint to LLA
//            struct LlaCoor_f lla;
//            lla_of_utm_f(&lla, &utm);
//            mission_item.x = lla.lat; // lattitude
//            mission_item.y = lla.lon; // longtitude
//            mission_item.z = lla.alt; // altitude
//            mission_item.seq = i;
//
//            mission_mgr.waypoints[i] = mission_item;
//        }
    for (uint8_t i = 0; i < NB_WAYPOINT; i++) {
      mavlink_mission_item_t mission_item;
      // waypoint_set_global_flag(i);
      if (waypoint_is_global(i)) {
        MAVLINK_DEBUG("Waypoint(%d): is global\n", i);
      } else {
        MAVLINK_DEBUG("Waypoint(%d): is NOT global\n", i);
      }
      waypoint_globalize(i);
      mission_item.x = (float)waypoint_get_lla(i)->lat * 1e-7; // lattitude
      mission_item.y = (float)waypoint_get_lla(i)->lon * 1e-7; // longtitude
      mission_item.z = (float)waypoint_get_lla(i)->alt * 1e-3; // altitude

      MAVLINK_DEBUG("WP: %f, %f, %f\n", mission_item.x, mission_item.y, mission_item.z);

      mission_item.seq = i;
      mission_mgr.waypoints[i] = mission_item;
    }
  } else {
    MAVLINK_DEBUG("ERROR: The waypoint array is empty\n");
  }
}

static void mavlink_send_wp_count(void)
{
  mavlink_message_t msg;
  mavlink_mission_count_t wp_count;
  wp_count.target_system = mission_mgr.rem_sysid;
  wp_count.target_component = mission_mgr.rem_compid;
  wp_count.count = NB_WAYPOINT; // From the generated flight plan

  mavlink_msg_mission_count_encode(mavlink_system.sysid, mavlink_system.compid, &msg,
                                   &wp_count); // encode the block count message

  MAVLINK_DEBUG("Sent WP_COUNT message\n");
  mavlink_send_message(&msg);
}

static void mavlink_send_wp(uint16_t seq)
{
  if (seq < NB_WAYPOINT) { // Due to indexing
    mavlink_message_t msg;
    mavlink_mission_item_t *mission_item = &(mission_mgr.waypoints[seq]); // Copy the reference to the mission item

    mission_item->target_system = mission_mgr.rem_sysid;
    mission_item->target_component = mission_mgr.rem_compid;

    mavlink_msg_mission_item_encode(mavlink_system.sysid, mavlink_system.compid, &msg, mission_item);

    MAVLINK_DEBUG("Sent MISSION_ITEM message\n");
    mavlink_send_message(&msg);
  } else {
    MAVLINK_DEBUG("ERROR: Wp index out of bounds\n");
  }
}

void mavlink_wp_init(void)
{
  mavlink_update_wp_list();
}

void mavlink_wp_message_handler(const mavlink_message_t *msg)
{
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
      MAVLINK_DEBUG("Received MISSION_REQUEST_LIST message\n");
      mavlink_mission_request_list_t mission_request_list_msg;
      mavlink_msg_mission_request_list_decode(msg,
                                              &mission_request_list_msg); // Cast the incoming message to a mission_request_list_msg
      if (mission_request_list_msg.target_system == mavlink_system.sysid) {
        if (mission_mgr.state == STATE_IDLE) {
          if (NB_WAYPOINT > 0) {
            mission_mgr.state = STATE_SEND_LIST;
            MAVLINK_DEBUG("State: %d\n", mission_mgr.state);
            mission_mgr.seq = 0;
            mission_mgr.rem_sysid = msg->sysid;
            mission_mgr.rem_compid = msg->compid;
          }
          mavlink_send_wp_count();

          mission_mgr.timer_id = sys_time_register_timer(MAVLINK_TIMEOUT,
                                 &timer_cb); // Register the timeout timer (it is continuous so it needs to be cancelled after triggering)
        } else {
          // TODO: Handle case when the state is not IDLE
        }
      } else {
        // TODO: Handle remote system id mismatch
      }

      break;
    }

    case MAVLINK_MSG_ID_MISSION_REQUEST: {
      MAVLINK_DEBUG("Received MISSION_REQUEST message\n");
      mavlink_mission_request_t mission_request_msg;
      mavlink_msg_mission_request_decode(msg, &mission_request_msg); // Cast the incoming message to a mission_request_msg
      if (mission_request_msg.target_system == mavlink_system.sysid) {
        if ((mission_mgr.state == STATE_SEND_LIST && mission_request_msg.seq == 0) || // Send the first waypoint
            (mission_mgr.state == STATE_SEND_ITEM && (mission_request_msg.seq == mission_mgr.seq
                || // Send the current waypoint again
                mission_request_msg.seq == mission_mgr.seq + 1))) { // Send the next waypoint
          sys_time_cancel_timer(mission_mgr.timer_id); // Cancel the timeout timer

          mission_mgr.state = STATE_SEND_ITEM;
          MAVLINK_DEBUG("State: %d\n", mission_mgr.state);
          mission_mgr.seq = mission_request_msg.seq;

          mavlink_update_wp_list(); // Update the waypoint list

          mavlink_send_wp(mission_mgr.seq);

          mission_mgr.timer_id = sys_time_register_timer(MAVLINK_TIMEOUT, &timer_cb); // Register the timeout timer
        } else {
          // TODO: Handle cases for which the above condition does not hold
        }
      } else {
        // TODO: Handle remote system id mismatch
      }

      break;
    }

    case MAVLINK_MSG_ID_MISSION_ITEM: {
      MAVLINK_DEBUG("Received MISSION_ITEM message\n");
      mavlink_mission_item_t mission_item_msg;
      mavlink_msg_mission_item_decode(msg, &mission_item_msg); // Cast the incoming message to a mission_item_msg

      if (mission_item_msg.target_system == mavlink_system.sysid) {
        if (mission_mgr.state == STATE_IDLE) { // Only handle incoming mission item messages if there a not currently being sent
          struct LlaCoor_i lla;
          lla.lat = (int32_t)(mission_item_msg.x * 1e7); // lattitude in degrees*1e7
          lla.lon = (int32_t)(mission_item_msg.y * 1e7); // longitude in degrees*1e7
          lla.alt = (int32_t)(mission_item_msg.z * 1e3); // altitude in millimeters

//                    struct UtmCoor_f utm;
//                    utm.zone = nav_utm_zone0;
//                    utm_of_lla_f(&utm, &lla);
          MAVLINK_DEBUG("Received WP(%d): %d, %d, %d\n", mission_item_msg.seq, lla.lat, lla.lon, lla.alt);
          // Set the new waypoint
          waypoint_move_lla(mission_item_msg.seq, &lla); // move the waypoint with the id equal to seq, only for rotorcraft
//                    nav_move_waypoint(mission_item_msg.seq, utm.east, utm.north, utm.alt);

          sendMissionAck();
        }
      }
    }
  }
}
