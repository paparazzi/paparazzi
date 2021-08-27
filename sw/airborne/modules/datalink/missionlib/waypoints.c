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
#include "modules/datalink/missionlib/mission_manager.h"
#include "modules/datalink/mavlink.h"

// include mavlink headers, but ignore some warnings
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#include "mavlink/paparazzi/mavlink.h"
#pragma GCC diagnostic pop

#include "generated/flight_plan.h"


#include "subsystems/navigation/waypoints.h"
//#include "subsystems/navigation/common_nav.h" // for fixed-wing aircraft

#include "modules/datalink/missionlib/mission_manager.h"

static void mavlink_send_wp(uint8_t sysid, uint8_t compid, uint16_t seq)
{
  if (seq < NB_WAYPOINT) { // Due to indexing
#ifdef FIXEDWING_FIRMWARE
    /* for fixedwing firmware send as LOCAL_ENU for now */
    mavlink_msg_mission_item_send(MAVLINK_COMM_0,
                                  sysid,
                                  compid,
                                  seq,
                                  MAV_FRAME_LOCAL_ENU,
                                  MAV_CMD_NAV_WAYPOINT,
                                  0, // current
                                  0, // autocontinue
                                  0, 0, 0, 0, // params
                                  WaypointX(seq),
                                  WaypointY(seq),
                                  WaypointAlt(seq));
#else
    /* for rotorcraft firmware use waypoint API and send as lat/lon */
    /* sending lat/lon as float is actually a bad idea,
     *  but it seems that most GCSs don't understand the MISSION_ITEM_INT
     */
    struct LlaCoor_i *lla = waypoint_get_lla(seq);
    mavlink_msg_mission_item_send(MAVLINK_COMM_0,
                                  sysid,
                                  compid,
                                  seq,
                                  MAV_FRAME_GLOBAL,
                                  MAV_CMD_NAV_WAYPOINT,
                                  0, // current
                                  0, // autocontinue
                                  0, 0, 0, 0, // params
                                  (float)lla->lat / 1e7,
                                  (float)lla->lon / 1e7,
                                  (float)lla->alt / 1e3);
#endif
    MAVLinkSendMessage();
    MAVLINK_DEBUG("Sent MISSION_ITEM message with seq %i\n", seq);
  } else {
    MAVLINK_DEBUG("ERROR: Wp index %i out of bounds\n", seq);
  }
}

void mavlink_wp_message_handler(const mavlink_message_t *msg)
{
  switch (msg->msgid) {

    /* request for mission list, answer with number of waypoints */
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
      MAVLINK_DEBUG("Received MISSION_REQUEST_LIST message\n");
      mavlink_mission_request_list_t mission_request_list_msg;
      mavlink_msg_mission_request_list_decode(msg, &mission_request_list_msg);
      if (mission_request_list_msg.target_system == mavlink_system.sysid) {
        if (mission_mgr.state == STATE_IDLE) {
          if (NB_WAYPOINT > 0) {
            mission_mgr.state = STATE_SEND_LIST;
            MAVLINK_DEBUG("State: %d\n", mission_mgr.state);
            mission_mgr.seq = 0;
            mission_mgr.rem_sysid = msg->sysid;
            mission_mgr.rem_compid = msg->compid;
          }
          mavlink_msg_mission_count_send(MAVLINK_COMM_0, msg->sysid, msg->compid, NB_WAYPOINT);
          MAVLinkSendMessage();

          // Register the timeout timer (it is continuous so it needs to be cancelled after triggering)
          mavlink_mission_set_timer();
        } else {
          // TODO: Handle case when the state is not IDLE
        }
      }
      break;
    }

    /* request for mission item, answer with waypoint */
    case MAVLINK_MSG_ID_MISSION_REQUEST: {
      mavlink_mission_request_t req;
      mavlink_msg_mission_request_decode(msg, &req);
      MAVLINK_DEBUG("Received MISSION_REQUEST message with seq %i\n", req.seq);

      if (req.target_system != mavlink_system.sysid || req.seq >= NB_WAYPOINT) {
        return;
      }
      /* Send if:
       * - the first waypoint
       * - current waypoint requested again
       * - or next waypoint requested
       */
      if ((mission_mgr.state == STATE_SEND_LIST && req.seq == 0) ||
          (mission_mgr.state == STATE_SEND_ITEM && (req.seq == mission_mgr.seq ||
                                                    req.seq == mission_mgr.seq + 1))) {
        // Cancel the timeout timer
        mavlink_mission_cancel_timer();

        mission_mgr.state = STATE_SEND_ITEM;
        MAVLINK_DEBUG("State: %d\n", mission_mgr.state);
        mission_mgr.seq = req.seq;

        mavlink_send_wp(msg->sysid, msg->compid, mission_mgr.seq);

        // Register the timeout timer
        mavlink_mission_set_timer();
      } else {
        // TODO: Handle cases for which the above condition does not hold
      }
      break;
    }

#ifndef AP
    /* change waypoints: only available when using waypoint API (rotorcraft firmware)
     * This uses the waypoint_set_x functions (opposed to waypoint_move_x),
     * meaning it doesn't send WP_MOVED Paparazzi messages.
     */

    /* initiate mission/waypoint write transaction */
    case MAVLINK_MSG_ID_MISSION_COUNT: {
      mavlink_mission_count_t mission_count;
      mavlink_msg_mission_count_decode(msg, &mission_count);
      if (mission_count.target_system != mavlink_system.sysid) {
        return;
      }
      MAVLINK_DEBUG("Received MISSION_COUNT message with count %i\n", mission_count.count);
      /* only allow new waypoint update transaction if currently idle */
      if (mission_mgr.state != STATE_IDLE) {
        MAVLINK_DEBUG("MISSION_COUNT error: mission manager not idle.\n");
        return;
      }
      if (mission_count.count != NB_WAYPOINT) {
        MAVLINK_DEBUG("MISSION_COUNT error: request writing %i instead of %i waypoints\n",
                      mission_count.count, NB_WAYPOINT);
        return;
      }
      /* valid initiation of waypoint write transaction, ask for first waypoint */
      MAVLINK_DEBUG("MISSION_COUNT: Requesting first waypoint\n");
      mavlink_msg_mission_request_send(MAVLINK_COMM_0, msg->sysid, msg->compid, 0);
      MAVLinkSendMessage();

      mission_mgr.seq = 0;
      mission_mgr.state = STATE_WAYPOINT_WRITE_TRANSACTION;

      // Register the timeout timer
      mavlink_mission_set_timer();
    }
      break;

    /* got MISSION_ITEM, update one waypoint if in valid transaction */
    case MAVLINK_MSG_ID_MISSION_ITEM: {
      mavlink_mission_item_t mission_item;
      mavlink_msg_mission_item_decode(msg, &mission_item);

      if (mission_item.target_system != mavlink_system.sysid) {
        return;
      }

      MAVLINK_DEBUG("Received MISSION_ITEM message with seq %i and frame %i\n",
                    mission_item.seq, mission_item.frame);

      /* reject non waypoint updates */
      if (mission_item.command != MAV_CMD_NAV_WAYPOINT ||
          mission_item.seq >= NB_WAYPOINT) {
        MAVLINK_DEBUG("rejected MISSION_ITEM command %i, seq %i\n",
                      mission_item.command, mission_item.seq);
        return;
      }

      /* Only accept mission item in IDLE (update single waypoint) or in write transaction */
      if (mission_mgr.state != STATE_IDLE && mission_mgr.state != STATE_WAYPOINT_WRITE_TRANSACTION) {
        MAVLINK_DEBUG("got MISSION_ITEM while not in waypoint write transaction or idle\n");
        return;
      }
      /* If in write transaction, only handle mission item if correct sequence */
      if (mission_mgr.state == STATE_WAYPOINT_WRITE_TRANSACTION && mission_item.seq != mission_mgr.seq) {
        MAVLINK_DEBUG("MISSION_ITEM, got waypoint seq %i, but requested %i\n",
                      mission_item.seq, mission_mgr.seq);
        return;
      }

      if (mission_item.frame == MAV_FRAME_GLOBAL) {
        MAVLINK_DEBUG("MISSION_ITEM, global wp: lat=%f, lon=%f, alt=%f\n",
                      mission_item.x, mission_item.y, mission_item.z);
        struct LlaCoor_i lla;
        lla.lat = mission_item.x * 1e7; // lattitude in degrees*1e7
        lla.lon = mission_item.y * 1e7; // longitude in degrees*1e7
        lla.alt = mission_item.z * 1e3; // altitude in millimeters
        waypoint_set_lla(mission_item.seq, &lla);
      }
      else if (mission_item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT) {
        MAVLINK_DEBUG("MISSION_ITEM, global_rel_alt wp: lat=%f, lon=%f, relative alt=%f\n",
                      mission_item.x, mission_item.y, mission_item.z);
        struct LlaCoor_i lla;
        lla.lat = mission_item.x * 1e7; // lattitude in degrees*1e7
        lla.lon = mission_item.y * 1e7; // longitude in degrees*1e7
        lla.alt = state.ned_origin_i.hmsl + mission_item.z * 1e3; // altitude in millimeters
        waypoint_set_lla(mission_item.seq, &lla);
      }
      else if (mission_item.frame == MAV_FRAME_LOCAL_ENU) {
        MAVLINK_DEBUG("MISSION_ITEM, local_enu wp: x=%f, y=%f, z=%f\n",
                      mission_item.x, mission_item.y, mission_item.z);
        struct EnuCoor_f enu;
        enu.x = mission_item.x;
        enu.y = mission_item.y;
        enu.z = mission_item.z;
        waypoint_set_enu(mission_item.seq, &enu);
      }
      else {
        MAVLINK_DEBUG("No handler for MISSION_ITEM with frame %i\n", mission_item.frame);
        return;
      }
      // acknowledge transmission single waypoint update
      if (mission_mgr.state == STATE_IDLE) {
        MAVLINK_DEBUG("Acknowledge single waypoint update\n");
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                     MAV_MISSION_ACCEPTED);
        MAVLinkSendMessage();
        mavlink_mission_cancel_timer();
      }
      // or end of transaction
      else if (mission_item.seq == NB_WAYPOINT -1) {
        MAVLINK_DEBUG("Acknowledging end of waypoint write transaction\n");
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                     MAV_MISSION_ACCEPTED);
        MAVLinkSendMessage();
        mavlink_mission_cancel_timer();
        mission_mgr.state = STATE_IDLE;
      }
      // or request next waypoint if still in middle of transaction
      else {
        MAVLINK_DEBUG("Requesting waypoint %i\n", mission_item.seq + 1);
        mavlink_msg_mission_request_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                         mission_item.seq + 1);
        MAVLinkSendMessage();
        mission_mgr.seq = mission_item.seq + 1;
        mavlink_mission_set_timer();
      }
      break;
    }

    case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
      mavlink_mission_item_int_t mission_item;
      mavlink_msg_mission_item_int_decode(msg, &mission_item);

      if (mission_item.target_system == mavlink_system.sysid) {
        MAVLINK_DEBUG("Received MISSION_ITEM_INT message with seq %i and frame %i\n",
                      mission_item.seq, mission_item.frame);
        if (mission_item.seq >= NB_WAYPOINT) {
          return;
        }
        if (mission_item.frame == MAV_FRAME_GLOBAL_INT) {
          MAVLINK_DEBUG("MISSION_ITEM_INT, global_int wp: lat=%i, lon=%i, alt=%f\n",
                        mission_item.x, mission_item.y, mission_item.z);
          struct LlaCoor_i lla;
          lla.lat = mission_item.x; // lattitude in degrees*1e7
          lla.lon = mission_item.y; // longitude in degrees*1e7
          lla.alt = mission_item.z * 1e3; // altitude in millimeters
          waypoint_set_lla(mission_item.seq, &lla);
          mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                     MAV_MISSION_ACCEPTED);
          MAVLinkSendMessage();
        }
        else {
          MAVLINK_DEBUG("No handler for MISSION_ITEM_INT with frame %i\n", mission_item.frame);
        return;
      }
      }
    }
      break;
#endif // AP

    default:
      break;
  }
}
