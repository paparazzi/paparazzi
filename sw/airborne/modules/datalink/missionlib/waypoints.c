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
#include "modules/datalink/mavlink.h"

// include mavlink headers, but ignore some warnings
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wswitch-default"
#include "mavlink/ardupilotmega/mavlink.h"
#pragma GCC diagnostic pop

#include "modules/mission/mission_common.h"
#include "state.h"

#include "modules/nav/waypoints.h"

#include "modules/datalink/missionlib/mission_manager.h"

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

uint8_t current_bool = 0;

// Declaration of helper functions to convert mavlink mission items to lla
inline void mavlink_lla_of_global(mavlink_mission_item *mission_item, struct LlaCoor_i *lla);
inline void mavlink_lla_of_global_relative_alt(mavlink_mission_item *mission_item, struct LlaCoor_i *lla);

static void mavlink_send_wp(uint8_t sysid, uint8_t compid, uint16_t seq)
{
  MAVLINK_DEBUG("SYSID = %i, COMPID = %i\n", sysid, compid);
  if (seq < mission_mgr.active_count && seq < MISSION_ELEMENT_NB) { // Due to indexing
    mavlink_msg_mission_item_int_send(MAVLINK_COMM_0,
                                  sysid,
                                  compid,
                                  seq,
                                  mission_mgr.active_mission_items[seq].frame,
                                  mission_mgr.active_mission_items[seq].cmd,
                                  mission_mgr.active_mission_items[seq].current, // current
                                  mission_mgr.active_mission_items[seq].autocontinue, // autocontinue
                                  0, 0, 0, 0, // params
                                  mission_mgr.active_mission_items[seq].x,
                                  mission_mgr.active_mission_items[seq].y,
                                  mission_mgr.active_mission_items[seq].z,
                                  MAV_MISSION_TYPE_MISSION);
    MAVLinkSendMessage();
    MAVLINK_DEBUG("Sent MISSION_ITEM_INT message with seq %i\n", seq);
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
        switch (mission_request_list_msg.mission_type) {
          case MAV_MISSION_TYPE_MISSION: {
            if (mission_mgr.state == STATE_IDLE) {
              mission_mgr.state = STATE_SEND_LIST;
              MAVLINK_DEBUG("State: %d\n", mission_mgr.state);
              mission_mgr.seq = 0;
              mission_mgr.rem_sysid = msg->sysid;
              mission_mgr.rem_compid = msg->compid;
              mavlink_msg_mission_count_send(MAVLINK_COMM_0, msg->sysid, msg->compid, mission_mgr.active_count, MAV_MISSION_TYPE_MISSION);
              MAVLinkSendMessage();

              // Register the timeout timer (it is continuous so it needs to be cancelled after triggering)
              mavlink_mission_set_timer();
            } else {
              // TODO: Handle case when the state is not IDLE
            } break;
          }

          case MAV_MISSION_TYPE_FENCE: {
            mavlink_msg_mission_count_send(MAVLINK_COMM_0, msg->sysid, msg->compid, 0, MAV_MISSION_TYPE_FENCE);
            MAVLinkSendMessage();
            // TODO: Add support for MISSION_TYPE_FENCE
            break;
          }

          case MAV_MISSION_TYPE_RALLY: {
            mavlink_msg_mission_count_send(MAVLINK_COMM_0, msg->sysid, msg->compid, 0, MAV_MISSION_TYPE_RALLY);
            MAVLinkSendMessage();
            // TODO: Add support for MISSION_TYPE_RALLY
            break;
          }
        }
      }
      break;
    }

    /* request for mission item, answer with waypoint */
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
      mavlink_mission_request_t req;
      mavlink_msg_mission_request_decode(msg, &req);
      MAVLINK_DEBUG("Received MISSION_REQUEST_INT message with seq %i\n", req.seq);

      if (req.target_system != mavlink_system.sysid || req.seq >= mission_mgr.active_count) {
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

    /* initiate mission/waypoint write transaction */
    case MAVLINK_MSG_ID_MISSION_COUNT: {
      mavlink_mission_count_t mission_count;
      mavlink_msg_mission_count_decode(msg, &mission_count);
      if (mission_count.target_system != mavlink_system.sysid) {
        MAVLINK_DEBUG("MISSION_COUNT FAIL target id not equal to sysid\n");
        return;
      }
      MAVLINK_DEBUG("Received MISSION_COUNT message with count %i\n", mission_count.count);
      /* only allow new waypoint update transaction if currently idle */
      if (mission_mgr.state != STATE_IDLE) {
        MAVLINK_DEBUG("MISSION_COUNT error: mission manager not idle.\n");
        return;
      }
      if (mission_count.count > MISSION_ELEMENT_NB) {
        MAVLINK_DEBUG("MISSION_COUNT error: request writing %i waypoints while %i waypoints is maximum\n",
                      mission_count.count, MISSION_ELEMENT_NB);
        return;
      }
      /* If fence */
      if (mission_count.mission_type == MAV_MISSION_TYPE_FENCE) {
        MAVLINK_DEBUG("MISSION_COUNT: Requesting first geofence point\n");
        MAVLINK_DEBUG("Acknowledging end of geofence write transaction\n");
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                     MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_FENCE);
        MAVLinkSendMessage();
        return;
      }
      /* If rally*/
      if (mission_count.mission_type == MAV_MISSION_TYPE_RALLY) {
        MAVLINK_DEBUG("MISSION_COUNT: Requesting first rally point\n");
        MAVLINK_DEBUG("Acknowledging end of rally write transaction\n");
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                     MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_RALLY);
        MAVLinkSendMessage();
        return;
      }

      /* valid initiation of waypoint write transaction, ask for first waypoint */
      mission_mgr.count = mission_count.count;
      MAVLINK_DEBUG("MISSION_COUNT: Requesting first waypoint\n");
      mavlink_msg_mission_request_int_send(MAVLINK_COMM_0, msg->sysid, msg->compid, 0, MAV_MISSION_TYPE_MISSION);
      MAVLinkSendMessage();

      mission_mgr.seq = 0;
      mission_mgr.state = STATE_WAYPOINT_WRITE_TRANSACTION;

      // Register the timeout timer
      mavlink_mission_set_timer();
      break;
    }

    /* initiate partial mission/waypoint write transaction */
    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST: {
      mavlink_mission_write_partial_list_t mission_write_partial_list;
      mavlink_msg_mission_write_partial_list_decode(msg, &mission_write_partial_list);
      if (mission_write_partial_list.target_system != mavlink_system.sysid) {
        MAVLINK_DEBUG("MISSION_WRITE_PARTIAL_LIST FAIL target id not equal to sysid\n");
        return;
      }
      MAVLINK_DEBUG("Received MISSION_WRITE_PARTIAL_LIST message with start idx %i and end idx %i\n", mission_write_partial_list.start_index, mission_write_partial_list.end_index);
      /* only allow new waypoint update transaction if currently idle */
      if (mission_mgr.state != STATE_IDLE) {
        MAVLINK_DEBUG("MISSION_WRITE_PARTIAL_LIST error: mission manager not idle.\n");
        return;
      }
      /* only allow update if already mission  onboard */
      if (mission_mgr.active_count > 0) {
        MAVLINK_DEBUG("MISSION_WRITE_PARTIAL_LIST error: no mission loaded yet.\n");
        return;
      }
      /* check if start_index greater or equal than list size*/
      if (mission_write_partial_list.start_index >= mission_mgr.active_count) {
        MAVLINK_DEBUG("MISSION_WRITE_PARTIAL_LIST error: start index greater or equal to mission count\n");
        return;
      }
      /* check if end_index greater or equal than list size and bigger or equal than start index*/
      if ((mission_write_partial_list.end_index >= mission_mgr.active_count) ||
            mission_write_partial_list.end_index < mission_write_partial_list.start_index) {
        MAVLINK_DEBUG("MISSION_WRITE_PARTIAL_LIST error: start index greater or equal to mission count\n");
        return;
      }

      /* valid initiation of waypoint write transaction, ask for first waypoint */
      MAVLINK_DEBUG("MISSION_WRITE_PARTIAL_LIST: Requesting waypoint with begin_index\n");
      mavlink_msg_mission_request_int_send(MAVLINK_COMM_0, msg->sysid, msg->compid, mission_write_partial_list.start_index, MAV_MISSION_TYPE_MISSION);
      MAVLinkSendMessage();

      mission_mgr.seq = mission_write_partial_list.start_index;
      mission_mgr.state = STATE_WAYPOINT_WRITE_TRANSACTION;

      // Register the timeout timer
      mavlink_mission_set_timer();
      break;
    }

    /* got MISSION_ITEM, update one waypoint if in valid transaction */
    case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
      mavlink_mission_item_int_t mission_item_int;
      mavlink_msg_mission_item_int_decode(msg, &mission_item_int);

      if (mission_item_int.target_system != mavlink_system.sysid) {
        return;
      }

      MAVLINK_DEBUG("Received MISSION_ITEM_INT message with seq %i and frame %i\n",
                    mission_item_int.seq, mission_item_int.frame);

      /* reject if seq number too high */
      if (mission_item_int.seq >= MISSION_ELEMENT_NB ||
          mission_item_int.seq >= mission_mgr.count) {
        MAVLINK_DEBUG("rejected MISSION_ITEM_INT command %i, seq %i because seq number is too high\n",
                      mission_item_int.command, mission_item_int.seq);
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                    MAV_MISSION_NO_SPACE, MAV_MISSION_TYPE_MISSION);
        MAVLinkSendMessage();
        mavlink_mission_cancel_timer();
        mission_mgr.state = STATE_IDLE;
        return;
      }

      /* reject if non supported command */
      if (mission_item_int.command != MAV_CMD_NAV_WAYPOINT &&
          mission_item_int.command != MAV_CMD_NAV_TAKEOFF &&
          mission_item_int.command != MAV_CMD_NAV_VTOL_TAKEOFF &&
          mission_item_int.command != MAV_CMD_NAV_LAND &&
          mission_item_int.command != MAV_CMD_DO_CHANGE_SPEED) {
        MAVLINK_DEBUG("rejected MISSION_ITEM_INT command %i, seq %i due to unsupported command\n",
                      mission_item_int.command, mission_item_int.seq);
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                    MAV_MISSION_UNSUPPORTED, MAV_MISSION_TYPE_MISSION);
        MAVLinkSendMessage();
        mavlink_mission_cancel_timer();
        mission_mgr.state = STATE_IDLE;
        return;
      }

      /* reject if non supported frame */
      if (mission_item_int.frame != MAV_FRAME_GLOBAL &&
          mission_item_int.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT && 
          mission_item_int.frame != MAV_FRAME_MISSION) {
        MAVLINK_DEBUG("rejected MISSION_ITEM_INT frame %i, seq %i due to unsupported frame\n",
                      mission_item_int.frame, mission_item_int.seq);
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                    MAV_MISSION_UNSUPPORTED_FRAME, MAV_MISSION_TYPE_MISSION);
        MAVLinkSendMessage();
        mavlink_mission_cancel_timer();
        mission_mgr.state = STATE_IDLE;
        return;
      }

      /* Only accept mission item in IDLE (update single waypoint) or in write transaction */
      if (mission_mgr.state != STATE_IDLE && mission_mgr.state != STATE_WAYPOINT_WRITE_TRANSACTION) {
        MAVLINK_DEBUG("got MISSION_ITEM_INT while not in waypoint write transaction or idle\n");
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                    MAV_MISSION_ERROR, MAV_MISSION_TYPE_MISSION);
        MAVLinkSendMessage();
        return;
      }
      /* If in write transaction, only handle mission item if correct sequence */
      if (mission_mgr.state == STATE_WAYPOINT_WRITE_TRANSACTION && mission_item_int.seq != mission_mgr.seq) {
        MAVLINK_DEBUG("MISSION_ITEM_INT, got waypoint seq %i, but requested %i\n",
                      mission_item_int.seq, mission_mgr.seq);
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                    MAV_MISSION_ERROR, MAV_MISSION_TYPE_MISSION);
        MAVLinkSendMessage();
        return;
      }

      /* Write item to mission_mgr */
      mission_mgr.standby_mission_items[mission_mgr.seq].seq = mission_item_int.seq;
      mission_mgr.standby_mission_items[mission_mgr.seq].frame = mission_item_int.frame;
      mission_mgr.standby_mission_items[mission_mgr.seq].cmd = mission_item_int.command;
      mission_mgr.standby_mission_items[mission_mgr.seq].current = mission_item_int.current;
      mission_mgr.standby_mission_items[mission_mgr.seq].autocontinue = mission_item_int.autocontinue;
      mission_mgr.standby_mission_items[mission_mgr.seq].x = mission_item_int.x;
      mission_mgr.standby_mission_items[mission_mgr.seq].y = mission_item_int.y;
      mission_mgr.standby_mission_items[mission_mgr.seq].z = mission_item_int.z;
      
      // acknowledge transmission single waypoint update
      if (mission_mgr.state == STATE_IDLE) {
        MAVLINK_DEBUG("Acknowledge single waypoint update\n");
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                     MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        MAVLinkSendMessage();
        mavlink_mission_cancel_timer();
      }
      // or end of transaction
      else if (mission_item_int.seq == mission_mgr.count -1) {
        // Check validaty of mission
        MAVLINK_DEBUG("Acknowledging end of waypoint write transaction\n");
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                    MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        MAVLinkSendMessage();
        mavlink_mission_cancel_timer();
       
        mavlink_mission_set_active();
        
        mission_mgr.state = STATE_IDLE;
      }
      // or request next waypoint if still in middle of transaction
      else {
        MAVLINK_DEBUG("Requesting waypoint %i\n", mission_item_int.seq + 1);
        mavlink_msg_mission_request_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                         mission_item_int.seq + 1, MAV_MISSION_TYPE_MISSION);
        MAVLinkSendMessage();
        mission_mgr.seq = mission_item_int.seq + 1;
        mavlink_mission_set_timer();
      }
      break;
    }

    default:
      break;
  }
}

extern void mavlink_waypoint_handler(mavlink_mission_item *mission_item)
{
  switch (mission_item->cmd) {
    case MAV_CMD_DO_CHANGE_SPEED: {
      if (mission_item->current) {
        current_bool = 1;
      }
      break;
    }
    case MAV_CMD_NAV_WAYPOINT: {
      if (mission_item->frame == MAV_FRAME_GLOBAL) {
        MAVLINK_DEBUG("MISSION_ITEM_INT, global wp: lat=%i, lon=%i, alt=%f\n",
                        mission_item->x, mission_item->y, mission_item->z);
        MAVLINK_DEBUG("Current set: %i\n", mission_item->current);

        struct LlaCoor_i lla;
        mavlink_lla_of_global(mission_item, &lla);

        struct _mission_element me;
        me.type = MissionWP;

        // if there is no valid local coordinate, do not insert mission element
        if (mission_point_of_lla(&me.element.mission_wp.wp, &lla)) { 
          me.duration = -1;
          me.index = mission_item->seq;

          enum MissionInsertMode insert = (enum MissionInsertMode)(Append);

          // Replace and start from current if current set
          if (mission_item->current || current_bool) {
            insert = (enum MissionInsertMode)(ReplaceAll);
            current_bool = 0;
          }

          int r = mission_insert(insert, &me);
          MAVLINK_DEBUG("mission_insert response %i\n", r);
          (void)r;
          
          #if PERIODIC_TELEMETRY
          DOWNLINK_SEND_MISSION_ITEM(DefaultChannel, DefaultDevice,
                                      &mission_mgr.count,
                                      &mission_item->seq,
                                      &mission_item->cmd,
                                      &lla.lat,
                                      &lla.lon,
                                      &mission_item->z);
          #endif

        } else { 
          return; 
        }
      } else if (mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT) {
        MAVLINK_DEBUG("MISSION_ITEM_INT, global_rel_alt wp: lat=%i, lon=%i, relative alt=%f\n",
                        mission_item->x, mission_item->y, mission_item->z);

          struct LlaCoor_i lla;
          mavlink_lla_of_global_relative_alt(mission_item, &lla);

          struct _mission_element me;
          me.type = MissionWP;

          // if there is no valid local coordinate, do not insert mission element
          if (mission_point_of_lla(&me.element.mission_wp.wp, &lla)) { 
            me.duration = -1;
            me.index = mission_item->seq;

            enum MissionInsertMode insert = (enum MissionInsertMode)(Append);

            // Replace and start from current if current set
            if (mission_item->current || current_bool) {
              insert = (enum MissionInsertMode)(ReplaceAll);
              current_bool = 0;
            }

            int r = mission_insert(insert, &me);
            MAVLINK_DEBUG("mission_insert response %i\n", r);
            (void)r;

            #if PERIODIC_TELEMETRY
            DOWNLINK_SEND_MISSION_ITEM(DefaultChannel, DefaultDevice,
                                        &mission_mgr.count,
                                        &mission_item->seq,
                                        &mission_item->cmd,
                                        &lla.lat,
                                        &lla.lon,
                                        &mission_item->z);
            #endif
          } else { 
            return; 
          }
      } else {
        MAVLINK_DEBUG("No handler for MISSION_ITEM_INT with frame %i\n", mission_item->frame);
        return;
      }
      break;
    }
    case MAV_CMD_NAV_LOITER_UNLIM: {
      break;
    }
    case MAV_CMD_NAV_LOITER_TIME: {
      break;
    }
    case MAV_CMD_NAV_RETURN_TO_LAUNCH: {
      break;
    }
    case MAV_CMD_NAV_LAND: {
      // TODO: Call custom navigation function (defined in airframe?)
      break;
    }
    case MAV_CMD_NAV_TAKEOFF: {
      // TODO: Call custom navigation function (defined in airframe?)
      MAVLINK_DEBUG("MISSION_ITEM_INT MAV_CMD_NAV_TAKEOFF received\n");
      struct _mission_element me;
      me.type = MissionCustom;
      me.element.mission_custom.reg = mission_get_registered("TO");
      if (me.element.mission_custom.reg == NULL) {
        MAVLINK_DEBUG("NO TO Custom mission item defined\n");
        return; // unknown type
      }
      me.duration = -1;
      me.index = mission_item->seq;

      enum MissionInsertMode insert = (enum MissionInsertMode)(Append);

      int r = mission_insert(insert, &me);
      MAVLINK_DEBUG("mission_insert response %i\n", r);
      (void)r;
      break;
    }
    case MAV_CMD_NAV_VTOL_TAKEOFF: {
      // TODO: Call custom navigation function (defined in airframe?)
      MAVLINK_DEBUG("MISSION_ITEM_INT MAV_CMD_NAV_VTOL_TAKEOFF received\n");
      struct _mission_element me;
      me.type = MissionCustom;
      me.element.mission_custom.reg = mission_get_registered("TO");
      if (me.element.mission_custom.reg == NULL) {
        MAVLINK_DEBUG("NO TO Custom mission item defined\n");
        return; // unknown type
      }
      me.duration = -1;
      me.index = mission_item->seq;

      enum MissionInsertMode insert = (enum MissionInsertMode)(Append);

      int r = mission_insert(insert, &me);
      MAVLINK_DEBUG("mission_insert response %i\n", r);
      (void)r;
      break;
    }
    default: {
      MAVLINK_DEBUG("MAVLINK_CMD not supported");
    }
  }
}

void mavlink_lla_of_global(mavlink_mission_item *mission_item, struct LlaCoor_i *lla)
{
  lla->lat = mission_item->x; // lattitude in degrees*1e7
  lla->lon = mission_item->y; // longitude in degrees*1e7
  lla->alt = mission_item->z * 1e3; // altitude in millimeters
}

void mavlink_lla_of_global_relative_alt(mavlink_mission_item *mission_item, struct LlaCoor_i *lla)
{
  lla->lat = mission_item->x; // lattitude in degrees*1e7
  lla->lon = mission_item->y; // longitude in degrees*1e7
  lla->alt = state.ned_origin_i.hmsl + mission_item->z * 1e3; // altitude in millimeters
}
