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

/** @file modules/datalink/missionlib/mission_manager.c
*  @brief Common functions used within the mission library
*/

// Include own header
#include "modules/datalink/missionlib/mission_manager.h"

#include "modules/datalink/mavlink.h"
//#include "modules/datalink/missionlib/blocks.h"
#include "modules/datalink/missionlib/waypoints.h"

// include mavlink headers, but ignore some warnings
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wswitch-default"
#include "mavlink/ardupilotmega/mavlink.h"
#pragma GCC diagnostic pop

#include "generated/flight_plan.h"

#include "mcu_periph/sys_time.h"


void mavlink_mission_init(mavlink_mission_mgr *mgr)
{
  mgr->count = 0;
  mgr->active_count = 0;
  mgr->mission_state = MISSION_STATE_NO_MISSION;
  mgr->seq = 0;
  mgr->end_index = 0;
  mgr->timer_id = -1;
}

void mavlink_mission_set_timer(void)
{
  if (mission_mgr.timer_id < 0) {
    mission_mgr.timer_id = sys_time_register_timer(MAVLINK_TIMEOUT, NULL);
  }
  else {
    sys_time_update_timer(mission_mgr.timer_id, MAVLINK_TIMEOUT);
  }
}

void mavlink_mission_cancel_timer(void)
{
  if (mission_mgr.timer_id >= 0) {
    sys_time_cancel_timer(mission_mgr.timer_id);
  }
  mission_mgr.timer_id = -1;
}

void mavlink_mission_message_handler(const mavlink_message_t *msg)
{
  // mavlink_block_message_handler(msg);

  mavlink_wp_message_handler(msg);

  if (msg->msgid == MAVLINK_MSG_ID_MISSION_ACK) {
    MAVLINK_DEBUG("Received MISSION_ACK message\n");
    mavlink_mission_cancel_timer();
    mission_mgr.state = STATE_IDLE;
    MAVLINK_DEBUG("State: %d\n", mission_mgr.state);
  }
}

/// update current block and send if changed
void mavlink_mission_periodic(void)
{
  // FIXME: really use the SCRIPT_ITEM message to indicate current block?
  if (mission_mgr.current_block != nav_block) {
    mission_mgr.current_block = nav_block;
    // mavlink_msg_script_current_send(MAVLINK_COMM_0, nav_block);
    // MAVLinkSendMessage();
  }
  // check if we had a timeout on a transaction
  if (sys_time_check_and_ack_timer(mission_mgr.timer_id)) {
    mavlink_mission_cancel_timer();
    mission_mgr.state = STATE_IDLE;
    mission_mgr.seq = 0;
    MAVLINK_DEBUG("Warning: Mavlink mission request timed out!\n");
  }
}

void mavlink_send_mission_ack(void)
{
  mavlink_msg_mission_ack_send(MAVLINK_COMM_0,  mission_mgr.rem_sysid, mission_mgr.rem_compid,
                               MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
  MAVLinkSendMessage();
  MAVLINK_DEBUG("Sent MISSION_ACK message\n");
}

enum MAV_MISSION_RESULT mavlink_mission_check_validity(void)
{
  // Check cmd and frame of mission items in mission
  for (uint8_t i = 0; i < mission_mgr.count; i++) {
    if (mission_mgr.standby_mission_items[i].cmd != MAV_CMD_NAV_WAYPOINT ||
        mission_mgr.standby_mission_items[i].cmd != MAV_CMD_NAV_LOITER_TIME) {
          return MAV_MISSION_UNSUPPORTED;
    }
    if (mission_mgr.standby_mission_items[i].frame != MAV_FRAME_GLOBAL || 
        mission_mgr.standby_mission_items[i].frame != MAV_FRAME_GLOBAL_RELATIVE_ALT) {
          return MAV_MISSION_UNSUPPORTED_FRAME;
    }  
  }
  return MAV_MISSION_ACCEPTED;
}


void mavlink_mission_set_active(void) 
{
  for (uint8_t i = 0; i < mission_mgr.count; i++) {
    mission_mgr.active_mission_items[i].seq = mission_mgr.standby_mission_items[i].seq;
    mission_mgr.active_mission_items[i].frame = mission_mgr.standby_mission_items[i].frame;
    mission_mgr.active_mission_items[i].cmd = mission_mgr.standby_mission_items[i].cmd;
    mission_mgr.active_mission_items[i].current = mission_mgr.standby_mission_items[i].current;
    mission_mgr.active_mission_items[i].autocontinue = mission_mgr.standby_mission_items[i].autocontinue;
    mission_mgr.active_mission_items[i].x = mission_mgr.standby_mission_items[i].x;
    mission_mgr.active_mission_items[i].y = mission_mgr.standby_mission_items[i].y;
    mission_mgr.active_mission_items[i].z = mission_mgr.standby_mission_items[i].z;
    // Process waypoint
    mavlink_waypoint_handler(&mission_mgr.active_mission_items[i]);
  }
  mission_mgr.active_count = mission_mgr.count;

  // TODO: set index correct in mission module
  
  MAVLINK_DEBUG("Mission with count %i activated", mission_mgr.count);
}
