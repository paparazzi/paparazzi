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

/** @file modules/datalink/missionlib/mission_manager.h
*  @brief Common functions used within the mission library, blocks and
*         waypoints cannot be send simultaneously (which should not
*         matter)
*/

#ifndef MISSIONLIB_COMMON_H
#define MISSIONLIB_COMMON_H

#include <stdio.h>

// #include "firmwares/rotorcraft/navigation.h"
#include "mcu_periph/sys_time.h"

#include "generated/flight_plan.h"
#include "mavlink/paparazzi/mavlink.h"

#ifndef MAVLINK_TIMEOUT
#define MAVLINK_TIMEOUT 15 // as in MAVLink waypoint convention
#endif

// State machine
enum MAVLINK_MISSION_MGR_STATES {
  STATE_IDLE = 0,
  STATE_SEND_LIST,
  STATE_SEND_ITEM,
  STATE_WRITE_ITEM, // only for updating waypoints
};

struct mavlink_mission_mgr {
  mavlink_mission_item_t waypoints[NB_WAYPOINT]; // Array containing the waypoints in global coordinates
  uint8_t current_block; // Counter that holds the index of the current block
  enum MAVLINK_MISSION_MGR_STATES state; // The current state of the mission handler
  uint16_t seq; // Sequence id (position of the current item on the list)
  uint8_t rem_sysid; // Remote system id
  uint8_t rem_compid; // Remote component id
  int timer_id; // Timer id
};

typedef struct mavlink_mission_mgr mavlink_mission_mgr;

extern mavlink_mission_mgr mission_mgr;

// Timer callback function
static inline void timer_cb(uint8_t id)
{
  sys_time_cancel_timer(id); // Cancel the timer that triggered the timeout event
  mission_mgr.state = STATE_IDLE;
#if MAVLINK_FLAG_DEBUG
  perror("Request timed out!");
#else
  // TODO: Fix for stm32 etc.
#endif
  // TODO: Handle timeout retries, for now just assume no retries
}

static inline void sendMissionAck()
{
  mavlink_message_t msg;
  mavlink_mission_ack_t mission_ack;
  mission_ack.target_system = mission_mgr.rem_sysid;
  mission_ack.target_component = mission_mgr.rem_compid;
  mission_ack.type = MAV_MISSION_ACCEPTED;
  mavlink_msg_mission_ack_encode(mavlink_system.sysid, mavlink_system.compid, &msg,
                                 &mission_ack); // encode the ack message
#if MAVLINK_FLAG_DEBUG_EVENT
  printf("Sent MISSION_ACK message\n");
#endif
  mavlink_send_message(&msg);
}

extern void mavlink_mission_init(mavlink_mission_mgr *mission_mgr);
extern void mavlink_mission_message_handler(const mavlink_message_t *msg);

#endif