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

#include <mavlink/mavlink_types.h>
#include "modules/mission/mission_common.h"

#ifndef MAVLINK_TIMEOUT
#define MAVLINK_TIMEOUT 15 // as in MAVLink waypoint convention
#endif

/// State machine
enum MAVLINK_MISSION_MGR_STATES {
  STATE_IDLE = 0,
  STATE_SEND_LIST,
  STATE_SEND_ITEM,
  STATE_WAYPOINT_WRITE_TRANSACTION
};

struct mavlink_mission_item {
  uint16_t seq;
  uint8_t frame;
  uint16_t cmd;
  uint8_t current;
  uint8_t autocontinue;
  int32_t x;
  int32_t y;
  float z;
};

typedef struct mavlink_mission_item mavlink_mission_item;

struct mavlink_mission_mgr {
  uint8_t current_block; // Counter that holds the index of the current block
  uint8_t count; // Count of mission elements
  uint8_t active_count; // Count of active mission elements
  enum MAVLINK_MISSION_MGR_STATES state; // The current state of the mission handler
  uint8_t mission_state; // The current MISSION_STATE defined by common mavlink
  uint16_t seq; // Sequence id (position of the current item on the list)
  uint16_t end_index; // Index of final element in a (partial) upload transaction
  uint8_t rem_sysid; // Remote system id
  uint8_t rem_compid; // Remote component id
  int timer_id; // Timer id
  mavlink_mission_item active_mission_items[MISSION_ELEMENT_NB]; // The activated mission items
  mavlink_mission_item standby_mission_items[MISSION_ELEMENT_NB]; // The standby mission items (used for fp upload)
};

typedef struct mavlink_mission_mgr mavlink_mission_mgr;

extern mavlink_mission_mgr mission_mgr;


extern void mavlink_mission_init(mavlink_mission_mgr *mgr);
extern void mavlink_mission_message_handler(const mavlink_message_t *msg);
extern void mavlink_mission_periodic(void);

extern void mavlink_send_mission_ack(void);

extern void mavlink_mission_set_timer(void);
extern void mavlink_mission_cancel_timer(void);

extern enum MAV_MISSION_RESULT mavlink_mission_check_validity(void);
extern void mavlink_mission_set_active(void);

#endif
