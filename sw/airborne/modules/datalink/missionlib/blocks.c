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

/** @file modules/datalink/missionlib/blocks.c
 *  @brief PPRZ specific mission block implementation
 */

// Include own header
#include "modules/datalink/missionlib/blocks.h"

#include <stdio.h>
#include <string.h>

#include "modules/datalink/mavlink.h"
#include "modules/datalink/missionlib/mission_manager.h"

#include "subsystems/navigation/common_flight_plan.h"

static void mavlink_send_block_count(void)
{
  mavlink_message_t msg;
  mavlink_script_count_t block_count;
  block_count.target_system = mission_mgr.rem_sysid;
  block_count.target_component = mission_mgr.rem_compid;
  block_count.count = NB_BLOCK; // From the generated flight plan

  mavlink_msg_script_count_encode(mavlink_system.sysid, mavlink_system.compid, &msg,
                                  &block_count); // encode the block count message

  MAVLINK_DEBUG("Sent BLOCK_COUNT message\n");
  mavlink_send_message(&msg);
}

static void mavlink_send_block(uint16_t seq)
{
  if (seq < NB_BLOCK) { // Due to indexing
    mavlink_message_t msg;
    mavlink_script_item_t block_item;
    block_item.seq = seq;
    char *blocks[] = FP_BLOCKS;
    block_item.len = (uint8_t)strlen(blocks[seq]); // Length of the block name
    strcpy(block_item.name, blocks[seq]); // String containing the name of the block
    block_item.target_system = mission_mgr.rem_sysid;
    block_item.target_component = mission_mgr.rem_compid;

    mavlink_msg_script_item_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &block_item);

    MAVLINK_DEBUG("Sent BLOCK_ITEM message\n");
    mavlink_send_message(&msg);
  } else {
    MAVLINK_DEBUG("ERROR: Block index out of bounds\n");
  }
}

void mavlink_block_init(void)
{
}

void mavlink_block_cb(uint16_t current_block)
{
  mavlink_send_block(current_block); // send the current block seq

  mission_mgr.timer_id = sys_time_register_timer(MAVLINK_TIMEOUT, &timer_cb); // wait for ack
}

void mavlink_block_message_handler(const mavlink_message_t *msg)
{
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_SCRIPT_REQUEST_LIST: {
      MAVLINK_DEBUG("Received BLOCK_REQUEST_LIST message\n");
      mavlink_script_request_list_t block_request_list_msg;
      mavlink_msg_script_request_list_decode(msg,
                                             &block_request_list_msg); // Cast the incoming message to a block_request_list_msg
      if (block_request_list_msg.target_system == mavlink_system.sysid) {
        if (mission_mgr.state == STATE_IDLE) {
          if (NB_BLOCK > 0) {
            mission_mgr.state = STATE_SEND_LIST;
            MAVLINK_DEBUG("State: %d\n", mission_mgr.state);
            mission_mgr.seq = 0;
            mission_mgr.rem_sysid = msg->sysid;
            mission_mgr.rem_compid = msg->compid;
          }
          mavlink_send_block_count();

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

    case MAVLINK_MSG_ID_SCRIPT_REQUEST: {
      MAVLINK_DEBUG("Received BLOCK_REQUEST message\n");
      mavlink_script_request_t block_request_msg;
      mavlink_msg_script_request_decode(msg, &block_request_msg); // Cast the incoming message to a block_request_msg
      if (block_request_msg.target_system == mavlink_system.sysid) {
        // Handle only cases in which the entire list is request, the block is sent again, or the next block was sent
        if ((mission_mgr.state == STATE_SEND_LIST && block_request_msg.seq == 0) ||
            (mission_mgr.state == STATE_SEND_ITEM && (block_request_msg.seq == mission_mgr.seq
                || block_request_msg.seq == mission_mgr.seq + 1))) {
          sys_time_cancel_timer(mission_mgr.timer_id); // Cancel the timeout timer

          mission_mgr.state = STATE_SEND_ITEM;
          MAVLINK_DEBUG("State: %d\n", mission_mgr.state);
          mission_mgr.seq = block_request_msg.seq;

          mavlink_send_block(mission_mgr.seq);

          mission_mgr.timer_id = sys_time_register_timer(MAVLINK_TIMEOUT, &timer_cb); // Register the timeout timer
        } else {
          // TODO: Handle cases for which the above condition does not hold
        }
      } else {
        // TODO: Handle remote system id mismatch
      }

      break;
    }

    case MAVLINK_MSG_ID_SCRIPT_ITEM: {
      MAVLINK_DEBUG("Received BLOCK_ITEM message\n");
      mavlink_script_item_t block_item_msg;
      mavlink_msg_script_item_decode(msg, &block_item_msg); // Cast the incoming message to a block_item_msg
      if (block_item_msg.target_system == mavlink_system.sysid) {
        if (mission_mgr.state == STATE_IDLE) { // Only handle incoming block item messages if there a not currently being sent
          nav_goto_block((uint8_t)block_item_msg.seq); // Set the current block
        }
      }
    }
  }
}
