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

#include "modules/datalink/missionlib/blocks.h"
#include "modules/datalink/missionlib/mission_manager.h"
#include "modules/datalink/mavlink.h"

// include mavlink headers, but ignore some warnings
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#include "mavlink/paparazzi/mavlink.h"
#pragma GCC diagnostic pop

#include "modules/nav/common_flight_plan.h"
#include "generated/flight_plan.h"

static void mavlink_send_block_count(void)
{
  mavlink_msg_script_count_send(MAVLINK_COMM_0, mission_mgr.rem_sysid, mission_mgr.rem_compid, NB_BLOCK);
  MAVLinkSendMessage();
  MAVLINK_DEBUG("Sent BLOCK_COUNT message: count %i\n", NB_BLOCK);
}

void mavlink_send_block(uint16_t seq)
{
  if (seq < NB_BLOCK) { // Due to indexing
    static const char *blocks[] = FP_BLOCKS;
    char block_name[50];
    strncpy(block_name, blocks[seq], 49); // String containing the name of the block
    // make sure the string is null terminated if longer than 49chars
    block_name[49] = 0;
    mavlink_msg_script_item_send(MAVLINK_COMM_0, mission_mgr.rem_sysid, mission_mgr.rem_compid,
                                 seq, block_name);
    MAVLinkSendMessage();
    MAVLINK_DEBUG("Sent BLOCK_ITEM message: seq %i, name %s\n", seq, block_name);
  } else {
    MAVLINK_DEBUG("ERROR: Block index %i out of bounds\n", seq);
  }
}

void mavlink_block_message_handler(const mavlink_message_t *msg)
{
  switch (msg->msgid) {
    /* request for script/block list, answer with number of blocks */
    case MAVLINK_MSG_ID_SCRIPT_REQUEST_LIST: {
      MAVLINK_DEBUG("Received BLOCK_REQUEST_LIST message\n");
      mavlink_script_request_list_t block_request_list_msg;
      mavlink_msg_script_request_list_decode(msg, &block_request_list_msg);
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

          // Register the timeout timer (it is continuous so it needs to be cancelled after triggering)
          mavlink_mission_set_timer();
        } else {
          // TODO: Handle case when the state is not IDLE
        }
      } else {
        // TODO: Handle remote system id mismatch
      }

      break;
    }

    /* request script/block, answer with SCRIPT_ITEM (block) */
    case MAVLINK_MSG_ID_SCRIPT_REQUEST: {
      MAVLINK_DEBUG("Received BLOCK_REQUEST message\n");
      mavlink_script_request_t block_request_msg;
      mavlink_msg_script_request_decode(msg, &block_request_msg);
      if (block_request_msg.target_system == mavlink_system.sysid) {
        // Handle only cases in which the entire list is request, the block is sent again,
        // or the next block was sent
        if ((mission_mgr.state == STATE_SEND_LIST && block_request_msg.seq == 0) ||
            (mission_mgr.state == STATE_SEND_ITEM && (block_request_msg.seq == mission_mgr.seq
                || block_request_msg.seq == mission_mgr.seq + 1))) {
          // Cancel the timeout timer
          mavlink_mission_cancel_timer();

          mission_mgr.state = STATE_SEND_ITEM;
          MAVLINK_DEBUG("State: %d\n", mission_mgr.state);
          mission_mgr.seq = block_request_msg.seq;

          mavlink_send_block(mission_mgr.seq);

          // Register the timeout timer
          mavlink_mission_set_timer();
        } else {
          // TODO: Handle cases for which the above condition does not hold
        }
      } else {
        // TODO: Handle remote system id mismatch
      }

      break;
    }

    /* got script item, change block */
    case MAVLINK_MSG_ID_SCRIPT_ITEM: {
      MAVLINK_DEBUG("Received BLOCK_ITEM message\n");
      mavlink_script_item_t block_item_msg;
      mavlink_msg_script_item_decode(msg, &block_item_msg);
      if (block_item_msg.target_system == mavlink_system.sysid) {
        // Only handle incoming block item messages if they a not currently being sent
        if (mission_mgr.state == STATE_IDLE) {
          nav_goto_block((uint8_t)block_item_msg.seq); // Set the current block
        }
      }
    }

    default:
      break;
  }
}
