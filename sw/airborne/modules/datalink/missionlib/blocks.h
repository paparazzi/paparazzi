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

#ifndef MISSIONLIB_BLOCKS_H
#define MISSIONLIB_BLOCKS_H

// Disable auto-data structures
#ifndef MAVLINK_NO_DATA
#define MAVLINK_NO_DATA
#endif

#ifndef MAVLINK_TIMEOUT
#define MAVLINK_TIMEOUT 15 // As in MAVLink waypoint convention
#endif

#include "generated/flight_plan.h"
#include "mavlink/paparazzi/mavlink.h"

/** 
 * State machine 
 */
enum MAVLINK_BLOCK_STATES {
	STATE_IDLE = 0, 
	STATE_SEND_BLOCK_LIST, 
	STATE_SEND_BLOCK
};

/** 
 * Block storage struct
 */ 
#ifndef MAVLINK_MAX_BLOCK_COUNT
#define MAVLINK_MAX_BLOCK_COUNT NB_BLOCK
#endif
struct mavlink_block_mgr {
	enum MAVLINK_BLOCK_STATES current_state; // The current state of the block handler
	uint16_t current_block; // Counter that holds the index of the current block
	uint8_t rem_sysid; // Remote system id
	uint8_t rem_compid; // Remote component id
	int timer_id; // Timer id
};

typedef struct mavlink_block_mgr mavlink_block_mgr;

extern mavlink_block_mgr block_mgr;

extern void mavlink_block_init(mavlink_block_mgr* block_mgr);
extern void mavlink_block_message_handler(const mavlink_message_t* msg);

#endif // MISSIONLIB_BLOCKS_H