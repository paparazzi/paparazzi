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

#include "modules/datalink/mavlink.h"
#include "modules/datalink/missionlib/mission_manager.h"

static void mavlink_send_wp_count(void)
{
    mavlink_message_t msg;
    mavlink_mission_count_t wp_count;	
    wp_count.target_system = mission_mgr.rem_sysid;
    wp_count.target_component = mission_mgr.rem_compid;
    wp_count.count = NB_WAYPOINT; // From the generated flight plan
	
    mavlink_msg_mission_count_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &wp_count); // encode the block count message

#ifdef MAVLINK_FLAG_DEBUG
        printf("Sent WP_COUNT message\n");
#endif
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

#ifdef MAVLINK_FLAG_DEBUG
        printf("Sent MISSION_ITEM message\n");
#endif
        mavlink_send_message(&msg);
    }
    else {
#ifdef MAVLINK_FLAG_DEBUG
        perror("Wp index out of bounds\n");
#else
        // TODO: Fix for stm32 etc.
#endif
    }
}

void mavlink_wp_init(void)
{
    // Store the waypoints in a mission item    
    if (NB_WAYPOINT > 0) {
        for (uint8_t i = 0; i < NB_WAYPOINT; i++) {
            mavlink_mission_item_t mission_item;
            mission_item.x = waypoints[i].lla.lat; // lattitude
            mission_item.y = waypoints[i].lla.lon; // longtitude
            mission_item.z = waypoints[i].lla.alt; // altitude
            mission_item.seq = i;
            mission_mgr.waypoints[i] = mission_item;
        }
    } else {
#ifdef MAVLINK_FLAG_DEBUG
        perror("The waypoint array is empty\n");
#else
        // TODO: Fix for stm32 etc.
#endif
    }
}

void mavlink_wp_message_handler(const mavlink_message_t* msg)
{
    switch(msg->msgid)
    { 
    	case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        {
#ifdef MAVLINK_FLAG_DEBUG
            printf("Received MISSION_REQUEST_LIST message\n");
#endif
            mavlink_mission_request_list_t mission_request_list_msg;
            mavlink_msg_mission_request_list_decode(msg, &mission_request_list_msg); // Cast the incoming message to a mission_request_list_msg
            if(mission_request_list_msg.target_system == mavlink_system.sysid) {				
                if (mission_mgr.state == STATE_IDLE) {
                    if (NB_WAYPOINT > 0) {
                        mission_mgr.state = STATE_SEND_LIST;
                        mission_mgr.seq = 0;
                        mission_mgr.rem_sysid = msg->sysid;
                        mission_mgr.rem_compid = msg->compid;
                    }
                    mavlink_send_wp_count();

					mission_mgr.timer_id = sys_time_register_timer(MAVLINK_TIMEOUT, &timer_cb); // Register the timeout timer (it is continuous so it needs to be cancelled after triggering)
                } else {
                    // TODO: Handle case when the state is not IDLE
                }
            } else {
				// TODO: Handle remote system id mismatch
			}

            break;
        }

        case MAVLINK_MSG_ID_MISSION_REQUEST:
        {
#ifdef MAVLINK_FLAG_DEBUG
            printf("Received MISSION_REQUEST message\n");
#endif
        	mavlink_mission_request_t mission_request_msg;
        	mavlink_msg_mission_request_decode(msg, &mission_request_msg); // Cast the incoming message to a mission_request_msg
        	if(mission_request_msg.target_system == mavlink_system.sysid) {	
        		if ((mission_mgr.state == STATE_SEND_LIST && mission_request_msg.seq == 0) || // Send the first waypoint
        				(mission_mgr.state == STATE_SEND_ITEM && (mission_request_msg.seq == mission_mgr.seq || // Send the current waypoint again
                                                                  mission_request_msg.seq == mission_mgr.seq+1))) { // Send the next waypoiny
        			sys_time_cancel_timer(mission_mgr.timer_id); // Cancel the timeout timer
        			
        			mission_mgr.state = STATE_SEND_ITEM;
        			mission_mgr.seq = mission_request_msg.seq;

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

        case MAVLINK_MSG_ID_MISSION_ITEM:
        {
#ifdef MAVLINK_FLAG_DEBUG
            printf("Received BLOCK_ITEM message\n");
#endif
            mavlink_mission_item_t mission_item_msg;
            mavlink_msg_mission_item_decode(msg, &mission_item_msg); // Cast the incoming message to a mission_item_msg
            if(mission_item_msg.target_system == mavlink_system.sysid) {              
                if (mission_mgr.state == STATE_IDLE) { // Only handle incoming mission item messages if there a not currently being sent
                    struct LlaCoor_i lla;
                    lla.lat = mission_item_msg.x; // lattitude
                    lla.lon = mission_item_msg.y; // longitude
                    lla.alt = mission_item_msg.z; // altitude
                    nav_set_waypoint_latlon(mission_item_msg.seq, &lla); // move the waypoint with the id equal to seq 
                }
            }
        }
    }
}