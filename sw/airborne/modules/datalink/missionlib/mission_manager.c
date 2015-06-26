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
#include "modules/datalink/mission_manager.h"

#include "modules/datalink/mavlink.h"

void mavlink_mission_message_handler(const mavlink_message_t* msg)
{
  switch(msg->msgid)
  { 
    case MAVLINK_MSG_ID_MISSION_ACK:
    {
#ifdef MAVLINK_FLAG_DEBUG
        printf("Received MISSION_ACK message\n");
#endif
      sys_time_cancel_timer(block_mgr.timer_id); // Cancel the timeout timer
      block_mgr.current_state = STATE_IDLE;
    }
  }
}