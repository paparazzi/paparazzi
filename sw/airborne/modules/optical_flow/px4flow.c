/*
 * Copyright (C) 2013 Gautier Hattenberger
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

/** @file modules/optical_flow/px4flow.c
 *  @brief driver for the optical flow sensor PX4FLOW
 *
 *  Sensor from the PIXHAWK project
 */

#include "modules/optical_flow/px4flow.h"
#include "modules/datalink/mavlink_decoder.h"
#include <string.h>

struct mavlink_optical_flow optical_flow;
bool_t optical_flow_available;

// message ID in Mavlink (v1.0)
#define MAVLINK_OPTICAL_FLOW_MSG_ID 100
// size of the structure
#define MAVLINK_OPTICAL_FLOW_LEN 26

// request struct for mavlink decoder
struct mavlink_msg_req req;

// callback function on message reception
static void decode_optical_flow_msg(struct mavlink_message *msg __attribute__((unused)))
{
  optical_flow_available = TRUE;
}

/** Initialization function
 */
void px4flow_init(void)
{
  optical_flow_available = FALSE;

  // register a mavlink message
  req.msg_id = MAVLINK_OPTICAL_FLOW_MSG_ID;
  req.callback = decode_optical_flow_msg;
  req.msg.payload = (uint8_t *)(&optical_flow);
  mavlink_register_msg(&mavlink_tp, &req);

}

// Messages
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

/** Downlink message for debug
 */
void px4flow_downlink(void)
{
  DOWNLINK_SEND_PX4FLOW(DefaultChannel, DefaultDevice,
                        &optical_flow.sensor_id,
                        &optical_flow.flow_x,
                        &optical_flow.flow_y,
                        &optical_flow.flow_comp_m_x,
                        &optical_flow.flow_comp_m_y,
                        &optical_flow.quality,
                        &optical_flow.ground_distance);
}

