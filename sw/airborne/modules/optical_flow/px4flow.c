/*
 * Copyright (C) 2013 Gautier Hattenberger
 * 2016 Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
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
#include "subsystems/abi.h"

// Messages
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

struct mavlink_heartbeat heartbeat;
struct mavlink_optical_flow optical_flow;
struct mavlink_optical_flow_rad optical_flow_rad;

// message ID in Mavlink (v1.0)
#define MAVLINK_OPTICAL_HEARTBEAT_MSG_ID 0
#define MAVLINK_OPTICAL_FLOW_MSG_ID 100
#define MAVLINK_OPTICAL_FLOW_RAD_MSG_ID 106

/*
 * acceptable quality of optical flow (0-min,1-max)
 * higher threshold means fewer measurements will be available
 * but they will be more precise. Generally it is probably a better
 * idea to have lower threshold, and propagate the noise (1-quality)
 * to the INS
 */
#ifndef PX4FLOW_QUALITY_THRESHOLD
#define PX4FLOW_QUALITY_THRESHOLD 0.1
#endif

// request structs for mavlink decoder
struct mavlink_msg_req req_flow;
struct mavlink_msg_req req_flow_rad;
struct mavlink_msg_req req_heartbeat;


// callback function on message reception
static void decode_optical_flow_msg(struct mavlink_message *msg __attribute__((unused)))
{
  static float quality = 0;
  static float noise = 0;
  quality = ((float)optical_flow.quality)/255.0;
  noise = 1-quality;

  if (quality > PX4FLOW_QUALITY_THRESHOLD) {
    // flip the axis (if the PX4FLOW is mounted as shown in
    // https://pixhawk.org/modules/px4flow
    AbiSendMsgVELOCITY_ESTIMATE(PX4FLOW_VELOCITY_ID,
                                optical_flow.time_usec,
                                optical_flow.flow_comp_m_x,
                                optical_flow.flow_comp_m_y,
                                0.0f,
                                noise);
  }

  // positive distance means it's known/valid
  if (optical_flow.ground_distance > 0) {
    AbiSendMsgAGL(AGL_SONAR_PX4FLOW_ID, optical_flow.ground_distance);
  }
}

/**
 * According to https://pixhawk.org/modules/px4flow the PX4flow module outputs
 * OPTICAL_FLOW_RAD message as well as OPTICAL_FLOW message.
 * However, on my camera only OPTICAL_FLOW is received. Maybe different firmwares send
 * different messages?
 */
static void decode_optical_flow_rad_msg(struct mavlink_message *msg __attribute__((unused)))
{
  // TODO: do something useful with the data here
  static float timestamp = 0;
  timestamp = ((float)optical_flow_rad.time_usec) * 0.000001;
  DOWNLINK_SEND_PX4FLOW_RAD(DefaultChannel, DefaultDevice,
                        &timestamp,
                        &optical_flow_rad.sensor_id,
                        &optical_flow_rad.integration_time_us,
                        &optical_flow_rad.integrated_x,
                        &optical_flow_rad.integrated_y,
                        &optical_flow_rad.integrated_xgyro,
                        &optical_flow_rad.integrated_ygyro,
                        &optical_flow_rad.integrated_zgyro,
                        &optical_flow_rad.temperature,
                        &optical_flow_rad.quality,
                        &optical_flow_rad.time_delta_distance_us,
                        &optical_flow_rad.distance);
}

static void decode_heartbeat_msg(struct mavlink_message *msg __attribute__((unused)))
{
  //TODO: Check whether we have a right version of firmware etc? i.e. heartbeat.autopilot=MAV_AUTOPILOT_PX4
  DOWNLINK_SEND_HEARTBEAT(DefaultChannel, DefaultDevice,
                        &heartbeat.type,
                        &heartbeat.autopilot,
                        &heartbeat.base_mode,
                        &heartbeat.custom_mode,
                        &heartbeat.system_status,
                        &heartbeat.mavlink_version);
}

/**
 * Initialization function
 */
void px4flow_init(void)
{
  // register mavlink messages
  req_flow.msg_id = MAVLINK_OPTICAL_FLOW_MSG_ID;
  req_flow.callback = decode_optical_flow_msg;
  req_flow.msg.payload = (uint8_t *)(&optical_flow);
  mavlink_register_msg(&mavlink_tp, &req_flow);

  req_flow_rad.msg_id = MAVLINK_OPTICAL_FLOW_RAD_MSG_ID;
  req_flow_rad.callback = decode_optical_flow_rad_msg;
  req_flow_rad.msg.payload = (uint8_t *)(&optical_flow_rad);
  mavlink_register_msg(&mavlink_tp, &req_flow_rad);

  req_heartbeat.msg_id = MAVLINK_OPTICAL_HEARTBEAT_MSG_ID;
  req_heartbeat.callback = decode_heartbeat_msg;
  req_heartbeat.msg.payload = (uint8_t *)(&heartbeat);
  mavlink_register_msg(&mavlink_tp, &req_heartbeat);

}



/**
 * Downlink message for debug
 */
void px4flow_downlink(void)
{
  static float timestamp = 0;
  timestamp = ((float)optical_flow.time_usec) * 0.000001;
  DOWNLINK_SEND_PX4FLOW(DefaultChannel, DefaultDevice,
                        &timestamp,
                        &optical_flow.sensor_id,
                        &optical_flow.flow_x,
                        &optical_flow.flow_y,
                        &optical_flow.flow_comp_m_x,
                        &optical_flow.flow_comp_m_y,
                        &optical_flow.quality,
                        &optical_flow.ground_distance);
}

