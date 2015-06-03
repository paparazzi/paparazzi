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

/** @file modules/datalink/mavlink.c
 *  @brief Interface to PPRZServices
 */

// Include own header
#include "modules/datalink/mavlink.h"

#include <stdio.h>

// Include UDP socket headers
#include <sys/socket.h> // for socket system call
#include <sys/types.h>
#include <netinet/in.h>
#include <fcntl.h> // for making the socket receiving function nonblocking
#include <unistd.h> // for close

#include "mcu_periph/sys_time.h"

/**
 * Send transmit buffer 
 */
static int mavlink_send_message(mavlink_message_t* msg)
{
  	uint8_t buf[MAVLINK_BUFFER_LENGTH];
  	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  	size_t bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr *)&host_addr, sizeof(host_addr));
  	if (bytes_sent != len) {
#define MAVLINK_BUFFER_SEND_ERROR_LINUX
#ifdef MAVLINK_BUFFER_SEND_ERROR_LINUX
  		printf("Failed to send current buffer.\n");
#else
  		// TODO: Fix for linux, stm32 etc.
#endif
  	}

  	return bytes_sent;
}

/**
 * Send heartbeat
 */
static void mavlink_send_heartbeat(void) 
{
  printf("Send heartbeat message\n");

  /* 
   * The heartbeat message will contain:
   *	- system ID (MAV ID)
   *    - comp ID (0)
   * 	- MAV type 
   * 	- autopilot type (MAV_AUTOPILOT_GENERIC = 0)
   * 	- base mode (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1)
   * 	- custom mode 
   * 	- system state // TODO: Transmit system state
   */
  mavlink_msg_heartbeat_pack(MAVLINK_SYSTEM_SYSID, // System id
  							 MAVLINK_SYSTEM_COMPID, // Component id 
  							 &msg, // MAVLink message
  							 MAV_TYPE_QUADROTOR, // TODO: Allow for different configurations 
  							 MAV_AUTOPILOT_GENERIC, // Default
  							 MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // Custom autopilot mode
  							 0, // TODO: Transmit custom autopilot mode
  							 MAV_STATE_UNINIT); // TODO: Allow state information (MODE/CALIB) to be transmitted

  mavlink_send_message(&msg);
}

 /**
 * Interface initialization
 */
void mavlink_init(void)
{
	// Create socket
	if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP /* domain, type, protocol */)) < 0) {
#define MAVLINK_SOCKET_ERROR_LINUX
#ifdef MAVLINK_SOCKET_ERROR_LINUX
		perror("Could not create socket.\n");
#else
		// TODO: Fix for linux, stm32 etc.
#endif	
	}

	// Initialize the local address struct
	memset(&loc_addr, 0, sizeof(loc_addr));
	loc_addr.sin_family = AF_INET; // Internet Protocol
	loc_addr.sin_addr.s_addr = INADDR_ANY; // Attach the socket to any local address (is stored onboard the MAV)
	loc_addr.sin_port = htons(MAVLINK_UDP_PORT); // TODO: Allow a changing in/out port
	
	// Bind the socket to port PPRZSERVICES_UDP_PORT
	if (bind(sock, (struct sockaddr *)&loc_addr, sizeof(struct sockaddr)) < 0)
    {
		perror("Binding of socket to port failed.\n");
		close(sock);
    } 
	
	// Make the socket non-blocking to avoid freezing up the event loop
	if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
		perror("Could not make the socket non-blocking.\n");
		close(sock);
    }
	
	// Initialize the host address struct
	memset(&host_addr, 0, sizeof(host_addr));
	host_addr.sin_family = AF_INET; // Internet Protocol
	host_addr.sin_addr.s_addr = inet_addr("192.168.42.2"); // Set the target IP address (which is fixed!)
	host_addr.sin_port = htons(MAVLINK_UDP_PORT);
}

/**
 * Periodic MAVLink calls.
 * Called at MAVLINK_PERIODIC_FREQUENCY (10Hz)
 */
void mavlink_periodic(void)
{
	RunOnceEvery(10, mavlink_send_heartbeat()); // Call at 1Hz
}

/**
 * Event loop MAVLink calls
 * Called at the end of the event loop (after the periodic loop is completed)
 */
void mavlink_event(void)
{
	// TODO: Handle messages
}
