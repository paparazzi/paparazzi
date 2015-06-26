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
 *  @brief Interface to MAVLink
 */

// Include own header
#include "modules/datalink/mavlink.h"

#include <stdio.h>
#include <stdint.h> // for INT16_MAX

// Include UDP socket headers
#include <sys/socket.h> // for socket system call
#include <sys/types.h>
#include <netinet/in.h>
#include <fcntl.h> // for making the socket receiving function nonblocking
#include <unistd.h> // for close

// Include PPRZ headers
#include "firmwares/rotorcraft/navigation.h" // for navigation calls
#include "mcu_periph/sys_time.h" // for periodic calls
#include "modules/datalink/missionlib/blocks.h" // for mission blocks
#include "subsystems/electrical.h"
#include "subsystems/gps.h"
#include "state.h"

mavlink_block_mgr block_mgr;

/**
 * REMOVE FOR RELEASE
 */
#define MAVLINK_FLAG_DEBUG 1
/**
 * REMOVE FOR RELEASE
 */

/**
 * Send heartbeat
 */
static void mavlink_send_heartbeat(void) 
{
#ifdef MAVLINK_FLAG_DEBUG
  printf("Send heartbeat message\n");
#endif 
  /* 
   * The heartbeat message consists of:
   *	- system ID (MAV ID)
   *    - comp ID (0)
   * 	- MAV type 
   * 	- autopilot type (MAV_AUTOPILOT_GENERIC = 0)
   * 	- base mode (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1)
   * 	- custom mode 
   * 	- system state // TODO: Transmit system state
   */
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(mavlink_system.sysid, // System id
  							 mavlink_system.compid, // Component id 
  							 &msg, // MAVLink message
  							 MAV_TYPE_QUADROTOR, // TODO: Allow for different configurations 
  							 MAV_AUTOPILOT_GENERIC, // Default
  							 MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // Custom autopilot mode
  							 0, // TODO: Transmit custom autopilot mode
  							 MAV_STATE_UNINIT); // TODO: Allow state information (MODE/CALIB) to be transmitted

  mavlink_send_message(&msg);
}

/**
 * Send attitude
 */
static void mavlink_send_attitude(void)
{
#ifdef MAVLINK_FLAG_DEBUG
  printf("Send attitude message\n");
#endif 	
  /*
   * The attitude message consists of:
   *	- boot time (ms)
   *	- roll angle (rad)
   *	- pitch angle (rad)
   * 	- yaw angle (rad)
   *	- roll rate (rad/s)
   * 	- pitch rate (rad/s)
   *	- yaw rate (rad/s)
   */
   mavlink_message_t msg;
   mavlink_msg_attitude_pack(mavlink_system.sysid, // system id
   							 mavlink_system.compid, // component id
   							 &msg, // MAVLink message
   							 get_sys_time_msec(), // timestamp
                             stateGetNedToBodyEulers_f()->phi, // roll angle
                             stateGetNedToBodyEulers_f()->theta, // pitch angle
                             stateGetNedToBodyEulers_f()->psi, // yaw angle
                             stateGetBodyRates_f()->p, // roll rate
                             stateGetBodyRates_f()->q, // pitch rate
                             stateGetBodyRates_f()->r); // yaw rate
   mavlink_send_message(&msg);
}

/**
 * Send local altitude and velocity in the local NED frame
 */
static void mavlink_send_altitude_ground_speed(void)
{
#ifdef MAVLINK_FLAG_DEBUG
  printf("Send altitude and ground speed message\n");
#endif 	
  /*
   * The altitude and ground speed message consists of:
   *	- airspeed (ignored)
   *	- groundspeed (m/s)
   * 	- altitude (m)
   *	- climb rate (m/s)
   *	- heading (ignored)
   *	- throttle setting (ignored) 
   */
   mavlink_message_t msg;
   mavlink_msg_vfr_hud_pack(mavlink_system.sysid, // system id
   							            mavlink_system.compid, // component id
   							            &msg, // MAVLink message
   							            -1, // airspeed
                            *stateGetHorizontalSpeedNorm_f(), // groundspeed
                            -1, // heading
                            0,  // throttle setting
                            stateGetPositionNed_f()->z, // altitude
                            stateGetSpeedNed_f()->z); // climb rate
   mavlink_send_message(&msg);
}

/**
 * Send battery status
 */
static void mavlink_send_battery_status(void)
{
#ifdef MAVLINK_FLAG_DEBUG
  printf("Send battery status message\n");
#endif 	
  /*
   * The battery status message consists of:
   *	- current consumed (ignored)
   *	- energy consumed (ignored)
   *	- temperature (ignored)
   *	- voltages[] (millivolt)
   *	- current battery (ignored)
   *	- id (ignored)
   *	- battery function (ignored)
   * 	- type (ignored)
   *	- battery remaining (ignored)
   */
   uint16_t voltages[10];
   voltages[0] = electrical.vsupply * 100; // The voltage of the battery is set to the first cell
   mavlink_message_t msg;
   mavlink_msg_battery_status_pack(mavlink_system.sysid, // system id
   							   			                mavlink_system.compid, // component id
   							  			                &msg, // MAVLink message
                                        0, // id
                                        0, // battery function
                                        0, // type
                                        INT16_MAX, // temperature
                                        voltages, // voltages[]
                                        -1, // current battery
							   			                  -1, // current consumed
                        	   			      -1, // energy consumed                    	   			      
                        	   			      0); // battery remaining
   mavlink_send_message(&msg);
}

/**
 * Send gps status
 */
static void mavlink_send_gps_status(void)
{
#ifdef MAVLINK_FLAG_DEBUG
  printf("Send GPS status message\n");
#endif 	
  /*
   * The GPS status message consists of:
   *	- satellites visible
   *	- satellite pnr[] (ignored)
   *	- satellite used[] (ignored)
   *	- satellite elevation[] (ignored)
   *	- satellite azimuth[] (ignored)
   *	- satellite snr[] (ignored)
   */
   uint8_t empty[20];
   mavlink_message_t msg;
   mavlink_msg_gps_status_pack(mavlink_system.sysid, // system id
   							   mavlink_system.compid, // component id
   							   &msg, // MAVLink message
   							   gps.num_sv, // satellites visible
                           	   empty, // satellite pnr[]
                           	   empty, // satellite used[]
                           	   empty, // satellite elevation[]
                           	   empty, // satellite azimuth[]
                           	   empty); // satellite snr[]
   mavlink_send_message(&msg);
}

/**
 * Send gps position
 */
static void mavlink_send_gps_position(void)
{
#ifdef MAVLINK_FLAG_DEBUG
  printf("Send GPS position message\n");
#endif 	
  /*
   * The GPS position message consists of:
   *	- time (ignored)
   *	- latitude (*1E7)
   *	- longitude (*1E7)
   *	- altitude (mm)
   *	- relative altitude (mm)
   *	- vx (ignored)
   *	- vy (ignored)
   *	- vz (ignored)
   *	- heading (degrees*100)
   */
  float heading = DegOfRad(stateGetNedToBodyEulers_f()->psi);
  if (heading < 0.) {
    heading += 360;
  }
  uint16_t compass_heading = heading * 100;
  
  int32_t relative_alt = stateGetPositionLla_i()->alt - state.ned_origin_f.hmsl;

  mavlink_message_t msg;
  mavlink_msg_global_position_int_pack(mavlink_system.sysid, // system id
 								   	   mavlink_system.compid, // component id
   								   	   &msg, // MAVLink message
   								   	   0, // time
                            	   	   stateGetPositionLla_i()->lat, // latitude
                                       stateGetPositionLla_i()->lon, // longitude
                                       stateGetPositionLla_i()->alt, // altitude
                                       relative_alt, // relative altitude
                            	   	   -1, // vx
                            	   	   -1, // vy
                            	   	   -1, // vz
                            	   	   compass_heading); // compass heading
  mavlink_send_message(&msg);
}

/**
 * Send transmit buffer 
 */
int mavlink_send_message(mavlink_message_t* msg)
{
  	uint8_t buf[MAVLINK_BUFFER_LENGTH];
  	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  	size_t bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr *)&rem_addr, sizeof(rem_addr));
  	if (bytes_sent != len) {
#define MAVLINK_BUFFER_SEND_ERROR_LINUX
#ifdef MAVLINK_BUFFER_SEND_ERROR_LINUX
  		perror("Failed to send current buffer.\n");
#else
  		// TODO: Fix for linux, stm32 etc.
#endif
  	}

  	return bytes_sent;
}

 /**
 * Interface initialization
 */
void mavlink_init(void)
{
	// Initialize MAVLink
	mavlink_system.sysid = 5;
	mavlink_system.compid = 20;
	mavlink_block_init(&block_mgr);

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
	memset(&rem_addr, 0, sizeof(rem_addr));
	rem_addr.sin_family = AF_INET; // Internet Protocol
	rem_addr.sin_addr.s_addr = inet_addr(MAVLINK_IP_ADDRESS_STR); 
  // rem_addr.sin_addr.s_addr = inet_addr("192.168.42.3"); 
	rem_addr.sin_port = htons(MAVLINK_UDP_PORT);
}

/**
 * Periodic MAVLink calls.
 * Called at MAVLINK_PERIODIC_FREQUENCY (10Hz)
 */
void mavlink_periodic(void)
{
	RunOnceEvery(10, mavlink_send_heartbeat()); // Call at 1Hz
	RunOnceEvery(1, mavlink_send_attitude()); // Call at 10Hz
	RunOnceEvery(1, mavlink_send_altitude_ground_speed()); // Call at 10Hz
	RunOnceEvery(10, mavlink_send_battery_status()); // Call at 1Hz
	RunOnceEvery(10, mavlink_send_gps_status()); // Call at 1Hz
	RunOnceEvery(1, mavlink_send_gps_position()); // Call at 10Hz
}

/**
 * Event loop MAVLink calls
 * Called at the end of the event loop (after the periodic loop is completed)
 */
void mavlink_event(void)
{
	// Update the current block index in the block manager
	if (block_mgr.current_block != nav_block) {
		block_mgr.current_block = nav_block;
		mavlink_block_cb(block_mgr.current_block);
	}

	socklen_t len;
	uint8_t buf[MAVLINK_BUFFER_LENGTH]; // incoming message buffer
	int16_t recsize = recvfrom(sock, (void *)buf, MAVLINK_BUFFER_LENGTH, 0, (struct sockaddr *)&rem_addr, &len); // retreive message from UDP buffer
	if (recsize > 0) {
		printf("Received %d bytes.\n", recsize);
		mavlink_message_t msg; // incoming message
		mavlink_status_t status;  
		for (uint16_t i = 0; i < recsize; ++i) {
			if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) { // parse a single byte at a time and continue if the message could be decoded
				// Packet received
#ifdef MAVLINK_FLAG_DEBUG
				printf("Received packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n\n", msg.sysid, msg.compid, msg.len, msg.msgid);
#endif
				mavlink_block_message_handler(&msg); // Blocks message handler
			}
		}
	}
}
