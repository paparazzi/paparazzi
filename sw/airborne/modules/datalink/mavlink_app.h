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

/** @file modules/datalink/mavlink.h
 *  @brief Interface to MAVLink
 */

#ifndef DATALINK_MAVLINK_H
#define DATALINK_MAVLINK_H

#include <arpa/inet.h> // for AP_INET addresses

#include "mavlink/mavlink_types.h"
#include "mavlink/paparazzi/mavlink.h"


#ifndef MAVLINK_DEBUG
#define MAVLINK_DEBUG(...) {};
#endif

#if MAVLINK_DEBUG == printf
#include <stdio.h>
#endif


#define QUOTE(name) #name
#define STR(macro) QUOTE(macro)

#define MAVLINK_IP_ADDRESS_STR STR(MAVLINK_IP_ADDRESS)

mavlink_system_t mavlink_system;

#ifndef MAVLINK_IP_ADDRESS
#error Please provide the IP address of the UDP interface
#endif
#ifndef MAVLINK_UDP_PORT
#define MAVLINK_UDP_PORT 5000
#endif
#ifndef MAVLINK_SYSTEM_SYSID
#define MAVLINK_SYSTEM_SYSID 0 // Arbitrary
#endif
#ifndef MAVLINK_SYSTEM_COMPID
#define MAVLINK_SYSTEM_COMPID 0 // MAV_COMP_ID_ALL=0
#endif
#define MAVLINK_BUFFER_LENGTH 256

// Create the socket identifier
int sock;

// Create the local address struct
struct sockaddr_in loc_addr;

// Create the host address struct
struct sockaddr_in rem_addr;

extern int mavlink_send_message(mavlink_message_t *msg);

void mavlink_init(void);
void mavlink_periodic(void);
void mavlink_event(void);

#endif // DATALINK_MAVLINK_H
