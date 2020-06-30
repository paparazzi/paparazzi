/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/datalink/bitcraze/crtp.h
 *
 * CRTP protocol for communication with bitcraze/crazyflie modems
 *
 * based on PX4 implementation
 */

#ifndef CRTP_H
#define CRTP_H

#include "std.h"

#define CRTP_PORT_CONSOLE   0x00
#define CRTP_PORT_PARAM	    0x02
#define CRTP_PORT_COMMANDER	0x03
#define CRTP_PORT_MEM       0x04
#define CRTP_PORT_LOG       0x05

#define CRTP_PORT_PPRZLINK  0x09 // Non-standard port for transmitting pprzlink messages

#define CRTP_PORT_PLATFORM  0x0D
#define CRTP_PORT_DEBUG     0x0E
#define CRTP_PORT_LINK      0x0F

#define CRTP_NULL(x) (((x).header & 0xf3) == 0xf3)

// 1 byte header + 31 bytes data = 32 (max ESB packet size)
// With the NRF51, this could be increased to ~250, but the Crazyradio PA uses a NRF24 which can't do this
#define CRTP_MAX_DATA_SIZE 31

typedef struct {
	uint8_t size; // Total size of this message, including the header (placed here to overlap with syslink length field)
	union {
		uint8_t header;
		struct {
			uint8_t channel : 2;
			uint8_t link : 2;
			uint8_t port : 4;
		};
	};

	uint8_t data[CRTP_MAX_DATA_SIZE];
} __attribute__((packed)) crtp_message_t;


typedef struct {
	float roll; // -20 to 20
	float pitch;
	float yaw; // -150 to 150
	uint16_t thrust;
} crtp_commander;

#endif

