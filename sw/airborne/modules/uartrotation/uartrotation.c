/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
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
 * @file "modules/readlocationfromodroid/readlocationfromodroid.c"
 * @author Roland
 * reads from the odroid and sends information of the drone to the odroid using JSON messages
 */

#include "modules/uartrotation/uartrotation.h"
#include "subsystems/abi.h"
#include <serial_port.h>
#include <stdio.h>
#include <inttypes.h>
#include <errno.h>
#include "state.h"
#include "subsystems/gps.h"
#include "subsystems/ins/ins_int.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "mcu_periph/uart.h"
#include "subsystems/gps.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/stereoprotocol.h"

int frameNumberSending=0;
float sonar_meas=0.0;
 static abi_event odroid_agl_ev;
 float lastKnownHeight = 0.0;
 int pleaseResetOdroid = 0;
 int gps_reset_position = 0;

 struct link_device *linkdevodroid;

static void write_serial_rot(struct transport_tx *trans, struct link_device *devasdf) {

	 	struct Int32RMat *ltp_to_body_mat = stateGetNedToBodyRMat_i();
	int32_t lengthArrayInformation = 11*sizeof(int32_t);
	uint8_t ar[lengthArrayInformation];
	int32_t *pointer = (int32_t*)ar;
	for(int indexRot = 0; indexRot < 9; indexRot++){
		pointer[indexRot]=ltp_to_body_mat->m[indexRot];
	}
	pointer[9]=(int32_t)(sonar_meas*100);
	pointer[10]=frameNumberSending++;
	printf("Whoo sending serial\n");
	stereoprot_sendArray( &((GPS_LINK).device),ar, lengthArrayInformation, 1);
	printf("Whoo sending serial2\n");
}

struct SerialPort *READING_port;
speed_t usbInputSpeed = B1000000;
char *serialResponse;
int writeLocationInput=0;

void odroid_loc_init() {
	printf("Init odroid loc");
	// Open the serial port
	READING_port = serial_port_new();
	int lengthBytesImage=50000;
	serialResponse=malloc(lengthBytesImage*sizeof(char));
	memset(serialResponse, '0', lengthBytesImage);
	register_periodic_telemetry(DefaultPeriodic, "SERIALRMAT", write_serial_rot);
 }

void odroid_loc_periodic() {

}

static void gps_odroid_height(uint8_t sender_id __attribute__((unused)), float distance) {
	// Update the distance if we got a valid measurement
	if (distance > 0) {
		lastKnownHeight = distance;
	}
}


