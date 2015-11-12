/*
 * Copyright (C) ROland
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
 * @file "modules/read_matrix_serial/read_matrix_serial.c"
 * @author ROland
 * reads from the serial
 */


#include <stdio.h>
#include <sys/fcntl.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <inttypes.h>
#include "read_matrix_serial.h"
#include "math/pprz_geodetic_int.h"
#include "modules/read_matrix_serial/read_matrix_serial.h"
#include "modules/computer_vision/stabilization_practical.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/stereo_cam/stereocam.h"

void serial_init(void) {

}

void serial_start(void)
{
	//printf("serial start\n");
}
void serial_update(void)
{
	if(stereocam_data.fresh){
		// Do something with:
//		stereocam_data.data;
	}
}





 
