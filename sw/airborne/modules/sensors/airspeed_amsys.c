/*
 * Driver for a Amsys Differential Presure Sensor I2C
 * AMS 5812-0003-D
 *
 * Copyright (C) 2010 The Paparazzi Team
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
 */

#include "sensors/airspeed_amsys.h"
#include "estimator.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"
#include <math.h>
//#include <stdlib.h>

#ifndef USE_AIRSPEED
// Just a Warning --> We do't use it.
//#ifndef SENSOR_SYNC_SEND
//#warning either set USE_AIRSPEED or SENSOR_SYNC_SEND to use amsys_airspeed
//#endif
#endif

#define AIRSPEED_AMSYS_ADDR 0xF4 // original F0
#ifndef AIRSPEED_AMSYS_SCALE
#define AIRSPEED_AMSYS_SCALE 1
#endif
#ifndef AIRSPEED_AMSYS_OFFSET
#define AIRSPEED_AMSYS_OFFSET 0
#endif
#define AIRSPEED_AMSYS_OFFSET_MAX 29491
#define AIRSPEED_AMSYS_OFFSET_MIN 3277
#define AIRSPEED_AMSYS_OFFSET_NBSAMPLES_INIT 40
#define AIRSPEED_AMSYS_OFFSET_NBSAMPLES_AVRG 60
#define AIRSPEED_AMSYS_NBSAMPLES_AVRG 10
#ifndef AIRSPEED_AMSYS_MAXPRESURE
#define AIRSPEED_AMSYS_MAXPRESURE 2068//2073 //Pascal
#endif
#ifndef AIRSPEED_AMSYS_I2C_DEV
#define AIRSPEED_AMSYS_I2C_DEV i2c0
#endif
#ifdef MEASURE_AMSYS_TEMPERATURE
#define TEMPERATURE_AMSYS_OFFSET_MAX 29491
#define TEMPERATURE_AMSYS_OFFSET_MIN 3277
#define TEMPERATURE_AMSYS_MAX 110
#define TEMPERATURE_AMSYS_MIN -25
#endif

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

// Global variables
uint16_t airspeed_amsys_raw;
uint16_t tempAS_amsys_raw;
bool_t airspeed_amsys_valid;
float airspeed_tmp;
float pressure_amsys; //Pascal
float airspeed_amsys; //mps
float airspeed_scale;
float airspeed_filter;
struct i2c_transaction airspeed_amsys_i2c_trans;

// Local variables
volatile bool_t airspeed_amsys_i2c_done;
float airspeed_temperature = 0.0;
float airspeed_old = 0.0;


void airspeed_amsys_init( void ) {
	airspeed_amsys_raw = 0;
	airspeed_amsys = 0.0;
	pressure_amsys = 0.0;
	airspeed_amsys_i2c_done = TRUE;
	airspeed_amsys_valid = TRUE;
	airspeed_scale = AIRSPEED_SCALE;
	airspeed_filter = AIRSPEED_FILTER;
	airspeed_amsys_i2c_trans.status = I2CTransDone;
}

void airspeed_amsys_read_periodic( void ) {
#ifndef SITL
	if (airspeed_amsys_i2c_trans.status == I2CTransDone)
#ifndef MEASURE_AMSYS_TEMPERATURE
		I2CReceive(AIRSPEED_AMSYS_I2C_DEV, airspeed_amsys_i2c_trans, AIRSPEED_AMSYS_ADDR, 2);
#else
		I2CReceive(AIRSPEED_AMSYS_I2C_DEV, airspeed_amsys_i2c_trans, AIRSPEED_AMSYS_ADDR, 4);
#endif
		
#else // SITL
		extern float sim_air_speed;
		EstimatorSetAirspeed(sim_air_speed);
#endif //SITL
}

void airspeed_amsys_read_event( void ) {
	
	// Get raw airspeed from buffer
	airspeed_amsys_raw = 0;
	airspeed_amsys_raw = (airspeed_amsys_i2c_trans.buf[0]<<8) | airspeed_amsys_i2c_trans.buf[1];
#ifdef MEASURE_AMSYS_TEMPERATURE
	tempAS_amsys_raw = (airspeed_amsys_i2c_trans.buf[2]<<8) | airspeed_amsys_i2c_trans.buf[3];
	airspeed_temperature = (float)((float)(tempAS_amsys_raw-TEMPERATURE_AMSYS_OFFSET_MIN)/((float)(TEMPERATURE_AMSYS_OFFSET_MAX-TEMPERATURE_AMSYS_OFFSET_MIN)/TEMPERATURE_AMSYS_MAX)+TEMPERATURE_AMSYS_MIN);// Tmin=-25, Tmax=85
#endif
	
	// Check if this is valid airspeed
	if (airspeed_amsys_raw == 0)
		airspeed_amsys_valid = FALSE;
	else
		airspeed_amsys_valid = TRUE;

	// Continue only if a new airspeed value was received
	if (airspeed_amsys_valid) {
	 
		// raw not under offest min
		if (airspeed_amsys_raw<AIRSPEED_AMSYS_OFFSET_MIN)
			airspeed_amsys_raw = AIRSPEED_AMSYS_OFFSET_MIN;
		// raw not over offest max
		if (airspeed_amsys_raw>AIRSPEED_AMSYS_OFFSET_MAX)
			airspeed_amsys_raw = AIRSPEED_AMSYS_OFFSET_MAX;

		// calculate raw to pressure
		pressure_amsys = (float)(airspeed_amsys_raw-AIRSPEED_AMSYS_OFFSET_MIN)*AIRSPEED_AMSYS_MAXPRESURE/(float)(AIRSPEED_AMSYS_OFFSET_MAX-AIRSPEED_AMSYS_OFFSET_MIN);

		airspeed_tmp = sqrtf(2*(pressure_amsys)*airspeed_scale/1.2041); //without offset

	// 	Lowpass filter
		airspeed_amsys = airspeed_filter * airspeed_old + (1 - airspeed_filter) * airspeed_tmp;
		airspeed_old = airspeed_amsys;		
		
#ifdef USE_AIRSPEED
		EstimatorSetAirspeed(airspeed_amsys);
#endif
#ifdef SENSOR_SYNC_SEND
		DOWNLINK_SEND_AMSYS_AIRSPEED(DefaultChannel, &airspeed_amsys_raw, &pressure_amsys, &airspeed_tmp, &airspeed_amsys, &airspeed_temperature);
#else
		RunOnceEvery(10, DOWNLINK_SEND_AMSYS_AIRSPEED(DefaultChannel, &airspeed_amsys_raw, &pressure_amsys, &airspeed_tmp, &airspeed_amsys, &airspeed_temperature));
#endif
	} 

	// Transaction has been read
	airspeed_amsys_i2c_trans.status = I2CTransDone;
}

