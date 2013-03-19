/*
 * Copyright (C) 2012 TU Delft Quatrotor Team 1
 *
 * Paparazzi AR Drone 2 Baro Sensor implementation:
 * These functions are mostly empty because of the calibration and calculations done by the Parrot Navigation board
 */

#include "subsystems/sensors/baro.h"
#include "baro_board.h"

struct Baro baro;

void baro_init(void) {
  baro.status = BS_UNINITIALIZED;
  baro.absolute     = 0;
  baro.differential = 0;
  baro_data_available = 0;
}

void baro_periodic(void) {
	baro.status = BS_RUNNING;
	if(navdata_baro_available == 1) {
		navdata_baro_available = 0;
//		baro.absolute = navdata->pressure; // When this is un-commented the ardrone gets a pressure
		// todo do the right calculations for the right absolute pressure
		baro.absolute = 0;
		baro_data_available = TRUE;
  }
  else
	  baro_data_available = FALSE;
}
