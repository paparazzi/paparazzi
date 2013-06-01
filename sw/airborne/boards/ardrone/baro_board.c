/*
 * Copyright (C) 2012 TU Delft Quatrotor Team 1
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file boards/ardrone/baro_board.c
 * Paparazzi AR Drone 2 Baro Sensor implementation:.
 *
 * These functions are mostly empty because of the calibration and calculations
 * done by the Parrot Navigation board.
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
//    baro.absolute = navdata->pressure; // When this is un-commented the ardrone gets a pressure
    // TODO do the right calculations for the right absolute pressure
    baro.absolute = 0;
    baro_data_available = TRUE;
  }
  else {
    baro_data_available = FALSE;
  }
}
