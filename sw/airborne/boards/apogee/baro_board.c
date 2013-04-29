 /*
 * Copyright (C) 2010 ENAC
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

/**
 * @file boards/apogee/baro_board.c
 *
 * integrated barometer for Apogee boards (mpl3115)
 */

#include "subsystems/sensors/baro.h"

// to get MPU status
#include "boards/apogee/imu_apogee.h"


/* Common Baro struct */
struct Baro baro;

/** Counter to init ads1114 at startup */
#define BARO_STARTUP_COUNTER 200
uint16_t startup_cnt;

void baro_init( void ) {
  mpl3115_init();
  baro.status = BS_UNINITIALIZED;
  baro.absolute     = 0;
  baro.differential = 1; /* not handled on this board, use extra module */
  startup_cnt = BARO_STARTUP_COUNTER;
}

void baro_periodic( void ) {

  if (baro.status == BS_UNINITIALIZED && mpl3115_data_available) {
    // Run some loops to get correct readings from the adc
    --startup_cnt;
    mpl3115_data_available = FALSE;
    if (startup_cnt == 0) {
      baro.status = BS_RUNNING;
    }
  }

  // Baro is slave of the MPU, only start reading it after MPU is configured
  if (imu_apogee.mpu.config.initialized)
    Mpl3115Periodic();
}

