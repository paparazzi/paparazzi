/*
 * Copyright (C) 2012 Gautier Hattenberger
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

/*
 * Wrapper for the board specific barometer
 * Allows to use external baro sensor to feed the general baro interface
 */

#include "modules/sensors/baro_board_module.h"

/* Common Baro struct */
struct Baro baro;

/* Counter to init custom baro at startup */
#define BARO_STARTUP_COUNTER 200
uint16_t startup_cnt;

/** Implementation of the generic baro interface initialization.
 *  No need to call this functions from the modules, already done by main.
 */
void baro_init( void ) {
  baro.status = BS_UNINITIALIZED;
  baro.absolute     = 0;
  baro.differential = 0;
  startup_cnt = BARO_STARTUP_COUNTER;
}

/** Implementation of the generic baro interface periodic task.
 *  No need to call this functions from the modules, already done by main.
 */
void baro_periodic( void ) {
  if (baro.status == BS_UNINITIALIZED) {
    // Run some loops to get correct readings from the adc
    --startup_cnt;
    if (startup_cnt == 0) {
      baro.status = BS_RUNNING;
    }
  }
}

