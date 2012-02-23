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


/* driver for the analog Barometer Mpxa6115 using ADC ads1114 (16 bits I2C 860SpS max) from Texas instruments
 * Navarro & Gorraz & Hattenberger
 */

#include "subsystems/sensors/baro.h"
#include "led.h"

/* Common Baro struct */
struct Baro baro;

/* Counter to init mcp355x at startup */
#define STARTUP_COUNTER 200
uint16_t startup_cnt;

void baro_init( void ) {
  mcp355x_init();
  baro.status = BS_UNINITIALIZED;
  baro.absolute     = 0;
  baro.differential = 0; /* not handled on this board */
#ifdef ROTORCRAFT_BARO_LED
  LED_OFF(ROTORCRAFT_BARO_LED);
#endif
  startup_cnt = STARTUP_COUNTER;
}

// Need to play with slave select
#include "mcu_periph/spi.h"

void baro_periodic( void ) {

  if (baro.status == BS_UNINITIALIZED) {
    /**
     * Crappy code to empty the buffer
     * then unselect the device (goes to shutdown ?)
     * reselect to go to continious conversion mode
     * make some readings before setting BS_RUNNING
     * don't unselect the slave !
     */
    if (startup_cnt == 150) { SpiSelectSlave0(); mcp355x_read(); }
    else if (startup_cnt == 149) { SpiUnselectSlave0(); }
    else if (startup_cnt == 100) { SpiSelectSlave0(); }
    else if (startup_cnt < 90) { RunOnceEvery(4, mcp355x_read()); }
    // decrease init counter
    --startup_cnt;
#ifdef ROTORCRAFT_BARO_LED
    LED_TOGGLE(ROTORCRAFT_BARO_LED);
#endif
    if (startup_cnt == 0) {
      baro.status = BS_RUNNING;
#ifdef ROTORCRAFT_BARO_LED
      LED_ON(ROTORCRAFT_BARO_LED);
#endif
    }
  }
  // Read the ADC (at 50/4 Hz, conversion time is 68 ms)
  else { RunOnceEvery(4,mcp355x_read()); }
}

