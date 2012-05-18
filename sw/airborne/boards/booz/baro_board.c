/* $Id$
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


#include "subsystems/sensors/baro.h"

#include "generated/airframe.h"
#include "led.h"

/* threshold >0 && <1023 */
#ifndef BOOZ_ANALOG_BARO_THRESHOLD
#define BOOZ_ANALOG_BARO_THRESHOLD 850
#endif

struct Baro baro;
struct BaroBoard baro_board;


void baro_init( void ) {

  adc_buf_channel(ADC_CHANNEL_BARO, &baro_board.buf, DEFAULT_AV_NB_SAMPLE);

  baro.status = BS_UNINITIALIZED;
  baro.absolute     = 0;
  baro.differential = 0; /* not handled on this board */

  baro_board.offset = 1023;
  DACSet(baro_board.offset);

  baro_board.value_filtered = 0;
  baro_board.data_available = FALSE;
#ifdef ROTORCRAFT_BARO_LED
  LED_OFF(ROTORCRAFT_BARO_LED);
#endif
}

void baro_periodic(void) {

  baro.absolute = baro_board.buf.sum/baro_board.buf.av_nb_sample;
  baro_board.value_filtered = (3*baro_board.value_filtered + baro.absolute)/4;
  if (baro.status == BS_UNINITIALIZED) {
    RunOnceEvery(10, { baro_board_calibrate();});
  }
  /*  else */
  baro_board.data_available = TRUE;
}

/* decrement offset until adc reading is over a threshold */
void baro_board_calibrate(void) {
  if (baro_board.value_filtered < BOOZ_ANALOG_BARO_THRESHOLD && baro_board.offset >= 1) {
    if (baro_board.value_filtered == 0 && baro_board.offset > 15)
      baro_board.offset -= 15;
    else
      baro_board.offset--;
    DACSet(baro_board.offset);
#ifdef ROTORCRAFT_BARO_LED
    LED_TOGGLE(ROTORCRAFT_BARO_LED);
#endif
  }
  else {
    baro.status = BS_RUNNING;
#ifdef ROTORCRAFT_BARO_LED
    LED_ON(ROTORCRAFT_BARO_LED);
#endif
  }
}




