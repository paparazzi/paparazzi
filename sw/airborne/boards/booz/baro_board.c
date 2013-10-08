/*
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

/**
 * @file boards/booz/baro_board.c
 *
 */


#include "subsystems/sensors/baro.h"

#include "generated/airframe.h"
#include "subsystems/abi.h"
#include "led.h"

#ifndef BOOZ_BARO_SENDER_ID
#define BOOZ_BARO_SENDER_ID 1
#endif

/** threshold >0 && <1023 */
#ifndef BOOZ_ANALOG_BARO_THRESHOLD
#define BOOZ_ANALOG_BARO_THRESHOLD 850
#endif

/** scale factor to convert raw ADC measurement to pressure in Pascal.
 * @todo check value
 * At low altitudes pressure change is ~1.2 kPa for every 100 meters.
 * So with previous scale of 15 for ADC -> meters with INT32_POS_FRAC we get:
 * 12 Pascal = (15 * ADC) << 8
 * -> SENS = ~ 12 / (15 * 256) = 0.003125
 */
#ifndef BOOZ_BARO_SENS
#define BOOZ_BARO_SENS 0.003125
#endif

struct BaroBoard baro_board;

void baro_init( void ) {

  adc_buf_channel(ADC_CHANNEL_BARO, &baro_board.buf, DEFAULT_AV_NB_SAMPLE);

  baro_board.status = BB_UNINITIALIZED;
  baro_board.absolute = 0;
  baro_board.offset = 1023;
  DACSet(baro_board.offset);
  baro_board.value_filtered = 0;
#ifdef BARO_LED
  LED_OFF(BARO_LED);
#endif
}

void baro_periodic(void) {

  baro_board.absolute = baro_board.buf.sum/baro_board.buf.av_nb_sample;
  baro_board.value_filtered = (3*baro_board.value_filtered + baro_board.absolute)/4;
  if (baro_board.status == BB_UNINITIALIZED) {
    RunOnceEvery(10, { baro_board_calibrate();});
  }
  else {
    float pressure = BOOZ_BARO_SENS*baro_board.absolute;
    AbiSendMsgBARO_ABS(BOOZ_BARO_SENDER_ID, &pressure);
  }
}

/* decrement offset until adc reading is over a threshold */
void baro_board_calibrate(void) {
  if (baro_board.value_filtered < BOOZ_ANALOG_BARO_THRESHOLD && baro_board.offset >= 1) {
    if (baro_board.value_filtered == 0 && baro_board.offset > 15)
      baro_board.offset -= 15;
    else
      baro_board.offset--;
    DACSet(baro_board.offset);
#ifdef BARO_LED
    LED_TOGGLE(BARO_LED);
#endif
  }
  else {
    baro_board.status = BB_RUNNING;
#ifdef BARO_LED
    LED_ON(BARO_LED);
#endif
  }
}

