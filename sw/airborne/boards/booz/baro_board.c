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


/** threshold >0 && <1023 */
#ifndef BOOZ_ANALOG_BARO_THRESHOLD
#define BOOZ_ANALOG_BARO_THRESHOLD 850
#endif

/** scale factor to convert raw ADC measurement to pressure in Pascal.
 *
 * Sensor Sensitivity -> SS = 0.045 mv / Pa
 * Sensor Gain -> G = 94.25
 * Sensitivity -> S = SS*G = 4.24125 mV / Pa
 * 10 bit ADC -> A = 3.3 V / 1024 = 3.223 mV / LSB
 * Total Sensitivity SENS = A / S = 0.759837
 *
 * For the real pressure you also need to take into account the (variable) offset
 *
 * supply voltage Vs = 5V
 * real sensor sensitivity Vout = Vs * (0.009 P - 0.095)
 * voltage variable offset Voff(DAC) = Vs / 69.23 + (DAC * 3.3 / 1024) / 21.77
 * ADC voltage at init Vadc = 3.3*BARO_THRESHOLD/1024 = Vout - Voff
 *
 * => Inverting these formulas can give the 'real' pressure
 *
 * since we don't care that much in this case, we can take a fixed offset of 101325 Pa
 */
#ifndef BOOZ_BARO_SENS
#define BOOZ_BARO_SENS 0.759837
#endif

struct BaroBoard baro_board;

void baro_init(void)
{

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

void baro_periodic(void)
{

  baro_board.absolute = baro_board.buf.sum / baro_board.buf.av_nb_sample;
  baro_board.value_filtered = (3 * baro_board.value_filtered + baro_board.absolute) / 4;
  if (baro_board.status == BB_UNINITIALIZED) {
    RunOnceEvery(10, { baro_board_calibrate();});
  } else {
    uint32_t now_ts = get_sys_time_usec();
    float pressure = 101325.0 - BOOZ_BARO_SENS * (BOOZ_ANALOG_BARO_THRESHOLD - baro_board.absolute);
    AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, now_ts, pressure);
  }
}

/* decrement offset until adc reading is over a threshold */
void baro_board_calibrate(void)
{
  if (baro_board.value_filtered < BOOZ_ANALOG_BARO_THRESHOLD && baro_board.offset >= 1) {
    if (baro_board.value_filtered == 0 && baro_board.offset > 15) {
      baro_board.offset -= 15;
    } else {
      baro_board.offset--;
    }
    DACSet(baro_board.offset);
#ifdef BARO_LED
    LED_TOGGLE(BARO_LED);
#endif
  } else {
    baro_board.status = BB_RUNNING;
#ifdef BARO_LED
    LED_ON(BARO_LED);
#endif
  }
}

