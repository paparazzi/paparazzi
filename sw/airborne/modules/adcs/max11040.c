/*
 * Copyright (C) 2010 Martin Mueller
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

/** \file max11040.c
 *  \brief Maxim MAX11040 ADC interface
 *
 */

#include "led.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "max11040.h"
#include "adcs/max11040_hw.h"

volatile uint8_t max11040_status;
volatile uint8_t max11040_data;
volatile int32_t max11040_values[MAX11040_BUF_SIZE][MAXM_NB_CHAN] = {{0}};
volatile uint32_t max11040_timestamp[MAX11040_BUF_SIZE] = {0};
volatile uint8_t max11040_count;
volatile uint32_t max11040_buf_in;
volatile uint32_t max11040_buf_out;


void max11040_init(void)
{
  max11040_status = MAX11040_RESET;
  max11040_data = MAX11040_RESET;
  max11040_count = 0;
  max11040_buf_in = 0;
  max11040_buf_out = 0;

  max11040_hw_init();
}

void max11040_periodic(void)
{
#ifdef MAX11040_DEBUG
  float max11040_values_f[16];
  int i;

  if (max11040_data == MAX11040_DATA_AVAILABLE) {
//    LED_TOGGLE(3);
    for (i = 0; i < 16; i++) {
      /* we assume that the buffer will be full always in this test mode anyway */
      max11040_values_f[i] = (max11040_values[max11040_buf_in][i] * 2.2) / 8388608.0;
    }

    DOWNLINK_SEND_TURB_PRESSURE_VOLTAGE(
      DefaultChannel, DefaultDevice,
      &max11040_values_f[0],
      &max11040_values_f[1],
      &max11040_values_f[2],
      &max11040_values_f[3],
      &max11040_values_f[4],
      &max11040_values_f[5],
      &max11040_values_f[6],
      &max11040_values_f[7],
      &max11040_values_f[8],
      &max11040_values_f[9],
      &max11040_values_f[10],
      &max11040_values_f[11],
      &max11040_values_f[12],
      &max11040_values_f[13],
      &max11040_values_f[14],
      &max11040_values_f[15]);
    max11040_data = MAX11040_IDLE;
  }
#endif
}

