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

/** \file light_temt.c
 *  \brief Vishay TEMT6000 ambient light sensor interface
 *
 *   This reads the values for light intensity from the Vishay TEMT6000 sensor.
 */


#include "modules/meteo/light_temt.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#ifndef ADC_CHANNEL_LIGHT_TEMT
#define ADC_CHANNEL_LIGHT_TEMT ADC_4
#endif

#ifndef ADC_CHANNEL_LIGHT_NB_SAMPLES
#define ADC_CHANNEL_LIGHT_NB_SAMPLES 16
#endif


uint16_t adc_light_temt;

static struct adc_buf buf_light_temt;

void light_temt_init(void)
{
  adc_buf_channel(ADC_CHANNEL_LIGHT_TEMT, &buf_light_temt, ADC_CHANNEL_LIGHT_NB_SAMPLES);
}

void light_temt_periodic(void)
{
  float f_light_temt;

  adc_light_temt = buf_light_temt.sum / buf_light_temt.av_nb_sample;

  /* 3.6k/6.8k voltage divider, 10 bits adc */
  f_light_temt = (adc_light_temt / 1024.) * 100.;

  DOWNLINK_SEND_TEMT_STATUS(DefaultChannel, DefaultDevice, &adc_light_temt, &f_light_temt);
}

