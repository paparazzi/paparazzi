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

/** \file humid_hih.c
 *  \brief Honeywell HIH-4030 sensor interface
 *
 *   This reads the values for humidity from the Honeywell HIH-4030 sensor.
 */

#include <std.h>
#include "modules/meteo/humid_hih.h"
#include "modules/meteo/temp_tmp102.h"
#include "modules/meteo/humid_dpicco.h"
#include "modules/meteo/humid_sht.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"


#ifndef ADC_CHANNEL_HUMID_HIH
#define ADC_CHANNEL_HUMID_HIH ADC_3
#endif

#ifndef ADC_CHANNEL_HUMID_NB_SAMPLES
#define ADC_CHANNEL_HUMID_NB_SAMPLES 16
#endif

uint16_t adc_humid_hih;
float fhih_humid;

static struct adc_buf buf_humid_hih;

void humid_hih_init(void)
{
  adc_buf_channel(ADC_CHANNEL_HUMID_HIH, &buf_humid_hih, ADC_CHANNEL_HUMID_NB_SAMPLES);
}

void humid_hih_periodic(void)
{
  float fvout, fhih_temp;

  /* get temperature from external source */
  fhih_temp = ftempsht;
  /****************************************/

  adc_humid_hih = buf_humid_hih.sum / buf_humid_hih.av_nb_sample;

  /* 36k/68k voltage divider, 3.3V full sweep, 10 bits adc */
  fvout = (adc_humid_hih / 1024.) * 3.3 * (104. / 68.);

  /* 5V supply, 1st order curve fit */
  fhih_humid = ((fvout / 5.0) - 0.16) / 0.0062;

  /* temperature compensation */
  fhih_humid = fhih_humid / (1.0546 - (0.00216 * fhih_temp));

  DOWNLINK_SEND_HIH_STATUS(DefaultChannel, DefaultDevice, &adc_humid_hih, &fhih_humid, &fhih_temp);
}

