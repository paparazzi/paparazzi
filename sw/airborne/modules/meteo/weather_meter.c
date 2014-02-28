/*
 * $Id$
 *
 * Copyright (C) 2014 Martin Mueller
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

/** \file weather_meter.c
 *  \brief Measure wind direction and speed with a weather meter
 *         from Ambient/Froggit/Sparkfun
 *
 *   The wind anemometer is connected to the PPM input, the analog
 *   wind direction output is connected to AUX4 (ADC 3)
 */

/* one turn per second is 2.4km/h, add 10x for integer,
   add 100x for cm/s, add 1000x for milliseconds */
#define TURN_FACTOR     (24 * 100 * 1000)

/* we want m/s, convert from km/h, add 10x for integer */
#define KMH_MS_FACTOR   (36)

#include "weather_meter.h"
#include "modules/meteo/weather_meter_hw.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#ifndef ADC_CHANNEL_WEATHER_MTR
#warning "ADC_CHANNEL_WEATHER_MTR not defined, using AUX4 (ADC3) as default"
#define ADC_CHANNEL_WEATHER_MTR ADC_3
#define USE_ADC_3
#endif

#ifndef ADC_CHANNEL_WEATHER_MTR_NB_SAMPLES
#define ADC_CHANNEL_WEATHER_MTR_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

static struct adc_buf buf_generic1;
uint16_t adc_weather_meter_val;

int16_t get_wind_dir(uint16_t adc_wdir) {
  /* we assume a 10k resistor is connected to 3.3V */
  if (adc_wdir > 984) return -1;
  if (adc_wdir > 916) return 2700;
  if (adc_wdir > 857) return 3150;
  if (adc_wdir > 807) return 2925;
  if (adc_wdir > 744) return 0;
  if (adc_wdir > 666) return 3375;
  if (adc_wdir > 614) return 2250;
  if (adc_wdir > 530) return 2475;
  if (adc_wdir > 433) return 450;
  if (adc_wdir > 346) return 225;
  if (adc_wdir > 265) return 1800;
  if (adc_wdir > 214) return 2025;
  if (adc_wdir > 155) return 1350;
  if (adc_wdir > 109) return 1575;
  if (adc_wdir >  88) return 900;
  if (adc_wdir >  75) return 675;
  if (adc_wdir >  33) return 1125;
  return -1;
}

void weather_meter_init ( void ) {
  weather_meter_hw_init();
  adc_buf_channel(ADC_CHANNEL_WEATHER_MTR, &buf_generic1, ADC_CHANNEL_WEATHER_MTR_NB_SAMPLES);
}

void weather_meter_periodic( void ) {

  int16_t wind_dir;
  static uint32_t cycle_time, count = 1000;
  static uint16_t wind_speed;

  adc_weather_meter_val = buf_generic1.sum / buf_generic1.av_nb_sample;
  wind_dir = get_wind_dir(adc_weather_meter_val);

  if (weather_meter_hw_valid == TRUE) {

    cycle_time = MSEC_OF_CPU_TICKS(delta_t0);
    if (cycle_time > 0) {
      wind_speed = TURN_FACTOR / (KMH_MS_FACTOR * cycle_time);
    } else {
      wind_speed = 0;
    }

    weather_meter_hw_valid = FALSE;
    count = 0;
  }
  else {
    cycle_time = 0;
    if (count++ > (4 * 5)) {
      wind_speed = 0;
      count = 0;
    }
  }

  DOWNLINK_SEND_WEATHER_METER(DefaultChannel, DefaultDevice,
              &cycle_time,
              &adc_weather_meter_val,
              &wind_speed,
              &wind_dir );
}
