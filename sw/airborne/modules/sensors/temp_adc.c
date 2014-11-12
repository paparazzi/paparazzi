/*
 * Copyright (C) 2014 Eduardo Lavratti <agressiva@hotmail.com>
 *
 * This file is part of paparazzi
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

/** @file modules/sensors/temp_adc.c
 * Temperature sensor module for LM35 or NTC (10k / 100k) sensor via analog input.
 */

#include "std.h"
#include "modules/sensors/temp_adc.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include BOARD_CONFIG

float temp_c1, temp_c2, temp_c3;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

#define LM35 0
#define NTC 1

#ifndef TEMP_ADC_CHANNEL1
#ifndef TEMP_ADC_CHANNEL2
#ifndef TEMP_ADC_CHANNEL3
#error "at least one TEMP_ADC_CHANNEL1/2/3 needs to be defined to use the temp_adc module"
#endif
#endif
#endif

#ifndef TEMP_ADC_CHANNEL1_TYPE
#define TEMP_ADC_CHANNEL1_TYPE LM35
#endif
#ifndef TEMP_ADC_CHANNEL2_TYPE
#define TEMP_ADC_CHANNEL2_TYPE LM35
#endif
#ifndef TEMP_ADC_CHANNEL3_TYPE
#define TEMP_ADC_CHANNEL3_TYPE LM35
#endif

#ifdef TEMP_ADC_CHANNEL1
static struct adc_buf temp_buf1;
#endif

#ifdef TEMP_ADC_CHANNEL2
static struct adc_buf temp_buf2;
#endif

#ifdef TEMP_ADC_CHANNEL3
static struct adc_buf temp_buf3;
#endif

#ifndef TEMP_ADC_NB_SAMPLES
#define TEMP_ADC_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

/** Send a TEMP_ADC message with every new measurement.
 * Mainly for debug, use with caution, sends message at ~10Hz.
 */
#ifndef TEMP_ADC_SYNC_SEND
#define TEMP_ADC_SYNC_SEND FALSE
#endif

float calc_ntc(int16_t raw_temp)
{
  float temp_c;
  //calc for NTC
  temp_c = log(((10240000 / raw_temp) - 10000) * 10);
  //temp_c = 1/(0.001129148+(0.000234125*temp_c)+(0.0000000876741*temp_c*temp_c*temp_c));
  temp_c = 1 / (0.000603985662844 + (0.000229995493730 * temp_c) + (0.000000067653027 * temp_c *
                temp_c * temp_c));
  temp_c = temp_c - 273.15; //convert do celcius
  return temp_c;
}

float calc_lm35(int16_t raw_temp)
{
  return ((float)raw_temp * (3300.0f / 1024.0f) / 10.0f);
}

static void temp_adc_downlink(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_TEMP_ADC(trans, dev, AC_ID, &temp_c1, &temp_c2, &temp_c3);
}

void temp_adc_init(void)
{
  temp_adc_sync_send = TEMP_ADC_SYNC_SEND;

#ifdef TEMP_ADC_CHANNEL1
  adc_buf_channel(TEMP_ADC_CHANNEL1, &temp_buf1, TEMP_ADC_NB_SAMPLES);
#endif

#ifdef TEMP_ADC_CHANNEL2
  adc_buf_channel(TEMP_ADC_CHANNEL2, &temp_buf2, TEMP_ADC_NB_SAMPLES);
#endif

#ifdef TEMP_ADC_CHANNEL3
  adc_buf_channel(TEMP_ADC_CHANNEL3, &temp_buf3, TEMP_ADC_NB_SAMPLES);
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "TEMP_ADC", temp_adc_downlink);
#endif
}


void temp_adc_periodic(void)
{
  uint16_t adc_raw;

#ifdef TEMP_ADC_CHANNEL1
  adc_raw = temp_buf1.sum / temp_buf1.av_nb_sample;
#if TEMP_ADC_CHANNEL1_TYPE == LM35
  temp_c1 = calc_lm35(adc_raw);
#elif TEMP_ADC_CHANNEL1_TYPE == NTC
  temp_c1 = calc_ntc(adc_raw);
#endif
#endif

#ifdef TEMP_ADC_CHANNEL2
  adc_raw = temp_buf2.sum / temp_buf2.av_nb_sample;
#if TEMP_ADC_CHANNEL2_TYPE == LM35
  temp_c2 = calc_lm35(adc_raw);
#elif TEMP_ADC_CHANNEL2_TYPE == NTC
  temp_c2 = calc_ntc(adc_raw);
#endif
#endif

#ifdef TEMP_ADC_CHANNEL3
  adc_raw = temp_buf3.sum / temp_buf3.av_nb_sample;
#if TEMP_ADC_CHANNEL3_TYPE == LM35
  temp_c3 = calc_lm35(adc_raw);
#elif TEMP_ADC_CHANNEL3_TYPE == NTC
  temp_c3 = calc_ntc(adc_raw);
#endif
#endif

  if (temp_adc_sync_send) {
    temp_adc_downlink(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
  }

}
