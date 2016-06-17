/*
 * Copyright (C) 2010 Martin Muller
 * Copyright (C) 2016 Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/adcs/adc_generic.c
 *
 * This module can be used to read one or two values from the ADC channels
 * in a generic way. Data is reported through the default telemetry
 * channel (by default) or can be redirected to an other one (alternate
 * telemetry, datalogger) at a frequency defined in the telemetry xml file.
 *
 */

#include "adc_generic.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

uint16_t adc_generic_val1;
uint16_t adc_generic_val2;

#ifndef ADC_CHANNEL_GENERIC1
#ifndef ADC_CHANNEL_GENERIC2
#error "at least one ADC_CHANNEL_GENERIC1/2 needs to be defined to use the generic_adc module"
#endif
#endif

#ifndef ADC_CHANNEL_GENERIC_NB_SAMPLES
#define ADC_CHANNEL_GENERIC_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

#ifndef ADC_GENERIC_PERIODIC_SEND
#define ADC_GENERIC_PERIODIC_SEND TRUE
#endif

#ifdef ADC_CHANNEL_GENERIC1
static struct adc_buf buf_generic1;
#endif

#ifdef ADC_CHANNEL_GENERIC2
static struct adc_buf buf_generic2;
#endif

static void adc_msg_send(struct transport_tx *trans, struct link_device *dev) {
#ifdef ADC_CHANNEL_GENERIC1
  adc_generic_val1 = buf_generic1.sum / buf_generic1.av_nb_sample;
#endif
#ifdef ADC_CHANNEL_GENERIC2
  adc_generic_val2 = buf_generic2.sum / buf_generic2.av_nb_sample;
#endif
  pprz_msg_send_ADC_GENERIC(trans, dev, AC_ID, &adc_generic_val1, &adc_generic_val2);
}

void adc_generic_init(void)
{
  adc_generic_val1 = 0;
  adc_generic_val2 = 0;

#ifdef ADC_CHANNEL_GENERIC1
  adc_buf_channel(ADC_CHANNEL_GENERIC1, &buf_generic1, ADC_CHANNEL_GENERIC_NB_SAMPLES);
#endif
#ifdef ADC_CHANNEL_GENERIC2
  adc_buf_channel(ADC_CHANNEL_GENERIC2, &buf_generic2, ADC_CHANNEL_GENERIC_NB_SAMPLES);
#endif
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ADC_GENERIC, adc_msg_send);
#endif

}

void adc_generic_periodic(void)
{
#if ADC_GENERIC_PERIODIC_SEND
  adc_msg_send(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif
}

