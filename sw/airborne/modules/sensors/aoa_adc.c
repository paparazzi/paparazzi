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
 *
 */

/**
 * @file modules/sensors/aoa_adc.c
 * @brief Angle of Attack sensor on ADC
 * Autor: Bruzzlee
 *
 * ex: US DIGITAL MA3-A10-236-N
 */

#include "modules/sensors/aoa_adc.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"
#include "state.h"

// Messages
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

/// Default offset value (assuming 0 AOA is in the middle of the range)
#ifndef AOA_OFFSET
#define AOA_OFFSET M_PI
#endif
/// Default filter value
#ifndef AOA_FILTER
#define AOA_FILTER 0.5
#endif
/// Default sensitivity (2*pi on a 10 bit ADC)
#ifndef AOA_SENS
#define AOA_SENS ((2.0*M_PI)/1024)
#endif


// Downlink

#ifndef ADC_CHANNEL_AOA
#error "ADC_CHANNEL_AOA needs to be defined to use AOA_adc module"
#endif

#ifndef ADC_CHANNEL_AOA_NB_SAMPLES
#define ADC_CHANNEL_AOA_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

struct Aoa_Adc aoa_adc;

void aoa_adc_init(void)
{
  aoa_adc.offset = AOA_OFFSET;
  aoa_adc.filter = AOA_FILTER;
  aoa_adc.sens = AOA_SENS;
  aoa_adc.angle = 0.0;
  adc_buf_channel(ADC_CHANNEL_AOA, &aoa_adc.buf, ADC_CHANNEL_AOA_NB_SAMPLES);
}

void aoa_adc_update(void)
{
  static float prev_aoa = 0.0;

  aoa_adc.raw = aoa_adc.buf.sum / aoa_adc.buf.av_nb_sample;

  // PT1 filter and convert to rad
  aoa_adc.angle = aoa_adc.filter * prev_aoa +
                  (1.0 - aoa_adc.filter) * (aoa_adc.raw * aoa_adc.sens - aoa_adc.offset);
  prev_aoa = aoa_adc.angle;

#ifdef USE_AOA
  uint8_t flag = 1;
  float foo = 0.f;
  AbiSendMsgINCIDENCE(AOA_ADC_ID, flag, aoa_adc.angle, foo);
  stateSetAngleOfAttack_f(aoa_adc.angle);
#endif

  RunOnceEvery(30, DOWNLINK_SEND_AOA(DefaultChannel, DefaultDevice, &aoa_adc.raw, &aoa_adc.angle));
}

