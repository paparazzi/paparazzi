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

/** @file modules/sensors/airspeed_adc.c
 * Read an airspeed or differential pressure sensor via onboard ADC.
 */

#include "modules/sensors/airspeed_adc.h"
#include "mcu_periph/adc.h"
#include BOARD_CONFIG
#include "generated/airframe.h"
#include "state.h"

#ifndef USE_AIRSPEED_ADC
#define USE_AIRSPEED_ADC TRUE
#endif
PRINT_CONFIG_VAR(USE_AIRSPEED_ADC)

#if !defined AIRSPEED_ADC_QUADRATIC_SCALE && !defined AIRSPEED_ADC_SCALE
#error "You need to define either AIRSPEED_ADC_QUADRATIC_SCALE or AIRSPEED_ADC_SCALE (linear)."
#endif

struct AirspeedAdc airspeed_adc;

#ifndef SITL // Use ADC if not in simulation

#ifndef ADC_CHANNEL_AIRSPEED
#error "ADC_CHANNEL_AIRSPEED needs to be defined to use airspeed_adc module"
#endif

#ifndef ADC_CHANNEL_AIRSPEED_NB_SAMPLES
#define ADC_CHANNEL_AIRSPEED_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

static struct adc_buf buf_airspeed;

#endif

void airspeed_adc_init(void)
{
  airspeed_adc.airspeed = 0.0f;
  airspeed_adc.offset = AIRSPEED_ADC_BIAS;
#ifdef AIRSPEED_ADC_QUADRATIC_SCALE
  airspeed_adc.scale = AIRSPEED_ADC_QUADRATIC_SCALE;
#else
  airspeed_adc.scale = AIRSPEED_ADC_SCALE;
#endif

#ifndef SITL
  adc_buf_channel(ADC_CHANNEL_AIRSPEED, &buf_airspeed, ADC_CHANNEL_AIRSPEED_NB_SAMPLES);
#endif
}

void airspeed_adc_update(void)
{
#ifndef SITL
  airspeed_adc.val = buf_airspeed.sum / buf_airspeed.av_nb_sample;
  float airspeed_unscaled = Max(airspeed_adc.val - airspeed_adc.offset, 0);
#ifdef AIRSPEED_ADC_QUADRATIC_SCALE
  airspeed_adc.airspeed = airspeed_adc.scale * sqrtf(airspeed_unscaled);
#else
  airspeed_adc.airspeed = airspeed_adc.scale * airspeed_unscaled;
#endif

#elif !defined USE_NPS
  extern float sim_air_speed;
  airspeed_adc.airspeed = sim_air_speed;
#endif //SITL

#if USE_AIRSPEED_ADC
  stateSetAirspeed_f(&airspeed_adc.airspeed);
#endif
}
