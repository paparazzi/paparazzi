/*
 * Copyright (C) 2011 Martin Mueller
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

/** \file light_solar.c
 *  \brief University of Reading solar radiation sensor interface
 *
 *   This reads the values for intensity from the University of Reading solar sensor.
 */


#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "modules/meteo/light_solar.h"

#ifndef ADC_CHANNEL_LIGHT_SOLAR_UP
#define ADC_CHANNEL_LIGHT_SOLAR_UP ADC_1
#endif
#ifndef ADC_CHANNEL_LIGHT_SOLAR_DN
#define ADC_CHANNEL_LIGHT_SOLAR_DN ADC_2
#endif

#ifndef ADC_CHANNEL_LIGHT_NB_SAMPLES
#define ADC_CHANNEL_LIGHT_NB_SAMPLES 16
#endif


uint16_t up[LIGHT_NB], dn[LIGHT_NB];
int32_t  light_cnt;

static struct adc_buf buf_light_sol_up;
static struct adc_buf buf_light_sol_dn;

void light_solar_init(void)
{
  adc_buf_channel(ADC_CHANNEL_LIGHT_SOLAR_UP, &buf_light_sol_up, ADC_CHANNEL_LIGHT_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_LIGHT_SOLAR_DN, &buf_light_sol_dn, ADC_CHANNEL_LIGHT_NB_SAMPLES);

  light_cnt = 0;
}

void light_solar_periodic(void)
{
  up[light_cnt] = buf_light_sol_up.sum / buf_light_sol_up.av_nb_sample;
  dn[light_cnt] = buf_light_sol_dn.sum / buf_light_sol_dn.av_nb_sample;

  /* 10k/10k voltage divider, 10 bits adc, 3.3V max */

  if (++light_cnt >= LIGHT_NB) {
    DOWNLINK_SEND_SOLAR_RADIATION(DefaultChannel, DefaultDevice,
                                  &up[0], &dn[0], &up[1], &dn[1], &up[2], &dn[2], &up[3], &dn[3],
                                  &up[4], &dn[4], &up[5], &dn[5], &up[6], &dn[6], &up[7], &dn[7],
                                  &up[8], &dn[8], &up[9], &dn[9]);
    light_cnt = 0;
  }
}

