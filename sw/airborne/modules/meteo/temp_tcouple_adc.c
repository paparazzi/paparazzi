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

/** \file temp_tcouple_adc.c
 *  \brief Universitaet Tuebingen thermocouple interface
 *
 *   This reads the values for reference and measurement temperature
 *   from the Universitaet Tuebingen thermocouple sensor.
 */


#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "modules/meteo/temp_tcouple_adc.h"

#ifndef ADC_CHANNEL_TEMP_REF
#define ADC_CHANNEL_TEMP_REF ADC_4
#endif
#ifndef ADC_CHANNEL_TEMP_VAL
#define ADC_CHANNEL_TEMP_VAL ADC_3
#endif

#ifndef ADC_CHANNEL_TEMP_TCOUPLE_NB_SAMPLES
#define ADC_CHANNEL_TEMP_TCOUPLE_NB_SAMPLES 16
#endif


uint16_t ref[TCOUPLE_NB], val[TCOUPLE_NB];
float    fref[TCOUPLE_NB], fval[TCOUPLE_NB];
int32_t  temp_cnt;

static struct adc_buf buf_temp_tcouple_ref;
static struct adc_buf buf_temp_tcouple_val;

void temp_tcouple_adc_init(void)
{
  adc_buf_channel(ADC_CHANNEL_TEMP_REF,
                  &buf_temp_tcouple_ref,
                  ADC_CHANNEL_TEMP_TCOUPLE_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_TEMP_VAL,
                  &buf_temp_tcouple_val,
                  ADC_CHANNEL_TEMP_TCOUPLE_NB_SAMPLES);
  temp_cnt = 0;
}

void temp_tcouple_adc_periodic(void)
{
  val[temp_cnt] = buf_temp_tcouple_val.sum / buf_temp_tcouple_val.av_nb_sample;
  ref[temp_cnt] = buf_temp_tcouple_ref.sum / buf_temp_tcouple_ref.av_nb_sample;

  /* no voltage divider, 10 bits adc, 3.3V max */
  /* T = U * 52.288899706 - 7.977784737996595 */
  fval[temp_cnt] = ((float)(val[temp_cnt] * 3.3) / 1023.)
                   * 52.288899706 - 7.977784737996595;
  fref[temp_cnt] = ((float)(ref[temp_cnt] * 3.3) / 1023.)
                   * 100. - 13.;

  if (++temp_cnt >= TCOUPLE_NB) {
    DOWNLINK_SEND_TEMP_TCOUPLE(DefaultChannel, DefaultDevice,
                               &fval[0], &fval[1], &fval[2], &fval[3],
                               &fref[0], &fref[1], &fref[2], &fref[3],
                               &val[0], &val[1], &val[2], &val[3],
                               &ref[0], &ref[1], &ref[2], &ref[3]);
    temp_cnt = 0;
  }
}

