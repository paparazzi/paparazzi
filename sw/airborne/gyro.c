/*
 * $Id$
 *  
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin
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

/** \file gyro.c
 * \brief Basic code for gyro acquisition on ADC channels
 *
*/

#include CONFIG
#include "gyro.h"
#include "std.h"
#include "adc.h"
#include "airframe.h"

int16_t roll_rate_adc;
float roll_rate;

static struct adc_buf buf_roll;

#if defined SPARK_FUN
#define RadiansOfADC(_adc) RadOfDeg((_adc/3.41))
static struct adc_buf buf_temp;
float temp_comp;
#elif defined IDC300
#define RadiansOfADC(_adc) RadOfDeg((_adc/3.41))
static struct adc_buf buf_pitch;
float pitch_rate;
#endif

void gyro_init( void) {
  adc_buf_channel(ADC_CHANNEL_GYRO_ROLL, &buf_roll, ADC_CHANNEL_GYRO_NB_SAMPLES);

#if defined SPARK_FUN
  adc_buf_channel(ADC_CHANNEL_GYRO_TEMP, &buf_temp, ADC_CHANNEL_GYRO_NB_SAMPLES);
#elif defined IDC300
  adc_buf_channel(ADC_CHANNEL_GYRO_PITCH, &buf_pitch, ADC_CHANNEL_GYRO_NB_SAMPLES);
#endif
}



void gyro_update( void ) {
#ifdef SPARK_FUN
  temp_comp = buf_temp.sum/buf_temp.av_nb_sample - GYRO_ADC_TEMP_NEUTRAL;
  
  roll_rate_adc = (buf_roll.sum/buf_roll.av_nb_sample) - (GYRO_ADC_ROLL_NEUTRAL+(GYRO_ADC_TEMP_SLOPE*temp_comp)); 
#elif defined IDC300
  pitch_rate = buf_pitch.sum/buf_pitch.av_nb_sample - GYRO_ADC_PITCH_NEUTRAL;
  pitch_rate = RadiansOfADC(pitch_rate);
  roll_rate_adc = buf_roll.sum/buf_roll.av_nb_sample - GYRO_ADC_ROLL_NEUTRAL;
#endif
  roll_rate = GYRO_ADC_ROLL_COEF * RadiansOfADC(roll_rate_adc);
}
