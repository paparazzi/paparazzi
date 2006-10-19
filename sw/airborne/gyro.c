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
#include "estimator.h"

int16_t roll_rate_adc;
static struct adc_buf buf_roll;

#define RadiansOfADC(_adc, scale) RadOfDeg((_adc * scale))

#if defined ADXRS150
static struct adc_buf buf_temp;
float temp_comp;
#elif defined IDG300
int16_t pitch_rate_adc;
static struct adc_buf buf_pitch;
#endif

void gyro_init( void) {
  adc_buf_channel(ADC_CHANNEL_GYRO_ROLL, &buf_roll, ADC_CHANNEL_GYRO_NB_SAMPLES);
#if defined ADXRS150
  adc_buf_channel(ADC_CHANNEL_GYRO_TEMP, &buf_temp, ADC_CHANNEL_GYRO_NB_SAMPLES);
#elif defined IDG300
  adc_buf_channel(ADC_CHANNEL_GYRO_PITCH, &buf_pitch, ADC_CHANNEL_GYRO_NB_SAMPLES);
#endif
}



void gyro_update( void ) {
  float pitch_rate = 0.;
  roll_rate_adc = (buf_roll.sum/buf_roll.av_nb_sample) - GYRO_ADC_ROLL_NEUTRAL; 
#ifdef ADXRS150
  temp_comp = buf_temp.sum/buf_temp.av_nb_sample - GYRO_ADC_TEMP_NEUTRAL;
  roll_rate_adc += GYRO_ADC_TEMP_SLOPE * temp_comp; 
#elif defined IDG300
  pitch_rate_adc = buf_pitch.sum/buf_pitch.av_nb_sample - GYRO_ADC_PITCH_NEUTRAL;
  pitch_rate = GYRO_PITCH_DIRECTION * RadiansOfADC(pitch_rate_adc, GYRO_PITCH_SCALE);
#endif
  float roll_rate = GYRO_ROLL_DIRECTION * RadiansOfADC(roll_rate_adc, GYRO_ROLL_SCALE);
  EstimatorSetRate(roll_rate, pitch_rate);
}
