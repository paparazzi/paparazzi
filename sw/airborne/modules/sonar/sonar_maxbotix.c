/*
 *
 * Copyright (C) 2010  Gautier Hattenberger
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

#include "modules/sonar/sonar_maxbotix.h"
#include "mcu_periph/adc.h"
#include "subsystems/datalink/downlink.h"
#ifdef SITL
#include "subsystems/gps.h"
#endif

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#ifdef SONAR_DISTANCE
#ifndef SONAR_OFFSET
#define SONAR_OFFSET 0
#endif
#ifndef SONAR_SCALE
#define SONAR_SCALE 166
#endif
#endif

uint16_t sonar_meas;
bool_t sonar_data_available;

#ifdef SONAR_DISTANCE
float sonar_distance;
uint16_t sonar_offset;
uint16_t sonar_scale;
#endif

#ifndef SITL
static struct adc_buf sonar_adc;
#endif

void maxbotix_init(void) {
  sonar_meas = 0;
  sonar_data_available = FALSE;

#ifdef SONAR_DISTANCE
  sonar_distance = 0;
  sonar_offset = SONAR_OFFSET;
  sonar_scale = SONAR_SCALE;
#endif

#ifndef SITL
  adc_buf_channel(ADC_CHANNEL_SONAR, &sonar_adc, DEFAULT_AV_NB_SAMPLE);
#endif
}

/** Read ADC value to update sonar measurement
 */
void maxbotix_periodic(void) {
#ifndef SITL
  sonar_meas = sonar_adc.sum / sonar_adc.av_nb_sample;
  sonar_data_available = TRUE;

#ifdef SONAR_DISTANCE
//sonar_offset in cm, sonar_distance in m!
sonar_distance = ((float)sonar_meas * (float)sonar_scale) / 10000 + (float)sonar_offset / 100;
#ifdef SENSOR_SYNC_SEND_SONAR
  DOWNLINK_SEND_SONAR_MAXBOTIX(DefaultChannel, DefaultDevice, &sonar_distance);
#endif
#endif

#else // SITL
#ifdef SONAR_DISTANCE
sonar_distance = (gps.hmsl / 1000.0) - ground_alt;
Bound(sonar_distance, 0.1f, 7.0f);
#ifdef SENSOR_SYNC_SEND_SONAR
  DOWNLINK_SEND_SONAR_MAXBOTIX(DefaultChannel, DefaultDevice, &sonar_distance);
#endif
#endif
#endif // SITL
}

