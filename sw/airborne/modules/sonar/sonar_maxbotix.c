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

uint16_t sonar_meas;
bool_t sonar_data_available;

static struct adc_buf sonar_adc;

void maxbotix_init(void) {
  sonar_meas = 0;
  sonar_data_available = FALSE;

  adc_buf_channel(ADC_CHANNEL_SONAR, &sonar_adc, DEFAULT_AV_NB_SAMPLE);
}

/** Read ADC value to update sonar measurement
 */
void maxbotix_read(void) {
  sonar_meas = sonar_adc.sum / sonar_adc.av_nb_sample;
  sonar_data_available = TRUE;
}

