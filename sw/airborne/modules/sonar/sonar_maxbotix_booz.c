/*
 * $Id: demo_module.c 3079 2009-03-11 16:55:42Z gautier $
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

#include "sonar_maxbotix_booz.h"
#include "booz2_analog.h"

uint16_t sonar_meas;
bool_t sonar_data_available;

void maxbotix_init(void) {
  sonar_meas = 0;
  sonar_data_available = FALSE;
}

/** Read ADC value to update sonar measurement
 */
void maxbotix_read(void) {
  booz2_analog_extra_adc_read();
  sonar_meas = booz2_adc_1;
  sonar_data_available = TRUE;
}

