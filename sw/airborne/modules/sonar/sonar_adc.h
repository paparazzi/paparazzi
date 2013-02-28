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

/** \file sonar_maxbotix.h
 *
 * simple driver to deal with one maxbotix sensor
 */

#ifndef SONAR_ADC_H
#define SONAR_ADC_H

#include "std.h"

extern uint16_t sonar_meas;
extern bool_t sonar_data_available;
extern float sonar_distance;

extern void sonar_adc_init(void);
extern void sonar_adc_read(void);

#define SonarEvent(_handler) { \
  if (sonar_data_available) { \
    _handler(); \
    sonar_data_available = FALSE; \
  } \
}

#endif
