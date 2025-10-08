/*
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

/**
 * @file sonar_adc.h
 * @brief Driver module for an analog rangefinder sensor connected to an ADC channel
 *
 * This module reads the ADC values from a rangefinder sensor and converts those values to a distance in meters or fractions thereof.
 *
 * Options include:
 * - Using a low-pass filter to smooth the distance output
 * - Updating AGL (Above Ground Level) in the state with the distance value.
 * - Rotation compensation, which compensates the AGL distance based on the current attitude of the aircraft.
 * - Use sensor simulation SITL (Software In The Loop).
 * - Sending periodic telemetry debug messages with the raw ADC value and the calculated distance.
 */

#ifndef SONAR_ADC_H
#define SONAR_ADC_H

#include "std.h"
struct SonarADC {
  uint16_t raw;    ///< raw measuread non scaled range value from sensor
  float scale;     ///< Scaling factor to convert raw value to a distance in SI unit (meters)
  float distance;  ///< Distance measured in meters
  bool update_agl; ///< Do or don't update AGL ABI message
};

extern struct SonarADC sonar_adc;

extern void sonar_adc_init(void);
extern void sonar_adc_periodic(void);
extern void sonar_adc_report(void);

#endif  /* SONAR_ADC_H */
