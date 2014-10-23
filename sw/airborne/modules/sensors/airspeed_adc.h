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

/** @file modules/sensors/airspeed_adc.h
 * Read an airspeed or differential pressure sensor via onboard ADC.
 */

#ifndef AIRSPEED_ADC_H
#define AIRSPEED_ADC_H

#include <inttypes.h>

struct AirspeedAdc {
  uint16_t val;
  uint16_t offset;
  float scale;    ///< used as quadratic scale if AIRSPEED_ADC_QUADRATIC_SCALE, otherwise linear
  float airspeed;
};

extern struct AirspeedAdc airspeed_adc;

void airspeed_adc_init(void);
void airspeed_adc_update(void);

#endif /* AIRSPEED_ADC_H */
