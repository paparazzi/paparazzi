/*
 * Copyright (C) 2014 Eduardo Lavratti <agressiva@hotmail.com>
 *
 * This file is part of paparazzi
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

/** @file modules/sensors/temp_adc.h
 * temperature driver for LM35 and 100k NTC analog sensores
 */

#ifndef TEMP_ADC_H
#define TEMP_ADC_H

#include "std.h"

/// flag to enable sending every new measurement via telemetry
bool_t temp_adc_sync_send;

float calc_ntc(int16_t raw_temp);
float calc_lm35(int16_t raw_temp);

void temp_adc_init(void);
void temp_adc_periodic(void);

#endif /* TEMP_ADC_H */
