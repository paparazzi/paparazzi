/*
 * Copyright (C) 2010 Martin Muller
 * Copyright (C) 2016 Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/adcs/adc_generic.h
 *
 * This module can be used to read one or two values from the ADC channels
 * in a generic way. Data is reported through the default telemetry
 * channel (by default) or can be redirected to an other one (alternate
 * telemetry, datalogger) at a frequency defined in the telemetry xml file.
 *
 */

#ifndef ADC_GENERIC_H
#define ADC_GENERIC_H

#include "std.h"

extern uint16_t adc_generic_val1;
extern uint16_t adc_generic_val2;
void adc_generic_init(void);
void adc_generic_periodic(void);

#endif /* ADC_GENERIC_H */
