/*
 * Copyright (C) 2010 The Paparazzi Team
 * 
 * Autor: Bruzzlee
 * Angle of Attack ADC Sensor 
 * US DIGITAL MA3-A10-236-N
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

#ifndef AOA_ADC_H
#define AOA_ADC_H

#include <inttypes.h>

extern uint16_t adc_AOA_val;
extern float AOA_offset, AOA_filter;

void AOA_adc_init( void );
void AOA_adc_update( void );

#endif /* AOA_ADC_H */
