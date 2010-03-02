/*
 * $Id$
 *  
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef BOOZ2_ANALOG_H
#define BOOZ2_ANALOG_H

extern void booz2_analog_init( void );

#ifdef USE_EXTRA_ADC
#include "std.h"

extern uint16_t booz2_adc_1; 
extern uint16_t booz2_adc_2; 
extern uint16_t booz2_adc_3; 
extern uint16_t booz2_adc_4; 

extern void booz2_analog_periodic( void );

extern void booz2_analog_baro_read(void);
extern void booz2_analog_bat_read(void);
extern void booz2_analog_extra_adc_read(void);
#endif


#include "booz2_analog_hw.h"

#endif /* BOOZ2_ANALOG_H */
