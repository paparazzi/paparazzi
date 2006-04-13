/*
 * Paparazzi adc functions
 *  
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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

/** \file adc.h
 *  \brief Analog to Digital Converter API
 *
 * Facility to store last values in a circular buffer for a specific
 *   channel:
 *  - allocate a (struct adc_buf)
 *  - register it with the ::adc_buf_channel function
 */

#ifndef _ADC_H_
#define _ADC_H_

#include <inttypes.h>

#define NB_ADC 8
#define MAX_AV_NB_SAMPLE 0x20
#define DEFAULT_AV_NB_SAMPLE 0x20

/** 
*/

/** Data structure used to store samples */
struct adc_buf {
  uint16_t sum;
  uint16_t values[MAX_AV_NB_SAMPLE];
  uint8_t  head;
  uint8_t  av_nb_sample;
};

/** Registers a buffer to be used to store the specified converted channel */
void adc_buf_channel(uint8_t adc_channel, struct adc_buf* s, uint8_t av_nb_sample);

/** Starts conversions */
void adc_init( void );
#endif
