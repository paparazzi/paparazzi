/*
 * Paparazzi adc functions
 *
 * Copyright (C) 2003-2010 The Paparazzi team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file adc.h
 *  \brief arch independent ADC (Analog to Digital Converter) API
 *
 * Facility to store last values in a circular buffer for a specific
 *   channel:
 *  - allocate a (struct adc_buf)
 *  - register it with the ::adc_buf_channel function
 */

#ifndef MCU_PERIPH_ADC_H
#define MCU_PERIPH_ADC_H

#include <inttypes.h>
#include "mcu_periph/adc_arch.h"

#define MAX_AV_NB_SAMPLE 0x20
#define DEFAULT_AV_NB_SAMPLE 0x20

/**
 Generic interface for all ADC hardware drivers, independent from
 microcontroller architecture.
*/

/**
    Struct to collect samples from ADC and building an average
    over MAX_AV_NB_SAMPLE values.
    See @ref adc_buf_channel.
*/
struct adc_buf {
  uint32_t sum;                      /* Sum of samples in buffer (avg = sum / av_nb_sample) */
  uint16_t values[MAX_AV_NB_SAMPLE]; /* Buffer for sample values from ADC                   */
  uint8_t  head;                     /* Position index of write head in buffer              */
  uint8_t  av_nb_sample;             /* Number of samples to use in buffer (used for avg)   */
};

/**
    Registers a buffer to be used to store the specified converted channel
    Usage:
@code
    struct adc_buf channel_buf;
    adc_buf_channel(1, &channel_buf, 12);
@endcode
    Registers channel_buf as buffer for ADC channel 1, with max index 12
    (12 samples).
*/
void adc_buf_channel(uint8_t adc_channel, struct adc_buf *s, uint8_t av_nb_sample);

/** Starts conversions */
void adc_init(void);

#endif
