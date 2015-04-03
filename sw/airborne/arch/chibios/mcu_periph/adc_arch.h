/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
 * Copyright (C) 2015 Gautier Hattenberger, Alexandre Bustico
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
/**
 * @file arch/chibios/mcu_periph/adc_arch.h
 * ADC driver
 *
 * Right now for STM32F1xx only
 */
#ifndef ADC_ARCH_H
#define ADC_ARCH_H

#include BOARD_CONFIG

// NB_ADCx_CHANNELS
enum adc1_channels {
#ifdef AD1_1_CHANNEL
  AD1_1,
#endif
#ifdef AD1_2_CHANNEL
  AD1_2,
#endif
#ifdef AD1_3_CHANNEL
  AD1_3,
#endif
#ifdef AD1_4_CHANNEL
  AD1_4,
#endif
#if USE_AD1_5
  AD1_5,
#endif
#if USE_AD1_6
  AD1_6,
#endif
#if USE_AD1_7
  AD1_7,
#endif
#if USE_AD1_8
  AD1_8,
#endif
#if USE_AD1_9
  AD1_9,
#endif
#if USE_AD1_10
  AD1_10,
#endif
#if USE_AD1_11
  AD1_11,
#endif
#if USE_AD1_12
  AD1_12,
#endif
#if USE_AD1_13
  AD1_13,
#endif
#if USE_AD1_14
  AD1_14,
#endif
#if USE_AD1_15
  AD1_15,
#endif
#if USE_AD1_16
  AD1_16,
#endif
  NB_ADC1_CHANNELS
};

enum adc2_channels {
//#if USE_AD2_1
//  AD2_1 = NB_ADC1_CHANNELS,
//#endif
//#if USE_AD2_2
//  AD2_2,
//#endif
//#if USE_AD2_3
//  AD2_3,
//#endif
//#if USE_AD2_4
//  AD2_4,
//#endif
  // TBC
  NB_ADC2_CHANNELS
};

#ifdef NB_ADC
#undef NB_ADC
#endif

#define NB_ADC (NB_ADC1_CHANNELS)

#include "hal.h"

// ADC error flags
extern uint8_t adc_error_flag;
extern ADCDriver *adcp_err;

#endif /* ADC_ARCH_H */
