/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
 * @brief ChibiOS arch dependent ADC files
 * @note Right now for STM32F1xx only
 */
#ifndef ADC_ARCH_H
#define ADC_ARCH_H

#include BOARD_CONFIG

#include "hal.h"


// ADC error flags, TODO: Make general for more ADC banks
extern uint8_t adc_error_flag;
extern ADCDriver* adcp_err;

// NB_ADCx_CHANNELS
enum adc1_channels {
#if USE_AD1_1
  ADC1_C1,
#endif
#if USE_AD1_2
  ADC1_C2,
#endif
#if USE_AD1_3
  ADC1_C3,
#endif
#if USE_AD1_4
  ADC1_C4,
#endif
  NB_ADC1_CHANNELS
};

enum adc2_channels {
#if USE_AD2_1
  ADC2_C1,
#endif
#if USE_AD2_2
  ADC2_C2,
#endif
#if USE_AD2_3
  ADC2_C3,
#endif
#if USE_AD2_4
  ADC2_C4,
#endif
  NB_ADC2_CHANNELS
};

#ifdef NB_ADC
#undef NB_ADC
#endif

#define NB_ADC (NB_ADC1_CHANNELS + NB_ADC2_CHANNELS)

#endif /* ADC_ARCH_H */
