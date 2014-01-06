/*
 * Copyright (C) 2010-2012  Paparazzi team
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

/**
 * @file arch/stm32/mcu_periph/adc_arch.h
 * @ingroup stm32_arch
 *
 * Driver for the analog to digital converters on STM32.
 */

#ifndef ADC_ARCH_H
#define ADC_ARCH_H

#include BOARD_CONFIG

// NB_ADCx_CHANNELS
enum adc1_channels {
#ifdef USE_AD1_1
  ADC1_C1,
#endif
#ifdef USE_AD1_2
  ADC1_C2,
#endif
#ifdef USE_AD1_3
  ADC1_C3,
#endif
#ifdef USE_AD1_4
  ADC1_C4,
#endif
  NB_ADC1_CHANNELS
};

enum adc2_channels {
#ifdef USE_AD2_1
  ADC2_C1,
#endif
#ifdef USE_AD2_2
  ADC2_C2,
#endif
#ifdef USE_AD2_3
  ADC2_C3,
#endif
#ifdef USE_AD2_4
  ADC2_C4,
#endif
  NB_ADC2_CHANNELS
};

#ifdef NB_ADC
#undef NB_ADC
#endif

#define NB_ADC (NB_ADC1_CHANNELS + NB_ADC2_CHANNELS)

#define AdcBank0(x) (x)
#define AdcBank1(x) (x+NB_ADC)

#endif /* ADC_ARCH_H */
