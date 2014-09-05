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
  ADC1_END
};

enum adc2_channels {
#ifdef AD2_1_CHANNEL
  AD2_1 = ADC1_END,
#endif
#ifdef AD2_2_CHANNEL
  AD2_2,
#endif
#ifdef AD2_3_CHANNEL
  AD2_3,
#endif
#ifdef AD2_4_CHANNEL
  AD2_4,
#endif
  ADC2_END
};

enum adc3_channels {
#ifdef AD3_1_CHANNEL
  AD3_1 = ADC2_END,
#endif
#ifdef AD3_2_CHANNEL
  AD3_2,
#endif
#ifdef AD3_3_CHANNEL
  AD3_3,
#endif
#ifdef AD3_4_CHANNEL
  AD3_4,
#endif
  ADC3_END
};

#if USE_ADC_WATCHDOG

/* Watchdog callback type definition
 */
typedef void (*adc_watchdog_callback)(void);

/* Watchdog register function
 *
 * @param adc adc bank to monitor
 * @param chan adc channel to monitor
 * @param low low threshhold for callback trigger
 * @param high high threshhold for callback trigger
 */
extern void register_adc_watchdog(uint32_t adc, uint8_t chan, uint16_t low, uint16_t high, adc_watchdog_callback cb);

#endif

#endif /* ADC_ARCH_H */
