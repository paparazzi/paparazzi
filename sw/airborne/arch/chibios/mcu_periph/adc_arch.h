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
 * ADC channel declarations for ChibiOS STM32 targets
 */
#ifndef ADC_ARCH_H
#define ADC_ARCH_H

#include "hal.h"

#include BOARD_CONFIG

#define ADC_NUM_CHANNELS 9

// ADC error flags
extern uint8_t adc_error_flag;
extern ADCDriver *adcp_err;

#if USE_ADC_WATCHDOG

/* Watchdog callback type definition
 */
typedef void (*adc_watchdog_callback)(void);

/* Watchdog register function
 *
 * @param adc adc bank to monitor
 * @param channel adc channel to monitor
 * @param vmin low threshhold for callback trigger
 * @param cb callback function call within ISR locked zone
 */
extern void register_adc_watchdog(ADCDriver *adc, adc_channels_num_t channel, adcsample_t vmin,
                                  adc_watchdog_callback cb);

#endif



#endif /* ADC_ARCH_H */
