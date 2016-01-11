/*
 * Copyright (C) Bart Slinger
 *
 * This file is part of paparazzi
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
 * @file "modules/adc_expansion_uart/adc_expansion_uart.h"
 * @author Bart Slinger
 * Get analog signals from expansion board to main autopilot over uart
 */

#ifndef ADC_EXPANSION_UART_H
#define ADC_EXPANSION_UART_H

#include "std.h"

extern uint16_t adc_uart_values[3];

extern void adc_expansion_uart_init(void);
extern void adc_expansion_uart_process_msg(void);

#endif

