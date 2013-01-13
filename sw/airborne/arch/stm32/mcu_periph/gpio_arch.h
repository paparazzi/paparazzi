/*
 * Copyright (C) 2012 Piotr Esden-Tempski <piotr@esden.net>
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

/**
 * @file arch/stm32/mcu_periph/gpio_arch.h
 * @ingroup stm32_arch
 *
 * Handling of GPIOs for STM32.
 */

#ifndef MY_GPIO_ARCH_H
#define MY_GPIO_ARCH_H

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>

#define GPIO_ARCH_SET_SPI_CS_HIGH()					\
{									\
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);        \
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,                         \
	        GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);                      \
  gpio_set(GPIOB, GPIO12);                                              \
}


#endif /* MY_GPIO_ARCH_H */
