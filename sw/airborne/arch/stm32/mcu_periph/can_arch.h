/*
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
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
 * @file arch/stm32/mcu_periph/can_arch.h
 * @ingroup stm32_arch
 *
 * Handling of CAN hardware for STM32.
 */

#ifndef MCU_PERIPH_STM32_CAN_ARCH_H
#define MCU_PERIPH_STM32_CAN_ARCH_H

void can_hw_init(void);
void usb_lp_can1_rx0_irq_handler(void);
int can_hw_transmit(uint32_t id, const uint8_t *buf, uint8_t len);

#endif /* MCU_PERIPH_STM32_CAN_ARCH_H */
