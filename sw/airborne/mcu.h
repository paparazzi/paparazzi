/*
 * Copyright (C) 2010-2012 The Paparazzi team
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
 * @file mcu.h
 * @brief Arch independent mcu ( Micro Controller Unit ) utilities.
 */

#ifndef MCU_H
#define MCU_H


#include <mcu_arch.h>

/**
 * @defgroup mcu_periph MCU Peripherals
 * @{
 */

/**
 * Microcontroller peripherals initialization.
 * This function is responisble for setting up the microcontroller
 * after Reset.
 */
extern void mcu_init(void);

/**
 * MCU event functions.
 * Calls the event functions of used peripherals like i2c, uart, etc.
 */
extern void mcu_event(void);

/**
 * Optional board init function called at the start of mcu_init().
 */
extern void board_init(void);

/**
 * Optional board init function called at the end of mcu_init().
 */
extern void board_init2(void);

/** @}*/

#endif /* MCU_H */


