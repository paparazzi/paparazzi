/*
 *
 * Copyright (C) 2009-2013 The Paparazzi Team
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
 * @file arch/linux/led_hw.h
 * linux arch dependant LED macros.
 */

#ifndef LED_HW_H_
#define LED_HW_H_

#include <stdint.h>
#include BOARD_CONFIG

#if defined BOARD_ARDRONE2_SDK || defined BOARD_ARDRONE2_RAW
extern uint32_t led_hw_values;
#define LED_INIT(i) { led_hw_values &= ~(1<<i); }
#define LED_ON(i) { led_hw_values |= (1<<i); }
#define LED_OFF(i) { led_hw_values &= ~(1<<i); }
#define LED_TOGGLE(i) { led_hw_values ^= (1<<i); }
#else
#define LED_INIT(i)   {}
#define LED_ON(i)     {}
#define LED_OFF(i)    {}
#define LED_TOGGLE(i) {}
#endif

#define LED_PERIODIC() {}

#endif /* LED_HW_H_ */
