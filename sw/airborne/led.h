/*
 * Copyright (C) 2003-2005  Antoine Drouin
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

/** \file led.h
 *  \brief arch independent LED (Light Emitting Diodes) API
 *
 *
 */

#ifndef LED_H
#define LED_H

#if defined USE_LED

#include "led_hw.h"

static inline void led_init ( void ) {
#if USE_LED_1
  LED_INIT(1);
  LED_OFF(1);
#endif /* LED_1 */

#if USE_LED_2
  LED_INIT(2);
  LED_OFF(2);
#endif /* LED_2 */

#if USE_LED_3
  LED_INIT(3);
  LED_OFF(3);
#endif /* LED_3 */

#if USE_LED_4
  LED_INIT(4);
  LED_OFF(4);
#endif /* LED_4 */

#if USE_LED_5
  LED_INIT(5);
  LED_OFF(5);
#endif /* LED_5 */

#if USE_LED_6
  LED_INIT(6);
  LED_OFF(6);
#endif /* LED_6 */

#if USE_LED_7
  LED_INIT(7);
  LED_OFF(7);
#endif /* LED_7 */

#if USE_LED_8
  LED_INIT(8);
  LED_OFF(8);
#endif /* LED_8 */

#if USE_LED_9
  LED_INIT(9);
  LED_OFF(9);
#endif /* LED_9 */

#if USE_LED_10
  LED_INIT(10);
  LED_OFF(10);
#endif /* LED_10 */

#ifdef USE_LED_BODY
  LED_INIT(BODY);
  LED_OFF(BODY);
#endif /* LED_BODY */

#if USE_LED_12
  LED_INIT(12);
  LED_OFF(12);
#endif /* LED_12 */
}

#define _LED_AVAILABLE(i) USE_LED_ ## i
#define LED_AVAILABLE(i) _LED_AVAILABLE(i)

#else /* USE_LED */
static inline void led_init ( void ) {}
#define LED_ON(i) {}
#define LED_OFF(i) {}
#define LED_TOGGLE(i) {}
#define LED_PERIODIC() {}
#define LED_AVAILABLE(i) 0
#endif /* USE_LED */

#endif /* LED_H */
