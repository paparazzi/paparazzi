/*
 * $Id$
 *  
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
 *  \brief arch independant LED (Light Emitting Diodes) API
 *
 *
 */

#ifndef LED_H
#define LED_H

#ifdef LED

#include "led_hw.h"

static inline void led_init ( void ) {
#ifdef LED_1_BANK
  LED_INIT(1);
  LED_OFF(1);
#endif /* LED_1_BANK */

#ifdef LED_2_BANK
  LED_INIT(2);
  LED_OFF(2);
#endif /* LED_2_BANK */

#ifdef LED_3_BANK
  LED_INIT(3);
  LED_OFF(3);
#endif /* LED_3_BANK */

#ifdef LED_4_BANK
  LED_INIT(4);
  LED_OFF(4);
#endif /* LED_4_BANK */

#ifdef LED_5_BANK
  LED_INIT(5);
  LED_OFF(5);
#endif /* LED_5_BANK */

#ifdef LED_6_BANK
  LED_INIT(6);
  LED_OFF(6);
#endif /* LED_6_BANK */

#ifdef LED_7_BANK
  LED_INIT(7);
  LED_OFF(7);
#endif /* LED_7_BANK */

#ifdef LED_8_BANK
  LED_INIT(8);
  LED_OFF(8);
#endif /* LED_8_BANK */
}

#else /* LED */
#define LED_ON(i) {}
#define LED_OFF(i) {}
#define LED_TOGGLE(i) {}
#endif /* LED */

#endif /* LED_H */
