/*
 * $Id$
 *  
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

#include <stm32/flash.h>
#include <stm32/misc.h>

#include BOARD_CONFIG
#include "init_hw.h"

void Delay(__IO uint32_t nCount);

int main(void) {

  hw_init();
  while (1) {
    LED_ON(1);
    LED_ON(3);
    LED_ON(5);
    LED_ON(7);
#if 1
    LED_OFF(0);
    LED_OFF(2);
    LED_OFF(4);
    LED_OFF(6);
#endif
    LED_PERIODIC();
    Delay(1000000);
    LED_OFF(1);
    LED_OFF(3);
    LED_OFF(5);
    LED_OFF(7);
#if 1
    LED_ON(0);
    LED_ON(2);
    LED_ON(4);
    LED_ON(6);
#endif
    LED_PERIODIC();
    Delay(1000000);

  };
  return 0;
}

void Delay(__IO uint32_t nCount) {
  for(; nCount != 0; nCount--);
}


