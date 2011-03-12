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
#include "mcu.h"
#include "led.h"

void Delay(__IO uint32_t nCount);
void led_on(int i);
void led_off(int i);

#ifdef BOARD_LISA_L
#define LED_PROGRAM_SIZE 26

const int LED_PROG_ON[LED_PROGRAM_SIZE] = {  3,  5,  7,  1,   -1, -1, -1, -1,    2,  4,  6,  0,     3,  5,  7,  1,    -1, -1, -1, -1,     -1, -1, -1, -1,     -1, -1   };
const int LED_PROG_OFF[LED_PROGRAM_SIZE] = {-1, -1, -1, -1,    3,  5,  7,  1,   -1, -1, -1, -1,    -1, -1, -1, -1,     3,  5,  7,  1,      2,  4,  6,  0,     -1, -1   };
#endif

#ifdef BOARD_LISA_M
#define LED_PROGRAM_SIZE 10

const int LED_PROG_ON[LED_PROGRAM_SIZE] =  {  1,  2,   -1, -1,   -1,    2,  1,   -1, -1,   -1 };
const int LED_PROG_OFF[LED_PROGRAM_SIZE] = { -1, -1,    1,  2,   -1,   -1, -1,    2,  1,   -1 };
#endif

void led_on(int i) {
#ifdef BOARD_LISA_L
  LED_ON(i);
#endif

#ifdef BOARD_LISA_M
  switch (i) {
    case 1:
      LED_ON(1);
      break;
    case 2:
      LED_ON(2);
      break;
    default:
      /* ignore as we only have 2 led's for now on lisa/m */
      break;
  }
#endif
}

void led_off(int i) {
#ifdef BOARD_LISA_L
  LED_OFF(i);
#endif

#ifdef BOARD_LISA_M
  switch (i) {
    case 1:
      LED_OFF(1);
      break;
    case 2:
      LED_OFF(2);
      break;
    default:
      /* ignore as we only have 2 led's for now on lisa/m */
      break;
  }
#endif
}

int main(void) {
  int i = 0;
  mcu_init();
  while (1) {
    for (i=0; i< LED_PROGRAM_SIZE; i++)
    {
      if (LED_PROG_ON[i] >= 0)
        led_on(LED_PROG_ON[i]);
      LED_PERIODIC();
      Delay(2000000);
      if (LED_PROG_OFF[i] >= 0)
        led_off(LED_PROG_OFF[i]);
    }
  };
  return 0;
}

void Delay(__IO uint32_t nCount) {
  for(; nCount != 0; nCount--);
}


