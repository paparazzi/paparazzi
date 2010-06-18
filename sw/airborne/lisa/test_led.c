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

#define LED_PROGRAM_SIZE 26

const int LED_PROG_ON[LED_PROGRAM_SIZE] = {  3,  5,  7,  1,   -1, -1, -1, -1,    2,  4,  6,  0,     3,  5,  7,  1,    -1, -1, -1, -1,     -1, -1, -1, -1,     -1, -1   };
const int LED_PROG_OFF[LED_PROGRAM_SIZE] = {-1, -1, -1, -1,    3,  5,  7,  1,   -1, -1, -1, -1,    -1, -1, -1, -1,     3,  5,  7,  1,      2,  4,  6,  0,     -1, -1   };


int main(void) {
  int i = 0;
  hw_init();
  while (1) {
    for (i=0; i< LED_PROGRAM_SIZE; i++)
    {
      if (LED_PROG_ON[i] >= 0)
        LED_ON(LED_PROG_ON[i]);
      LED_PERIODIC();
      Delay(2000000);
      if (LED_PROG_OFF[i] >= 0)
        LED_OFF(LED_PROG_OFF[i]);
    }
  };
  return 0;
}

void Delay(__IO uint32_t nCount) {
  for(; nCount != 0; nCount--);
}


