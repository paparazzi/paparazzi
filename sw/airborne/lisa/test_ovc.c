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
#include "sys_time.h"

static inline void main_periodic( void );

int main(void) {

  hw_init();
  sys_time_init();

  while (1) {

    if (sys_time_periodic())
      main_periodic();
  
  }
  
  return 0;
}


static inline void main_periodic( void ) {
  //  LED_TOGGLE(1);

  uart2_transmit('a');
  uart2_transmit('b');
  uart2_transmit('b');
  uart2_transmit('\n');
}

