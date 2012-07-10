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

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

static inline void main_init( void );
static inline void main_periodic( void );

int main(void) {

  main_init();

  while (1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic();
  }
  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
}

static inline void main_periodic( void ) {
  char ch;

  Uart1Transmit('a');
  Uart2Transmit('b');
  Uart3Transmit('c');

  if (Uart1ChAvailable()) {
	ch = Uart1Getch();
	if (ch == 'a') {
		LED_OFF(0);
		LED_ON(1);
	} else {
		LED_ON(0);
		LED_ON(1);
	}
  } else {
		LED_ON(0);
		LED_OFF(1);
  }

  if (Uart2ChAvailable()) {
	ch = Uart2Getch();
	if (ch == 'b') {
		LED_OFF(2);
		LED_ON(3);
	} else {
		LED_ON(2);
		LED_ON(3);
	}
  } else {
		LED_ON(2);
		LED_OFF(3);
  }

  if (Uart3ChAvailable()) {
	ch = Uart3Getch();
	if (ch == 'c') {
		LED_OFF(4);
		LED_ON(5);
	} else {
		LED_ON(4);
		LED_ON(5);
	}
  } else {
		LED_ON(4);
		LED_OFF(5);
  }

  LED_PERIODIC();
}



