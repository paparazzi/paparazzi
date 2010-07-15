/*
 * $Id:$
 *
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
 */


#include "init_hw.h"
#include "sys_time.h"
#include "can.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

uint16_t servos[4];

int main(void) {
  main_init();

  while(1) {
	  if (sys_time_periodic())
		  main_periodic_task();
	  main_event_task();
  }

  return 0;
}

static inline void main_init( void ) {
	hw_init();
	sys_time_init();
	can_init();
}

static inline void main_periodic_task( void ) {

	can_transmit(1, 0, (uint8_t *)servos, 1);

	LED_TOGGLE(2);

	LED_PERIODIC();
}



static inline void main_event_task( void ) {

}
