/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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


#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "modem.h"
#include "timer.h"

const uint8_t *msg = "ap modem alive\n"; 

int main( void ) {
  timer_init();
  modem_init();
  sei();
  while (1) {
    if (timer_periodic()) {
      uint8_t i = 0;
      while (msg[i]) {
	MODEM_PUT_1_BYTE(msg[i]);
	i++;
      }
      MODEM_CHECK_RUNNING();
    }
  }
  return 0;
}
