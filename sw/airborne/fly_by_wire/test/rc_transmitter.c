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

#include "timer.h"
#include "ppm.h"
#include "radio.h"

#include "uart.h"

inline void transmit_radio(void) {
  uint8_t ctl;
  uart_transmit((uint8_t)0); uart_transmit((uint8_t)0);
  for(ctl = 0; ctl < RADIO_CTL_NB; ctl++) {
    extern uint16_t ppm_pulses[];
    uint16_t x = ppm_pulses[ctl] / 16;
    uart_transmit((uint8_t)(x >> 8));
    uart_transmit((uint8_t)(x & 0xff)); 
  }
  uart_transmit((uint8_t)'\n');
  //  uart_transmit('A');uart_transmit('\n');
}

int main( void ) {
  uart_init_tx();
  uart_print_string("Calib_radio Booting $Id$\n");
  timer_init();
  ppm_init();
  sei();
  int n = 0;
  while( 1 ) {
    if( ppm_valid ) {
      ppm_valid = FALSE;
      n++;
      if (n == 4) {
	n = 0;
	transmit_radio();
      }
    }

    // A rajouter pour envoyer un message de vie quand la radio n'est pas recue
    //    if(timer_periodic()) {
    //      uart_transmit('B');uart_transmit('\n');
    //    }
  } 
  return 0;
}
