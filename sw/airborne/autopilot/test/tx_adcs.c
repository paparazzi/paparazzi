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
#include <avr/interrupt.h>
#include <avr/io.h>
#include "timer.h"
#include "uart.h"
#include "adc.h"

static struct adc_buf buffers[NB_ADC];

void transmit_adc(void) {
  uint8_t i;
  uart0_transmit((uint8_t)0); uart0_transmit((uint8_t)0);
  for(i = 0; i < NB_ADC; i++) {
    uint16_t value = buffers[i].sum / AV_NB_SAMPLE;
    uart0_transmit((uint8_t)(value >> 8));
    uart0_transmit((uint8_t)(value & 0xff));
  }
  uart0_transmit((uint8_t)'\n');
}

int main( void ) {
  uint8_t i;
  uart0_init();
  timer_init();
  adc_init();
  for(i = 0; i < NB_ADC; i++)
    adc_buf_channel(i, &buffers[i]);
  sei();

  while( 1 ) {
    static uint8_t _1Hz = 0;

    if(timer_periodic()) {
      _1Hz++;
      if (_1Hz == 60) {
	_1Hz = 0;
	transmit_adc();
      }
    } 
  }
  return 0;
}
