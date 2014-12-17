/*
 * Copyright (C) 2009  Martin Mueller
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

/** \file blitzer.c
 *  \brief LED flasher
 *
 *   Flashes a LED connected to PB1 (pin 6) and PB2 (pin 7) of an
 * ATTINY25 through an IRML2502 transistor, PBx low -> LED off.
 *
 * fuse high byte: 0xdf, fuse low byte: 0x62
 *
 */

#include <avr/pgmspace.h>

void wait(int msec_time)
{
  volatile unsigned short cnta, cntb;

  /* roughly based on internal oscillator with divider by 8 enabled */
  for (cnta = 0; cnta < msec_time; cnta++) {
    for (cntb = 0; cntb < 38; cntb++) { cntb = cntb; }
  }
}

int main(void)
{
  DDRB |= (1 << PB2);   // PB2 output
  DDRB |= (1 << PB1);   // PB1 output
  DDRB &= ~(1 << PB0);  // PB0 input

  while (1) {
    PORTB |= (1 << PB2);
    PORTB |= (1 << PB1);
    wait(25);
    PORTB &= ~(1 << PB2);
    PORTB &= ~(1 << PB1);
    wait(110);
    PORTB |= (1 << PB2);
    PORTB |= (1 << PB1);
    wait(25);
    PORTB &= ~(1 << PB2);
    PORTB &= ~(1 << PB1);
    wait(780);
  }

  return (0);
}
