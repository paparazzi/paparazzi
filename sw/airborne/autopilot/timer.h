/*
 * Paparazzi mcu0 timer functions
 *  
 * Copied from autopilot (autopilot.sf.net) thanx alot Trammell
 *
 * Copyright (C) 2002 Trammell Hudson <hudson@rotomotion.com>
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

#ifndef TIMER_H
#define TIMER_H

#include "std.h"
#include <avr/signal.h>
#include <avr/io.h>


/*
 * Enable Timer1 (16-bit) running at Clk/1 for the global system
 * clock.  This will be used for computing the servo pulse widths,
 * PPM decoding, etc.
 *
 * Low frequency periodic tasks will be signaled by timer 0
 * running at Clk/1024.  For 16 Mhz clock, this will be every
 * 262144 microseconds, or 61 Hz.
 */
static inline void timer_init( void ) {

  /* Timer0: Modem clock is started in modem.h in ctc mode*/

  /* Timer1 @ Clk/1: System clock, ppm and servos */
  TCCR1A		= 0x00;
  TCCR1B		= 0x01;

  /* Timer2 @ Clk/1024: Periodic clock */
  TCCR2		= 0x05;
}


/*
 * Retrieve the current time from the global clock in Timer1,
 * disabling interrupts to avoid stomping on the TEMP register.
 * If interrupts are already off, the non_atomic form can be used.
 */
static inline uint16_t
timer_now( void )
{
  return TCNT1;
}

static inline uint16_t
timer_now_non_atomic( void )
{
  return TCNT1L;
}


/*
 *  Periodic tasks occur when Timer2 overflows.  Check and unset
 * the overflow bit.  We cycle through four possible periodic states,
 * so each state occurs every 30 Hz.
 */
static inline bool_t
timer_periodic( void )
{
  if( !bit_is_set( TIFR, TOV2 ) )
    return FALSE;

  TIFR = 1 << TOV2;
  return TRUE;
}

#endif
