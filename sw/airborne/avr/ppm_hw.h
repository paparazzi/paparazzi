/* $Id$
 *
 * Decoder for the trainer ports or hacked receivers for both
 * Futaba and JR formats.  The ppm_valid flag is set whenever
 * a valid frame is received.
 *
 * Pulse widths are stored as unscaled 16-bit values in ppm_pulses[].
 * If you require actual microsecond values, divide by CLOCK.
 * For an 8 Mhz clock and typical servo values, these will range
 * from 0x1F00 to 0x4000.
 * 
 * Copied from autopilot (autopilot.sf.net) thanx alot Trammell
 *
 * (c) 2002 Trammell Hudson <hudson@rotomotion.com>
 * (c) 2003 Pascal Brisset, Antoine Drouin
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

#ifndef PPM_HW_H
#define PPM_HW_H

#ifdef FBW

#include <inttypes.h>
#include <avr/io.h>

#include CONFIG

/*
 * PPM pulses are falling edge clocked on the ICP, which records
 * the state of the global clock.  We do not use any noise
 * canceling features.
 *
 * JR might be rising edge clocked; set that as an option
 */
static inline void
ppm_init( void )
{
#if defined RADIO_CONTROL_TYPE && RADIO_CONTROL_TYPE == RC_FUTABA
  cbi( TCCR1B, ICES1 );
#elif defined RADIO_CONTROL_TYPE && RADIO_CONTROL_TYPE == RC_JR
  sbi( TCCR1B, ICES1 );
#else
#error "ppm_hw.h: Unknown RADIO_CONTROL_TYPE"
#endif
  /* No noise cancelation */
  sbi( TCCR1B, ICNC1 );
  
  /* Set ICP to input, no internal pull up */
  cbi( PPM_DDR, PPM_PIN);
  
  /* Enable interrupt on input capture */
  sbi( TIMSK, TICIE1 );

#ifdef TIMER1_TOP
  /* Enable timer1 overflow it. */
  /* needed to increase timer1 count to 16 bits in fast pwm mode (TIMER1_TOP rollover) */
  sbi( TIMSK, TOIE1 );
#endif
  ppm_valid = FALSE;
}

#endif /* FBW */

#endif /* PPM_HW_H */
