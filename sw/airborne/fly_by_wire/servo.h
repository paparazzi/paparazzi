/*  $Id$
 *
 * Copied from autopilot (autopilot.sf.net) thanx alot Trammell
 * (c) 2002 Trammell Hudson <hudson@rotomotion.com>
 * (c) 2003 Pascal Brisset, Antoine Drouin
 *
 * This is the new decade counter based servo driving code.  It uses
 * one 16-bit output compare registers to determine when the regular
 * servo clock line should be toggled, causing the output to move to the
 * next servo.  The other 16-bit output compare is used to drive a
 * JR or Futaba compatible high-speed digital servo.
 *
 * User visibile routines:
 *
 * - servo_init();
 *
 * Call once at the start of the program to bring the servos online
 * and start the external decade counters.  This will also start the
 * high speed servo.
 *
 * - servo_make_pulse_width( length );
 *
 * Converts a position value between 0 and 65536 to actual pulsewidth.  0 is
 * all the way left (1.0 ms pulse) and 65536 is all the way right (2.0 ms
 * pulse). Use it like this:
 *
 * servo_widths[ i ] = servo_make_pulse_width( val )
 *
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

#ifndef SERVO_H
#define SERVO_H

#include <inttypes.h>
#include "timer.h"

extern void servo_init( void );
extern void servo_set(const pprz_t values[]);
extern void servo_set_one(uint8_t servo, uint16_t value_us);
extern void servo_transmit(void);


#endif /*  SERVO_H */
