/*  $Id$
 *
 * Copied from autopilot (autopilot.sf.net) thanx alot Trammell
 * (c) 2003-2005 Pascal Brisset, Antoine Drouin
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


/** Implementation of command.h */

#include <avr/io.h>
#include "command.h"

#define MAX_TICK 0x3FF
#define MOT_CTL_0 OCR3C
#define MOT_CTL_1  OCR1A
#define MOT_CTL_2  OCR3B
#define MOT_CTL_3 OCR3A

void command_init ( void ) {
  /* OC1A output    */
  DDRB |= _BV(5); 
  /* fast PWM, 10 bits */
  TCCR1A  |= _BV(WGM10) | _BV(WGM11) | _BV(COM1A1);
  TCCR1B  |= _BV(WGM12);
  /* OC3A, OC3B, OC3C outputs    */
  DDRE |= _BV(3) | _BV(4) | _BV(5);
  /* fast PWM : 10 bits */
  TCCR3A  |= _BV(WGM30) | _BV(WGM31) | _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1);
  TCCR3B  |= _BV(WGM32);
}

#define SERVO_NEUTRAL(_) 0
#define COMMAND_(i) MOT_CTL_ ## i
#define COMMAND(i) COMMAND_(i)
#define ChopServo(x) (x > MAX_TICK ? MAX_TICK : x)


void command_set(const pprz_t values[]) {
  CommandSet(values); /*Generated from airframe.xml */
}
