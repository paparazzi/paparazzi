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

/*
 
  4 DC motor controller with mosfets on OCR1A OCR3A OCR3B OCR3C

*/

#include <avr/io.h>
#include "command.h"
#include "std.h"
#include "airframe.h"

#define MAX_TICK 0x3FF
#define MOT_CTL_0 OCR3C
#define MOT_CTL_1 OCR1A
#define MOT_CTL_2 OCR3B
#define MOT_CTL_3 OCR3A

#define COMMAND_(i) MOT_CTL_ ## i
#define COMMAND(i) COMMAND_(i)
#define ChopServo(x,_a,b) (x > b ? b : x)
#define SERVOS_TICS_OF_USEC(s) (s)

const pprz_t failsafe_values[COMMANDS_NB] = COMMANDS_FAILSAFE;

void command_set(const pprz_t values[]) {
  CommandsSet(values); /*Generated from airframe.xml */
}
