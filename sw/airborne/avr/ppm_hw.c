/* $Id$
 * Copied from autopilot (autopilot.sf.net) thanx alot Trammell
 *
 * (c) 2003 Trammell Hudson <hudson@rotomotion.com>
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

#if (__GNUC__ == 3)
#include <avr/signal.h>
#endif

#include <avr/interrupt.h>
#include "ppm.h"
#include "sys_time.h"

/*
 * Pulse width is computed as the difference between now and the
 * previous pulse.  If no pulse has been received between then and
 * now, the time of the last pulse will be equal to the last pulse
 * we measured.  Unfortunately, the Input Capture Flag (ICF1) will
 * not be set since the interrupt routine disables it.
 * 
 * Sync pulses are timed with Timer2, which runs at Clk/1024.  This
 * is slow enough at both 4 and 8 Mhz to measure the lengthy (10ms
 * or longer) pulse.
 *
 * Otherwise, compute the pulse width with the 16-bit timer1,
 * push the pulse width onto the stack and increment the
 * pulse counter until we have received eight pulses.
 */

uint16_t ppm_pulses[ PPM_NB_PULSES ];
volatile bool_t ppm_valid;

/* MC3030, Trame PPM7: 25ms, 10.4 au neutre, 
   sync pulse = 16.2ms with low value on every channels */

#define RestartPpmCycle() { state = 0;  sync_start = TCNT2; return; }

#ifdef TIMER1_TOP
static volatile uint16_t tmr1_ov_cnt = 0;
SIGNAL(SIG_OVERFLOW1) {
  tmr1_ov_cnt += TIMER1_TOP;
  return;
}
#endif

SIGNAL( SIG_INPUT_CAPTURE1 )
{
  static uint16_t	last;
  uint16_t		this;
  uint16_t		width;
  static uint8_t	state = 0;
  static uint8_t	sync_start;

  this		= ICR1;
#ifdef TIMER1_TOP
  this += tmr1_ov_cnt;
#endif
  width		= this - last;
  last		= this;
  
  if( state == 0 ) {
    uint8_t end = TCNT2;
    uint8_t diff = (end - sync_start);
    sync_start = end;

    /* The frame period of the mc3030 seems to be 25ms. 
     * One pulse lasts from 1.05ms to 2.150ms.
     * Sync pulse is at least 7ms : (7000*CLOCK)/1024 = 109
     */
    if( diff > LONG_SYS_TICS_OF_USEC(PPM_SYNC_MIN_LEN) &&
	diff < LONG_SYS_TICS_OF_USEC(PPM_SYNC_MAX_LEN)) {
      //  if( diff > (uint8_t)(((uint32_t)7000ul*16)/1024ul) ) {
      state = 1;
    }
  } 
  else {
    /* Read a data pulses */
    if( width > SYS_TICS_OF_USEC(PPM_DATA_MAX_LEN) || 
	width < SYS_TICS_OF_USEC(PPM_DATA_MIN_LEN))
      RestartPpmCycle();
    ppm_pulses[state - 1] = width;

    if (state >= PPM_NB_PULSES) {
      ppm_valid	= TRUE;
      RestartPpmCycle();
    } else 
      state++;
  }
  return;
}
