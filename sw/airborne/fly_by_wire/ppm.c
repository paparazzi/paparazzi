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

#include "radio.h"
#include "ppm.h"

#define AVERAGING_PERIOD (PPM_FREQ/4)

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
pprz_t last_radio[ PPM_NB_PULSES ];
pprz_t avg_last_radio[ PPM_NB_PULSES ];
bool_t last_radio_contains_avg_channels = FALSE;
volatile bool_t ppm_valid;
uint8_t ppm_status;
uint16_t ppm_time_since_last_valid;

/* MC3030, Trame PPM7: 25ms, 10.4 au neutre, 
   sync pulse = 16.2ms with low value on every channels */


    
  
#define RestartPpmCycle() { state = 0;  sync_start = TCNT2; return; }

SIGNAL( SIG_INPUT_CAPTURE1 )
{
  static uint16_t		last;
  uint16_t		this;
  uint16_t		width;
  static uint8_t		state;
  static uint8_t		sync_start;

  this		= ICR1;
  width		= this - last;
  last		= this;
  
  if( state == 0 ) {
    uint8_t	end = inp( TCNT2 );
    uint8_t diff = (end - sync_start);
    sync_start = end;

    /* The frame period of the mc3030 seems to be 25ms. 
     * One pulse lasts from 1.05ms to 2.150ms.
     * Sync pulse is at least 5.5ms : (5500*CLOCK)/1024 = 109
     */
    if( diff > (uint8_t)(((uint32_t)(5500ul*CLOCK))/1024ul) ) {
      state = 1;
    }
  } 
  else {
    /* Read a data pulses */
    if( width < 700ul*CLOCK || width > 2300ul*CLOCK)
      RestartPpmCycle();
    ppm_pulses[state - 1] = width;

    if (state >= PPM_NB_PULSES) {
      ppm_valid	= 1;
      RestartPpmCycle();
    } else 
      state++;
  }
  return;
}

#define Int16FromPulse(i) (int16_t)((ppm_pulses[(i)] - PpmOfUs(((uint16_t[])RADIO_NEUTRALS_US)[i]))* (2*MAX_PPRZ)/(PpmOfUs(((uint16_t[])RADIO_MAXS_US[i])-((uint16_t[])RADIO_MINS_US[i]))))
#define NewInt16FromPulse(i) (int16_t)((ppm_pulses[(i)] - ((uint16_t[])RADIO_NEUTRALS_PPM)[i])) * (float[])RADIO_TRAVEL_PPM[i]


/* Copy from the ppm receiving buffer to the buffer sent to mcu0 */
void last_radio_from_ppm() {
  static uint8_t avg_cpt = 0; /* Counter for averaging */
  uint8_t i;
  
  for(i = 0; i < RADIO_CTL_NB; i++) {
    int16_t pprz = NewInt16FromPulse(i);
    if (pprz > MAX_PPRZ)
      pprz = MAX_PPRZ;
    else if (pprz < MIN_PPRZ)
      pprz = MIN_PPRZ;

    if (i == RADIO_THROTTLE) {
      int16_t gaz = pprz/2;
      last_radio[i] = (gaz < 0 ? 0 : gaz);
    } else if (AveragedChannel(i)) {
      avg_last_radio[i] += pprz / AVERAGING_PERIOD;
    } else
      last_radio[i] = pprz;
  }

  avg_cpt++;
  if (avg_cpt == AVERAGING_PERIOD) {
    avg_cpt = 0;
    for(i = 0; i < RADIO_CTL_NB; i++)
      if (AveragedChannel(i)) {
	last_radio[i] = avg_last_radio[i];
	avg_last_radio[i] = 0;	
      }
    last_radio_contains_avg_channels = TRUE;
  }
}
