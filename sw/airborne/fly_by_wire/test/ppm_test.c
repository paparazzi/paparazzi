/* $Id$
 * Copied from autopilot (autopilot.sf.net) thanx alot Trammell
 *
 * (c) 2005 Pascal Brisset, Antoine Drouin
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

#include "ppm_test.h"

/*
 * Sync pulses are timed with Timer2, which runs at Clk/1024.  This
 * is slow enough at 16MHz to measure up to a 16ms sync pulse.
 *
 * Otherwise, compute the pulse width with the 16-bit timer1,
 *
 */

uint16_t ppm_pulses[ PPM_MAX_PULSES ];
uint16_t ppm_pulses_buf[ PPM_MAX_PULSES ];
volatile bool_t ppm_available;
uint16_t ppm_time_since_last_valid;
uint8_t  ppm_status;
uint8_t  ppm_nb_received_channel;
uint8_t  ppm_sync_len;

SIGNAL( SIG_INPUT_CAPTURE1 )
{
  /* t1 is running @ CLOCK */
  static uint16_t	last_t1;
  uint16_t		cur_t1;
  uint16_t		data_len;
  /* t2 is running @ CLOCK/1024 */
  static uint8_t	last_t2;
  uint8_t               cur_t2;
  uint8_t               sync_len;

  static uint8_t	data_pulse_idx;

  /* compute elapsed timer1 ticks */
  /* allows high resolution on data pulses */
  cur_t1		= ICR1;
  data_len		= cur_t1 - last_t1;
  last_t1		= cur_t1;
 
  /* compute elapsed timer2 ticks */
  /* allows measuring long sync pulse */
  cur_t2                = TCNT2;
  sync_len              = cur_t2 - last_t2;
  last_t2               = cur_t2;

  /*  we have a pulse that is >= 3ms ; Let say it's a sync pulse*/
  if (sync_len >= (uint8_t)(((uint32_t)(3000ul*CLOCK))/1024ul)) {
    ppm_sync_len = sync_len;                  /* store the sync pulse len                   */
    ppm_nb_received_channel = data_pulse_idx; /* store how many channels we received before */
    if (ppm_nb_received_channel != 0)
      ppm_available = TRUE;      
    data_pulse_idx = 0;                       /* prepare to receive a new frame             */
  }
  /* let's assume we have a data pulse */
  else {
    if (data_pulse_idx <= PPM_MAX_PULSES) {
      ppm_pulses_buf[data_pulse_idx] = data_len;
      data_pulse_idx++;
    }
  }
}

void ppm_mainloop_task(void) {
  uint8_t i;
  ppm_available = FALSE;
  for (i=0; i<ppm_nb_received_channel; i++)
    ppm_pulses[i] = ppm_pulses_buf[i]/CLOCK;
}

