/*
 * $Id$
 *  
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 */

#include "booz_radio_control.h"

#include "led.h"

struct RadioControl radio_control;

void radio_control_init(void) {
  uint8_t i;
  for (i=0; i<RADIO_CONTROL_NB_CHANNEL; i++)
    radio_control.values[i] = 0;
  radio_control.status = RADIO_CONTROL_REALLY_LOST;
  radio_control.time_since_last_frame = RADIO_CONTROL_REALLY_LOST_TIME;
  radio_control.radio_ok_cpt = 0;
  radio_control.frame_rate = 0;
  radio_control.frame_cpt = 0;
  radio_control_impl_init();
}


void radio_control_periodic(void) {
 
  /* compute frame rate */
  RunOnceEvery(60, {
      radio_control.frame_rate = radio_control.frame_cpt;
      radio_control.frame_cpt = 0;
    });
  
  /* check for timeouts */
  if (radio_control.time_since_last_frame >= RADIO_CONTROL_REALLY_LOST_TIME) {
    radio_control.status = RADIO_CONTROL_REALLY_LOST;
  } 
  else {
    if (radio_control.time_since_last_frame >= RADIO_CONTROL_LOST_TIME) {
      radio_control.status = RADIO_CONTROL_LOST;
      radio_control.radio_ok_cpt = RADIO_CONTROL_OK_CPT;
    }
    radio_control.time_since_last_frame++;
  }

  /* sigal status with LEDs */
#if defined RADIO_CONTROL_LED
  if (radio_control.status == RADIO_CONTROL_OK) {
    LED_ON(RADIO_CONTROL_LED);
  }
  else {
    LED_OFF(RADIO_CONTROL_LED);
  }
#endif

}
