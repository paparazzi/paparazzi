/*
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
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

#ifndef RADIO_CONTROL_H
#define RADIO_CONTROL_H

#if defined RADIO_CONTROL

#include "led.h"
#include "generated/airframe.h"
#include "paparazzi.h"

/* underlying hardware */
#include RADIO_CONTROL_TYPE_H
/* must be defined by underlying hardware */
extern void radio_control_impl_init(void);
/* RADIO_CONTROL_NB_CHANNEL has to be defined by the implementation */

/* timeouts - for now assumes 60Hz periodic */
#define RC_AVG_PERIOD 8  /* TODO remove if IIR filter is used */
#define RC_LOST_TIME 30  /* 500ms with a 60Hz timer */
#define RC_REALLY_LOST_TIME 60 /* ~1s */
/* Number of valid frames before going back to RC OK */
#define RC_OK_CPT 15

#define RC_OK          0
#define RC_LOST        1
#define RC_REALLY_LOST 2

struct RadioControl {
  uint8_t status;
  uint8_t time_since_last_frame;
  uint8_t radio_ok_cpt;
  uint8_t frame_rate;
  uint8_t frame_cpt;
  pprz_t  values[RADIO_CONTROL_NB_CHANNEL];
};

extern struct RadioControl radio_control;


/************* INIT ******************************************************/
static inline void radio_control_init ( void ) {
  uint8_t i;
  for (i=0; i<RADIO_CONTROL_NB_CHANNEL; i++)
    radio_control.values[i] = 0;
  radio_control.status = RC_REALLY_LOST;
  radio_control.time_since_last_frame = RC_REALLY_LOST_TIME;
  radio_control.radio_ok_cpt = 0;
  radio_control.frame_rate = 0;
  radio_control.frame_cpt = 0;
  radio_control_impl_init();
}

/************* PERIODIC ******************************************************/
static inline void radio_control_periodic_task ( void ) {
  static uint8_t _1Hz;
  _1Hz++;

  if (_1Hz >= 60) {
    _1Hz = 0;
    radio_control.frame_rate = radio_control.frame_cpt;
    radio_control.frame_cpt = 0;
  }

  if (radio_control.time_since_last_frame >= RC_REALLY_LOST_TIME) {
    radio_control.status = RC_REALLY_LOST;
  } else {
    if (radio_control.time_since_last_frame >= RC_LOST_TIME) {
      radio_control.status = RC_LOST;
      radio_control.radio_ok_cpt = RC_OK_CPT;
    }
    radio_control.time_since_last_frame++;
  }

#if defined RADIO_CONTROL_LED
  if (radio_control.status == RC_OK) {
    LED_ON(RADIO_CONTROL_LED);
  }
  else {
    LED_OFF(RADIO_CONTROL_LED);
  }
#endif

}

/********** EVENT ************************************************************/
// Implemented in radio_control/*.h


#endif /* RADIO_CONTROL */

#endif /* RADIO_CONTROL_H */
