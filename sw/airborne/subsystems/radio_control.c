/*
 * Copyright (C) 2006-2014 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file subsystems/radio_control.c
 *
 * Implementation independant radio control handing.
 *
 */

#include "subsystems/radio_control.h"

struct RadioControl radio_control;

void radio_control_init(void)
{
  uint8_t i;
  for (i = 0; i < RADIO_CONTROL_NB_CHANNEL; i++) {
    radio_control.values[i] = 0;
  }
  radio_control.status = RC_REALLY_LOST;
  radio_control.time_since_last_frame = RC_REALLY_LOST_TIME;
  radio_control.radio_ok_cpt = 0;
  radio_control.frame_rate = 0;
  radio_control.frame_cpt = 0;
  radio_control_impl_init();
}

void radio_control_periodic_task(void)
{
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
  } else {
    LED_OFF(RADIO_CONTROL_LED);
  }
#endif

}
