/*
 * Copyright (C) 2013 Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file subsystems/radio_control/superbitrf_rc.h
 * DSM2 and DSMX radio control implementation for the cyrf6936 2.4GHz radio chip trough SPI
 */

#ifndef RADIO_CONTROL_SUPERBITRF_RC_H
#define RADIO_CONTROL_SUPERBITRF_RC_H

#include "subsystems/datalink/superbitrf.h"

/* Theoretically you could have 14 channel over DSM2/DSMX */
#ifndef RADIO_CONTROL_NB_CHANNEL
#define RADIO_CONTROL_NB_CHANNEL 14
#endif

/* The channel ordering is always the same for DSM2 and DSMX */
#define RADIO_THROTTLE   0
#define RADIO_ROLL       1
#define RADIO_PITCH      2
#define RADIO_YAW        3
#define RADIO_GEAR       4
#define RADIO_FLAP       5
#define RADIO_AUX1       5
#define RADIO_AUX2       6
#define RADIO_AUX3       7
#define RADIO_AUX4       8
#define RADIO_AUX5       9
#define RADIO_AUX6       10
#define RADIO_AUX7       11
#define RADIO_AUX8       12
#define RADIO_AUX9       13

/* Map the MODE default to the gear switch */
#ifndef RADIO_MODE
#define RADIO_MODE       RADIO_GEAR
#endif

/* Macro that normalize superbitrf rc_values to radio values */
#define NormalizeRcDl(_in, _out, _count) {                              \
  uint8_t i;                                                            \
  for(i = 0; i < _count; i++) {                                         \
    if(i == RADIO_THROTTLE) {                                           \
    	_out[i] = (_in[i] + MAX_PPRZ) / 2;                              \
    	Bound(_out[i], 0, MAX_PPRZ);                                    \
    } else {                                                            \
    	_out[i] = -_in[i];                                              \
    	Bound(_out[i], MIN_PPRZ, MAX_PPRZ);                             \
    }                                                                   \
  }                                                                     \
}

/* The radio control event handler */
#define RadioControlEvent(_received_frame_handler) {                \
  cyrf6936_event(&superbitrf.cyrf6936);                             \
  superbitrf_event();                                               \
  if(superbitrf.rc_frame_available) {                               \
	  radio_control.frame_cpt++;                                    \
      radio_control.time_since_last_frame = 0;                      \
      radio_control.radio_ok_cpt = 0;                               \
      radio_control.status = RC_OK;                                 \
      NormalizeRcDl(superbitrf.rc_values,radio_control.values       \
        ,superbitrf.num_channels);                                  \
      _received_frame_handler();                                    \
      superbitrf.rc_frame_available = FALSE;                        \
  }                                                                 \
}

#endif /* RADIO_CONTROL_SUPERBITRF_RC_H */
