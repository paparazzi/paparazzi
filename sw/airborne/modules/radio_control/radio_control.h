/*
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
 *               2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 *
 */

/**
 * @file modules/radio_control/radio_control.h
 * Generic interface for radio control modules
 */

#ifndef RADIO_CONTROL_H
#define RADIO_CONTROL_H

#include "generated/airframe.h"
#include "paparazzi.h"

/* timeouts - for now assumes 60Hz periodic */
#ifndef RC_AVG_PERIOD
#define RC_AVG_PERIOD 8  /* TODO remove if IIR filter is used */
#endif
#ifndef RC_LOST_TIME
#define RC_LOST_TIME 30  /* 500ms with a 60Hz timer */
#endif
#ifndef RC_REALLY_LOST_TIME
#define RC_REALLY_LOST_TIME 60 /* ~1s */
#endif
/* Number of valid frames before going back to RC OK */
#ifndef RC_OK_CPT
#define RC_OK_CPT 15
#endif

#define RC_OK          0
#define RC_LOST        1
#define RC_REALLY_LOST 2

// if not redefined, use default number of channels
// value should be large enough for all implementation by default
#ifndef RADIO_CONTROL_NB_CHANNEL
#define RADIO_CONTROL_NB_CHANNEL 32
#endif


struct RadioControl {
  uint8_t status;
  uint8_t time_since_last_frame;
  uint8_t radio_ok_cpt;
  uint8_t frame_rate;
  uint8_t frame_cpt;
  uint8_t nb_channel;
  pprz_t  values[RADIO_CONTROL_NB_CHANNEL];
};

extern struct RadioControl radio_control;

// For easy access in command_laws
#define RadioControlValues(_chan) radio_control.values[_chan]

// Test is radio is reall lost
#define RadioControlIsLost() (radio_control.status == RC_REALLY_LOST)

/** Set a radio control channel value
 * @param idx rc channel index
 * @param value new value
 */
static inline void radio_control_set(uint8_t idx, pprz_t value)
{
  if (idx < radio_control.nb_channel) {
    // Bound value ???
    radio_control.values[idx] = value;
  }
}

/** Get a radio control channel value
 * @param idx rc channel index
 * @return current value, 0 if index is invalid
 */
static inline pprz_t radio_control_get(uint8_t idx)
{
  if (idx < radio_control.nb_channel) {
    return radio_control.values[idx];
  }
  return 0;
}


extern void radio_control_init(void);
extern void radio_control_periodic_task(void);

#endif /* RADIO_CONTROL_H */
