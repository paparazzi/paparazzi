/*
 * Copyright (C) 2009-2014 The Paparazzi Team
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/radio_control/spektrum.h
 *
 * Radio control spektrum interface.
 */

#ifndef RADIO_CONTROL_SPEKTRUM_H
#define RADIO_CONTROL_SPEKTRUM_H

#include "std.h"

/* Include channels information */
#include "spektrum_radio.h"

/* For now only two satellites are supported */
#define SPEKTRUM_SATELLITES_NB 2

/* Timings and maximums of Spektrum DSM protocol */
#define SPEKTRUM_FRAME_LEN 16             ///< 16 bytes in a standard frame
#define SPEKTRUM_CHANNELS_PER_FRAME 7     ///< Maximum amount of RC channels per frame
#define SPEKTRUM_MAX_FRAMES 2             ///< Maximum amount of RC frames containing different channels
#define SPEKTRUM_MAX_CHANNELS (SPEKTRUM_CHANNELS_PER_FRAME * SPEKTRUM_MAX_FRAMES)
#define SPEKTRUM_MIN_FRAME_SPACE  7       ///< Minum amount of time between frames (7ms), in fact either 11 or 22 ms

/* Per satellite we keep track of data */
struct spektrum_sat_t {
  bool valid;                             ///< True when we received a packet else false
  uint32_t timer;                         ///< Timer to keep track of the UART synchronisation
  uint8_t lost_frame_cnt;                 ///< Amount of RC frames lost
  uint8_t buf[SPEKTRUM_FRAME_LEN];        ///< input buffer
  uint8_t idx;                            ///< input buffer index
  int16_t values[SPEKTRUM_MAX_CHANNELS];  ///< RC channel values
};

/* Main spektrum structure */
struct spektrum_t {
  bool valid;                             ///< True when we received a packet else false
  uint8_t tx_type;                        ///< Transmitter type encoded (see wiki)
  int8_t signs[SPEKTRUM_NB_CHANNEL];      ///< Signs for the RC channels
  struct spektrum_sat_t satellites[SPEKTRUM_SATELLITES_NB]; ///< All the satellites connected
};

/* External functions */
extern void spektrum_init(void);
extern void spektrum_event(void);
extern void spektrum_try_bind(void);

#endif /* RADIO_CONTROL_SPEKTRUM_H */
