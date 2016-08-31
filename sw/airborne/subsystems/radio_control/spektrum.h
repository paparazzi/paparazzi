/*
 * Copyright (C) 2009-2014 The Paparazzi Team
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
 * @file subsystems/radio_control/spektrum.h
 *
 * Radio control spektrum interface.
 */

#ifndef RADIO_CONTROL_SPEKTRUM_H
#define RADIO_CONTROL_SPEKTRUM_H

#include "std.h"

/* Include channels information */
#include "spektrum_radio.h"

/* Automatically determine amount of satellites */
#if defined SPEKTRUM_UART_SAT4
#define SPEKTRUM_SATELLITES_NB 4
#elif defined SPEKTRUM_UART_SAT3
#define SPEKTRUM_SATELLITES_NB 3
#elif defined SPEKTRUM_UART_SAT2
#define SPEKTRUM_SATELLITES_NB 2
#elif defined SPEKTRUM_UART_SAT1
#define SPEKTRUM_SATELLITES_NB 1
#else
#error "You must at least define 1 Spektrum satellite receiver. (SPEKTRUM_UART_SAT1 must be defined)"
#endif

/* Timings and maximums of Spektrum DSM protocol */
#define SPEKTRUM_CHANNELS_PER_FRAME 7   ///< Maximum amount of RC channels per frame
#define SPEKTRUM_MAX_FRAMES 2           ///< Maximum amount of RC frames containing different channels
#define SPEKTRUM_MAX_CHANNELS (SPEKTRUM_CHANNELS_PER_FRAME * SPEKTRUM_MAX_FRAMES)
#define SPEKTRUM_GUESS_FRAMES 6         ///< Amount of frames needed for guessing TX type
#define SPEKTRUM_MIN_FRAME_SPACE  7     ///< Minum amount of time between frames (7ms)
#define SPEKTRUM_MAX_FRAME_TIME  4      ///< Maximum amount a frame takes to receive (4ms)

/* Set the event function to the correct */
#define RadioControlEvent(_received_frame_handler) spektrum_event(_received_frame_handler)

/* Different binding states */
enum spektrum_bind_state {
  SPEKTRUM_BIND_NONE,       ///< No binding happening
  SPEKTRUM_BIND_DIRECT,     ///< Bind directly
  SPEKTRUM_BIND_BOOT,       ///< Bind on next boot
};

/* Per satellite we keep track of data */
struct spektrum_sat_t {
  bool valid;                 ///< True when we received a packet else false
  uint32_t timer;             ///< Timer to keep track of the UART synchronisation
  struct uart_periph *dev;    ///< UART device which the satellite is connected to
  uint8_t lost_frame_cnt;     ///< Amount of RC frames lost
  uint8_t tx_type;            ///< Transmitter type encoded (see wiki)
  int16_t values[SPEKTRUM_MAX_CHANNELS];  ///< RC channel values
};

/* Main spektrum structure */
struct spektrum_t {
  bool valid;                               ///< True when we received a packet else false
  int8_t signs[RADIO_CONTROL_NB_CHANNEL];   ///< Signs for the RC channels
  uint8_t bind_pulses;                      ///< Amount of bind pulses
  enum spektrum_bind_state bind_state;      ///< Binding states
  struct spektrum_sat_t satellites[SPEKTRUM_SATELLITES_NB]; ///< All the satellites connected
};

/* External functions */
extern void spektrum_event(void (*_received_frame_handler)(void));
extern void spektrum_set_bind(uint8_t val);

#endif /* RADIO_CONTROL_SPEKTRUM_H */
