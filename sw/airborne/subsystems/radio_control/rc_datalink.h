/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
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

#ifndef RC_DATALINK_H
#define RC_DATALINK_H

#include "std.h"

#define RC_DL_NB_CHANNEL 5
#define RADIO_CONTROL_NB_CHANNEL RC_DL_NB_CHANNEL

/**
 * Redefining RADIO_*
 * Do not use with radio.h (ppm rc)
 */
#define RADIO_ROLL      0
#define RADIO_PITCH     1
#define RADIO_YAW       2
#define RADIO_THROTTLE  3
#define RADIO_MODE      4

extern int8_t rc_dl_values[ RC_DL_NB_CHANNEL ];
extern volatile bool_t rc_dl_frame_available;

/**
 * Decode datalink message to get rc values with RC_3CH message
 * Mode and throttle are merge in the same byte
 */
extern void parse_rc_3ch_datalink(
    uint8_t throttle_mode,
    int8_t roll,
    int8_t pitch);

/**
 * Decode datalink message to get rc values with RC_4CH message
 */
extern void parse_rc_4ch_datalink(
    uint8_t mode,
    uint8_t throttle,
    int8_t roll,
    int8_t pitch,
    int8_t yaw);

/**
 * Macro that normalize rc_dl_values to radio values
 */
#define NormalizeRcDl(_in, _out) {  \
  _out[RADIO_ROLL] = (MAX_PPRZ/128) * _in[RADIO_ROLL];            \
  Bound(_out[RADIO_ROLL], MIN_PPRZ, MAX_PPRZ);                    \
  _out[RADIO_PITCH] = (MAX_PPRZ/128) * _in[RADIO_PITCH];          \
  Bound(_out[RADIO_PITCH], MIN_PPRZ, MAX_PPRZ);                   \
  _out[RADIO_YAW] = 0;                                            \
  Bound(_out[RADIO_YAW], MIN_PPRZ, MAX_PPRZ);                     \
  _out[RADIO_THROTTLE] = ((MAX_PPRZ/128) * _in[RADIO_THROTTLE]);   \
  Bound(_out[RADIO_THROTTLE], 0, MAX_PPRZ);                       \
  _out[RADIO_MODE] = MAX_PPRZ * (_in[RADIO_MODE] - 1);            \
  Bound(_out[RADIO_MODE], MIN_PPRZ, MAX_PPRZ);                    \
}

/**
 * Event macro with handler callback
 */
#define RadioControlEvent(_received_frame_handler) {  \
  if (rc_dl_frame_available) {                        \
    radio_control.frame_cpt++;                        \
    radio_control.time_since_last_frame = 0;          \
    radio_control.radio_ok_cpt = 0;                   \
    radio_control.status = RC_OK;                     \
    NormalizeRcDl(rc_dl_values,radio_control.values); \
    _received_frame_handler();                        \
    rc_dl_frame_available = FALSE;                    \
  }                                                   \
}

#endif /* RC_DATALINK_H */
