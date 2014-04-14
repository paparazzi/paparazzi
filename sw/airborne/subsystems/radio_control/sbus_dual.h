/*
 * Copyright (C) 2014 Christophe De Wagter
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

#ifndef RC_SBUS_DUAL_H
#define RC_SBUS_DUAL_H

/** @file subsystems/radio_control/sbus_dual.h
 *
 * Dual SBUS radio_control
 */

#include "subsystems/radio_control/sbus_common.h"


extern struct _sbus sbus1, sbus2;

/**
 * Decoding event function
 */
extern void sbus_dual_decode_event(void);

/**
 * Event macro with handler callback
 */
#define RadioControlEvent(_received_frame_handler) {  \
  sbus_dual_decode_event();                           \
  if (sbus2.frame_available) {                        \
    radio_control.frame_cpt++;                        \
    radio_control.time_since_last_frame = 0;          \
    if (radio_control.radio_ok_cpt > 0) {             \
      radio_control.radio_ok_cpt--;                   \
    } else {                                          \
      radio_control.status = RC_OK;                   \
      NormalizePpmIIR(sbus2.pulses,radio_control);    \
      _received_frame_handler();                      \
    }                                                 \
    sbus2.frame_available = FALSE;                    \
  }                                                   \
  if (sbus1.frame_available) {                        \
    radio_control.frame_cpt++;                        \
    radio_control.time_since_last_frame = 0;          \
    if (radio_control.radio_ok_cpt > 0) {             \
      radio_control.radio_ok_cpt--;                   \
    } else {                                          \
      radio_control.status = RC_OK;                   \
      NormalizePpmIIR(sbus1.pulses,radio_control);    \
      _received_frame_handler();                      \
    }                                                 \
    sbus1.frame_available = FALSE;                    \
  }                                                   \
}

#endif /* RC_SBUS_H */
