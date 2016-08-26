/*
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

#ifndef RC_PPM_H
#define RC_PPM_H

#include "std.h"

#if USE_CHIBIOS_RTOS
#define EVT_RADIO_DATA 0
#define EVT_PPM_FRAME 1
extern EventSource eventPpmFrame;
extern EventSource eventRadioData;
#define chibios_broadcast_ppm_frame {                        \
        chSysLockFromIsr();                                  \
        chEvtBroadcastFlagsI(&eventPpmFrame, EVT_PPM_FRAME); \
        chSysUnlockFromIsr();                                \
        }
#else
#define chibios_broadcast_ppm_frame {}
#endif

/**
 * Architecture dependant code
 */
#include "subsystems/radio_control/ppm_arch.h"
/* must be implemented by arch dependant code */
extern void ppm_arch_init(void);

/**
 * Generated code holding the description of a given
 * transmitter
 */
#include "generated/radio.h"

/**
 * Define number of channels
 * Using generated code radio.h
 */
#define RADIO_CONTROL_NB_CHANNEL RADIO_CTL_NB

/**
 *  ppm pulse type : futaba is falling edge clocked whereas JR is rising edge
 */
#define PPM_PULSE_TYPE_POSITIVE 0
#define PPM_PULSE_TYPE_NEGATIVE 1

extern uint16_t ppm_pulses[ RADIO_CONTROL_NB_CHANNEL ];
extern volatile bool_t ppm_frame_available;

/**
 * Event macro with handler callback
 * PPM frame are normalize using the IIR filter
 */
#define RadioControlEvent(_received_frame_handler) {  \
  if (ppm_frame_available) {                          \
    radio_control.frame_cpt++;                        \
    radio_control.time_since_last_frame = 0;          \
    if (radio_control.radio_ok_cpt > 0) {             \
      radio_control.radio_ok_cpt--;                   \
    } else {                                          \
      radio_control.status = RC_OK;                   \
      NormalizePpmIIR(ppm_pulses,radio_control);      \
      _received_frame_handler();                      \
    }                                                 \
    ppm_frame_available = FALSE;                      \
  }                                                   \
}

/**
 * State machine for decoding ppm frames
 */
extern uint8_t  ppm_cur_pulse;
extern uint32_t ppm_last_pulse_time;
extern bool_t   ppm_data_valid;

/**
 * RssiValid test macro.
 * This macro has to be defined to test the validity of ppm frame
 * from an other source (ex: GPIO).
 * By default, always true.
 */
#ifndef RssiValid
#define RssiValid() TRUE
#endif

/**
 * A valid ppm frame:
 * - synchro blank
 * - correct number of channels
 * - synchro blank
 */
#define DecodePpmFrame(_ppm_time) {                         \
  uint32_t length = _ppm_time - ppm_last_pulse_time;        \
  ppm_last_pulse_time = _ppm_time;                          \
                                                            \
  if (ppm_cur_pulse == PPM_NB_CHANNEL) {                    \
    if (length > RC_PPM_TICKS_OF_USEC(PPM_SYNC_MIN_LEN) &&  \
        length < RC_PPM_TICKS_OF_USEC(PPM_SYNC_MAX_LEN)) {  \
      if (ppm_data_valid && RssiValid()) {                  \
        ppm_frame_available = TRUE;                         \
        chibios_broadcast_ppm_frame;                        \
        ppm_data_valid = FALSE;                             \
      }                                                     \
      ppm_cur_pulse = 0;                                    \
    }                                                       \
    else {                                                  \
      ppm_data_valid = FALSE;                               \
    }                                                       \
  }                                                         \
  else {                                                    \
    if (length > RC_PPM_TICKS_OF_USEC(PPM_DATA_MIN_LEN) &&  \
        length < RC_PPM_TICKS_OF_USEC(PPM_DATA_MAX_LEN)) {  \
      ppm_pulses[ppm_cur_pulse] = length;                   \
      ppm_cur_pulse++;                                      \
      if (ppm_cur_pulse == PPM_NB_CHANNEL) {                \
        ppm_data_valid = TRUE;                              \
      }                                                     \
    }                                                       \
    else {                                                  \
      ppm_cur_pulse = PPM_NB_CHANNEL;                       \
      ppm_data_valid = FALSE;                               \
    }                                                       \
  }                                                         \
}

#endif /* RC_PPM_H */
