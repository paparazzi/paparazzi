/*
 * Copyright (C) 2013 Alexandre Bustico, Gautier Hattenberger
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
 * @file modules/radio_control/radio_control.hott_common.h
 *
 * Common hott structs and defines.
 */

#ifndef RC_HOTT_COMMON_H
#define RC_HOTT_COMMON_H

#include "std.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/gpio.h"

/* in case you want to override RADIO_CONTROL_NB_CHANNEL */
#include "generated/airframe.h"

/**
 * Macro to use radio.h file
 *
 *  HOTT:   7040..12000..16800 
 *  PPM:  880..1500..2100
 */
#define RC_PPM_TICKS_OF_USEC(_v)        ((_v) * 8 ) // USEC IN HOTT OUT
#define RC_PPM_SIGNED_TICKS_OF_USEC(_v) ((_v) * 8 ) // usec -1000 + 1000 in, HOTT out
#define USEC_OF_RC_PPM_TICKS(_v)        ((_v) / 8) // HOTT IN USEC OUT

/**
 * Generated code holding the description of a given
 * transmitter
 */
#include "generated/radio.h"

/**
 * Define number of channels.
 *
 * HOTT SUMD frame has between 2 and 32 channels
 * The radio XML file is used to assign the
 * input values to RC channels.
 */
#define HOTT_NB_CHANNEL 32
#define HOTT_BUF_LENGTH (HOTT_NB_CHANNEL*2+3+2) // 2 bytes per chennel 3 bytes header, 2 bytes CRC 

/**
 * Default number of channels to actually use.
 */
#ifndef RADIO_CONTROL_NB_CHANNEL
#define RADIO_CONTROL_NB_CHANNEL HOTT_NB_CHANNEL
#endif

#if RADIO_CONTROL_NB_CHANNEL > HOTT_NB_CHANNEL
#error "RADIO_CONTROL_NB_CHANNEL mustn't be higher than 32."
#endif

/**
 * HOTT structure
 */
struct SHott {
  uint16_t pulses[HOTT_NB_CHANNEL]; ///< decoded values
  uint16_t ppm[HOTT_NB_CHANNEL];    ///< decoded and converted values
  bool frame_available;           ///< new frame available
  uint8_t buffer[HOTT_BUF_LENGTH];  ///< input buffer
  uint8_t expected_channels;      ///< expected number of channels send in header
  uint8_t idx;                      ///< input index
  uint8_t status;                   ///< decoder state machine status
};

/**
 * Init function
 */
void hott_common_init(struct SHott *hott, struct uart_periph *dev);

/**
 * Decoding event function
 */
void hott_common_decode_event(struct SHott *hott, struct uart_periph *dev);


/**
 * RC event function with handler callback.
 */
extern void radio_control_impl_event(void (* _received_frame_handler)(void));

/**
 * Event macro with handler callback
 */
#define RadioControlEvent(_received_frame_handler) radio_control_impl_event(_received_frame_handler)


#endif /* RC_HOTT_H */
