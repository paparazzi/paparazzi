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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef RC_SBUS_COMMON_H
#define RC_SBUS_COMMON_H

/** @file subsystems/radio_control/sbus_common.h
 *
 * Futaba SBUS decoder
 */

#include "std.h"
#include "mcu_periph/uart.h"

/**
 * Macro to use radio.h file
 *
 *  SBUS:   0..1024..2047 (sweep 2048)
 *  PPM:  880..1520..2160 (sweep 1280)
 */
#define RC_PPM_TICKS_OF_USEC(_v)        ((((_v) - 880) * 8) / 5)
#define RC_PPM_SIGNED_TICKS_OF_USEC(_v) (((_v) * 8) / 5)
#define USEC_OF_RC_PPM_TICKS(_v)        ((((_v) * 5) / 8) + 880)

/**
 * Generated code holding the description of a given
 * transmitter
 */
#include "generated/radio.h"

/**
 * Define number of channels.
 *
 * SBUS frame always have 16 channels
 * but only the X first one will be available
 * depending of the RC transmitter.
 * The radio XML file is used to assign the
 * input values to RC channels.
 */
#define SBUS_BUF_LENGTH 24
#define SBUS_NB_CHANNEL 16
#define RADIO_CONTROL_NB_CHANNEL SBUS_NB_CHANNEL

/**
 * SBUS structure
 */
struct _sbus {
  uint16_t pulses[SBUS_NB_CHANNEL]; ///< decoded values
  uint16_t ppm[SBUS_NB_CHANNEL];    ///< decoded and converted values
  bool_t frame_available;           ///< new frame available
  uint8_t buffer[SBUS_BUF_LENGTH];  ///< input buffer
  uint8_t idx;                      ///< input index
  uint8_t status;                   ///< decoder state machine status
};

/**
 * Init function
 */
void sbus_common_init(struct _sbus* sbus, struct uart_periph* dev);

/**
 * Decoding event function
 */
void sbus_common_decode_event(struct _sbus* sbus, struct uart_periph* dev);


#endif /* RC_SBUS_H */
