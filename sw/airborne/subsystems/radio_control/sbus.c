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

/** @file subsystems/radio_control/sbus.c
 *
 * Futaba SBUS decoder
 */

#include "subsystems/radio_control.h"
#include "subsystems/radio_control/sbus.h"
#include BOARD_CONFIG
#include "mcu_periph/uart.h"
#include <string.h>

/*
 * SBUS protocol and state machine status
 */
#define SBUS_START_BYTE 0x0f
#define SBUS_END_BYTE 0x00
#define SBUS_BIT_PER_CHANNEL 11
#define SBUS_BIT_PER_BYTE 8

#define SBUS_STATUS_UNINIT      0
#define SBUS_STATUS_GOT_START   1

/** Set polarity using RC_POLARITY_LED.
 *  SBUS signal has a reversed polarity compared to normal UART
 *  this allow to using hardware UART peripheral by changing
 * the input signal polarity
 */
#ifndef RC_SET_POLARITY
#define RC_SET_POLARITY LED_ON
#endif

/** UART selecting
 */
#define __SBusLink(dev, _x) dev##_x
#define _SBusLink(dev, _x)  __SBusLink(dev, _x)
#define SBusLink(_x) _SBusLink(SBUS_LINK, _x)

/* SBUS struct */
struct _sbus sbus;

// Init function
void radio_control_impl_init(void) {
  sbus.frame_available = FALSE;
  sbus.status = SBUS_STATUS_UNINIT;

  // Set UART parameters (100K, 8 bits, 2 stops, even parity)
  SBusLink(SetBitsStopParity(UBITS_8, USTOP_2, UPARITY_EVEN));
  SBusLink(SetBaudrate(B100000));

  // Set polarity
#ifdef RC_POLARITY_LED
  RC_SET_POLARITY(RC_POLARITY_LED);
#endif
}

/*
 * This part is used by the autopilot to read data from a uart
 */
#define SBusBuffer() SBusLink(ChAvailable())
#define SBusGet() SBusLink(Getch())

// Decode the raw buffer
static void decode_sbus_buffer (const uint8_t *src, uint16_t *dst)
{
  // reset counters
  uint8_t byteInRawBuf = 0;
  uint8_t bitInRawBuf = 0;
  uint8_t channel = 0;
  uint8_t bitInChannel = 0;

  // clear bits
  memset (dst, 0, SBUS_NB_CHANNEL*sizeof(uint16_t));

  // decode sbus data
  for (uint8_t i=0; i< (SBUS_NB_CHANNEL*SBUS_BIT_PER_CHANNEL); i++) {
    if (src[byteInRawBuf] & (1<<bitInRawBuf))
      dst[channel] |= (1<<bitInChannel);

    bitInRawBuf++;
    bitInChannel++;

    if (bitInRawBuf == SBUS_BIT_PER_BYTE) {
      bitInRawBuf = 0;
      byteInRawBuf++;
    }
    if (bitInChannel == SBUS_BIT_PER_CHANNEL) {
      bitInChannel = 0;
      channel++;
    }
  }
}

// Decoding event function
// Reading from UART
void sbus_decode_event(void) {
  uint8_t rbyte;
  if (SBusBuffer()) {
    do {
      rbyte = SBusGet();
      switch (sbus.status) {
        case SBUS_STATUS_UNINIT:
          // Wait for the start byte
          if (rbyte == SBUS_START_BYTE) {
            sbus.status++;
            sbus.idx = 0;
          }
          break;
        case SBUS_STATUS_GOT_START:
          // Store buffer
          sbus.buffer[sbus.idx] = rbyte;
          sbus.idx++;
          if (sbus.idx == SBUS_BUF_LENGTH) {
            // Decode if last byte is the correct end byte
            if (rbyte == SBUS_END_BYTE) decode_sbus_buffer(sbus.buffer, sbus.pulses);
            sbus.status = SBUS_STATUS_UNINIT;
          }
          break;
        default:
          break;
      }
    } while (SBusBuffer());
  }
}

