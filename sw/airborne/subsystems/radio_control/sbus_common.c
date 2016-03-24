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

/** @file subsystems/radio_control/sbus_common.c
 *
 * Futaba SBUS decoder
 */

#include "subsystems/radio_control.h"
#include "subsystems/radio_control/sbus_common.h"
#include BOARD_CONFIG
#include "mcu_periph/gpio.h"
#include <string.h>

/*
 * SBUS protocol and state machine status
 */
#define SBUS_START_BYTE 0x0f
#define SBUS_END_BYTE 0x00
#define SBUS_BIT_PER_CHANNEL 11
#define SBUS_BIT_PER_BYTE 8
#define SBUS_FLAGS_BYTE 22
#define SBUS_FRAME_LOST_BIT 2

#define SBUS_STATUS_UNINIT      0
#define SBUS_STATUS_GOT_START   1

/** Set polarity using RC_POLARITY_GPIO.
 * SBUS signal has a reversed polarity compared to normal UART
 * this allows to using hardware UART peripheral by changing
 * the input signal polarity.
 * Setting this gpio ouput high inverts the signal,
 * output low sets it to normal polarity.
 */
#ifndef RC_SET_POLARITY
#define RC_SET_POLARITY gpio_set
#endif


void sbus_common_init(struct Sbus *sbus_p, struct uart_periph *dev)
{
  sbus_p->frame_available = false;
  sbus_p->status = SBUS_STATUS_UNINIT;

  // Set UART parameters (100K, 8 bits, 2 stops, even parity)
  uart_periph_set_bits_stop_parity(dev, UBITS_8, USTOP_2, UPARITY_EVEN);
  uart_periph_set_baudrate(dev, B100000);

  // Set polarity
#ifdef RC_POLARITY_GPIO_PORT
  gpio_setup_output(RC_POLARITY_GPIO_PORT, RC_POLARITY_GPIO_PIN);
  RC_SET_POLARITY(RC_POLARITY_GPIO_PORT, RC_POLARITY_GPIO_PIN);
#endif

}


/** Decode the raw buffer */
static void decode_sbus_buffer(const uint8_t *src, uint16_t *dst, bool *available,
                               uint16_t *dstppm)
{
  // reset counters
  uint8_t byteInRawBuf = 0;
  uint8_t bitInRawBuf = 0;
  uint8_t channel = 0;
  uint8_t bitInChannel = 0;

  // clear bits
  memset(dst, 0, SBUS_NB_CHANNEL * sizeof(uint16_t));

  // decode sbus data
  for (uint8_t i = 0; i < (SBUS_NB_CHANNEL * SBUS_BIT_PER_CHANNEL); i++) {
    if (src[byteInRawBuf] & (1 << bitInRawBuf)) {
      dst[channel] |= (1 << bitInChannel);
    }

    bitInRawBuf++;
    bitInChannel++;

    if (bitInRawBuf == SBUS_BIT_PER_BYTE) {
      bitInRawBuf = 0;
      byteInRawBuf++;
    }
    if (bitInChannel == SBUS_BIT_PER_CHANNEL) {
      bitInChannel = 0;
#if PERIODIC_TELEMETRY
      dstppm[channel] = USEC_OF_RC_PPM_TICKS(dst[channel]);
#endif
      channel++;
    }
  }
  // test frame lost flag
  *available = !bit_is_set(src[SBUS_FLAGS_BYTE], SBUS_FRAME_LOST_BIT);
}

// Decoding event function
// Reading from UART
void sbus_common_decode_event(struct Sbus *sbus_p, struct uart_periph *dev)
{
  uint8_t rbyte;
  if (uart_char_available(dev)) {
    do {
      rbyte = uart_getch(dev);
      switch (sbus_p->status) {
        case SBUS_STATUS_UNINIT:
          // Wait for the start byte
          if (rbyte == SBUS_START_BYTE) {
            sbus_p->status++;
            sbus_p->idx = 0;
          }
          break;
        case SBUS_STATUS_GOT_START:
          // Store buffer
          sbus_p->buffer[sbus_p->idx] = rbyte;
          sbus_p->idx++;
          if (sbus_p->idx == SBUS_BUF_LENGTH) {
            // Decode if last byte is the correct end byte
            if (rbyte == SBUS_END_BYTE) {
              decode_sbus_buffer(sbus_p->buffer, sbus_p->pulses, &sbus_p->frame_available, sbus_p->ppm);
            }
            sbus_p->status = SBUS_STATUS_UNINIT;
          }
          break;
        default:
          break;
      }
    } while (uart_char_available(dev));
  }
}
