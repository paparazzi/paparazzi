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

/** @file modules/radio_control/sbus_common.c
 *
 * Futaba SBUS decoder
 */

#include "modules/radio_control/radio_control.h"
#include "modules/radio_control/sbus_common.h"
#include "mcu_periph/sys_time.h"
#include BOARD_CONFIG
#include <string.h>
#include <stdbool.h>

/*
 * SBUS protocol and state machine status
 */
#define SBUS_START_BYTE 0x0f
#define SBUS_END_BYTE_0 0x00 // only possible end byte for SBUS v1
#define SBUS_END_BYTE_1 0x04 // with SBUS v2 and 14CH mode
#define SBUS_END_BYTE_2 0x14 // end byte is cycling
#define SBUS_END_BYTE_3 0x24 // through the following 4 types
#define SBUS_END_BYTE_4 0x34 // in order
#define SBUS_END_BYTE_5 0x08 // with SBUS v2 and 12CH mode
#define SBUS_BIT_PER_CHANNEL 11
#define SBUS_BIT_PER_BYTE 8
#define SBUS_FLAGS_BYTE 22
#define SBUS_FRAME_LOST_BIT 2
#define SBUS_RC_FAILSAFE_BIT 3
#define SBUS_RC_LOST_BIT 5

#define SBUS_STATUS_UNINIT 0
#define SBUS_STATUS_GOT_START 1

/**
 * Time before timeout when receiving
 * a full SBUS frame
 */
#define SBUS_TIMEOUT_MS 4

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

void sbus_common_init(struct Sbus *sbus_p, struct uart_periph *dev,
                      gpio_port_t gpio_polarity_port, uint16_t gpio_polarity_pin)
{
  //Assume the worst ;)
  sbus_p->frame_available = false;
  sbus_p->rc_failsafe = true;
  sbus_p->rc_lost = true;
  sbus_p->status = SBUS_STATUS_UNINIT;
  sbus_p->start_time = 0;

  // Set UART parameter, SBUS used a baud rate of 100000, 8 data bits, even parity bit, and 2 stop bits
  uart_periph_set_baudrate(dev, B100000);
  uart_periph_set_bits_stop_parity(dev, UBITS_8, USTOP_2, UPARITY_EVEN);
  // Try to invert RX data logic when available in hardware periph
  uart_periph_invert_data_logic(dev, true, false);

  // Set polarity (when not done in hardware, don't use both!)
  if (gpio_polarity_port != 0) {
    gpio_setup_output(gpio_polarity_port, gpio_polarity_pin);
    RC_SET_POLARITY(gpio_polarity_port, gpio_polarity_pin);
  }

}

/** Decode the raw buffer */
static void decode_sbus_buffer(const uint8_t *src, uint16_t *dst,
		                       bool *is_frame_available, bool *is_rc_failsafe, bool *is_rc_lost,
                               uint16_t *dstppm __attribute__((unused)))
{
  // decode sbus data, unrolling the loop for efficiency
  dst[0]  = ((src[0]) | (src[1] << 8))                  & 0x07FF;
  dst[1]  = ((src[1] >> 3) | (src[2] << 5))                  & 0x07FF;
  dst[2]  = ((src[2] >> 6) | (src[3] << 2)  | (src[4] << 10))  & 0x07FF;
  dst[3]  = ((src[4] >> 1) | (src[5] << 7))                  & 0x07FF;
  dst[4]  = ((src[5] >> 4) | (src[6] << 4))                  & 0x07FF;
  dst[5]  = ((src[6] >> 7) | (src[7] << 1) | (src[8] << 9))   & 0x07FF;
  dst[6]  = ((src[8] >> 2) | (src[9] << 6))                  & 0x07FF;
  dst[7]  = ((src[9] >> 5)  | (src[10] << 3))                 & 0x07FF;
  dst[8]  = ((src[11]) | (src[12] << 8))                 & 0x07FF;
  dst[9]  = ((src[12] >> 3) | (src[13] << 5))                 & 0x07FF;
  dst[10] = ((src[13] >> 6) | (src[14] << 2) | (src[15] << 10)) & 0x07FF;
  dst[11] = ((src[15] >> 1) | (src[16] << 7))                 & 0x07FF;
  dst[12] = ((src[16] >> 4) | (src[17] << 4))                 & 0x07FF;
  dst[13] = ((src[17] >> 7) | (src[18] << 1) | (src[19] << 9))  & 0x07FF;
  dst[14] = ((src[19] >> 2) | (src[20] << 6))                 & 0x07FF;
  dst[15] = ((src[20] >> 5) | (src[21] << 3))                 & 0x07FF;

  // Convert sbus to ppm
#if PERIODIC_TELEMETRY
  for (int channel = 0; channel < SBUS_NB_CHANNEL; channel++) {
    dstppm[channel] = USEC_OF_RC_PPM_TICKS(dst[channel]);
  }
#endif

  // Test frame loss flag
  *is_frame_available = !bit_is_set(src[SBUS_FLAGS_BYTE], SBUS_FRAME_LOST_BIT);
  // Check if receiver is in Failsafe mode
  *is_rc_failsafe = bit_is_set(src[SBUS_FLAGS_BYTE], SBUS_RC_FAILSAFE_BIT);
  // Also check if the RC link is lost
  *is_rc_lost = bit_is_set(src[SBUS_FLAGS_BYTE], SBUS_RC_LOST_BIT);

}

// Decoding event function
// Reading from UART
void sbus_common_decode_event(struct Sbus *sbus_p, struct uart_periph *dev)
{
  uint8_t rbyte;
  if (uart_char_available(dev)) {
    // Took too long to receive a full SBUS frame (usually during boot to synchronize)
    if(get_sys_time_msec() > (sbus_p->start_time+SBUS_TIMEOUT_MS)) {
      sbus_p->status = SBUS_STATUS_UNINIT;
    }

    do {
      rbyte = uart_getch(dev);
      switch (sbus_p->status) {
        case SBUS_STATUS_UNINIT:
          // Wait for the start byte
          if (rbyte == SBUS_START_BYTE) {
            sbus_p->status++;
            sbus_p->idx = 0;
            sbus_p->start_time = get_sys_time_msec();
          }
          break;
        case SBUS_STATUS_GOT_START:
          // Store buffer
          sbus_p->buffer[sbus_p->idx] = rbyte;
          sbus_p->idx++;
          if (sbus_p->idx == SBUS_BUF_LENGTH) {
            // Decode if last byte is (one of) the correct end byte
            if (rbyte == SBUS_END_BYTE_0 ||
                rbyte == SBUS_END_BYTE_1 ||
                rbyte == SBUS_END_BYTE_2 ||
                rbyte == SBUS_END_BYTE_3 ||
                rbyte == SBUS_END_BYTE_4 ||
                rbyte == SBUS_END_BYTE_5) {
              decode_sbus_buffer(sbus_p->buffer, sbus_p->pulses, &sbus_p->frame_available, &sbus_p->rc_failsafe, &sbus_p->rc_lost, sbus_p->ppm);
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
