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

#if USE_CHIBIOS_RTOS
mutex_t mtx_sbus;
static THD_WORKING_AREA(wa_thd_sbus_rx, CH_THREAD_AREA_SBUS_RX);

/*
 * Serial config for SBUS
 * 100k Baud rate, 2 stop bits, Even parity (default), 8 data bits (default)
 */
static const SerialConfig sbus_config =
{
  B100000,              /*     BITRATE    */
  0,                    /*    USART CR1   */
  USART_CR2_STOP2_BITS, /*    USART CR2   */
  0                     /*    USART CR3   */
};
#endif /* USE_CHIBIOS_RTOS */

void sbus_common_init(struct Sbus *sbus_p, struct uart_periph *dev)
{
  sbus_p->frame_available = FALSE;
  sbus_p->status = SBUS_STATUS_UNINIT;

#if USE_CHIBIOS_RTOS
  // init mutex
  chMtxObjectInit(&mtx_sbus);

  // set serial port to right values
  sdStop(dev->reg_addr);
  sdStart(dev->reg_addr, &sbus_config);

  // init thread
  chThdCreateStatic(wa_thd_sbus_rx, sizeof(wa_thd_sbus_rx), NORMALPRIO, thd_sbus_rx, dev);
#else /* Classic PPRZ */
  // Set UART parameters (100K, 8 bits, 2 stops, even parity)
  uart_periph_set_bits_stop_parity(dev, UBITS_8, USTOP_2, UPARITY_EVEN);
  uart_periph_set_baudrate(dev, B100000);

  // Set polarity
#ifdef RC_POLARITY_GPIO_PORT
  gpio_setup_output(RC_POLARITY_GPIO_PORT, RC_POLARITY_GPIO_PIN);
  RC_SET_POLARITY(RC_POLARITY_GPIO_PORT, RC_POLARITY_GPIO_PIN);
#endif  /* RC_POLARITY_GPIO_PORT */
#endif  /* USE_CHIBIOS_RTOS */

}


/** Decode the raw buffer */
static void decode_sbus_buffer(const uint8_t *src, uint16_t *dst, bool_t *available,
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
#if !USE_CHIBIOS_RTOS
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
#else /* USE_CHIBIOS_RTOS */
void sbus_common_decode_event(struct Sbus *sbus_p __attribute__((unused)),
    struct uart_periph *dev __attribute__((unused))) {}

/**
 * Parse incoming sbus data
 * @param c
 */
void sbus_parse(uint8_t rbyte) {
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
      if (rbyte == SBUS_END_BYTE) {
        decode_sbus_buffer(sbus.buffer, sbus.pulses, &sbus.frame_available, sbus.ppm);
      }
      sbus.status = SBUS_STATUS_UNINIT;
    }
    break;
    default:
    break;
  }
}

/**
 * Replacement for sbus_decode_event()
 * @param decode_sbus_buffer - callback to be called once buffer is full
 * @return
 */
void thd_sbus_rx(void* arg) {
  chRegSetThreadName("sbus_rx");
  struct uart_periph *dev = (struct uart_periph*)arg;

  // BACKUP version: naive implementation
  while (TRUE) {
    sbus_parse(uart_getch(dev));
    if (sbus.frame_available) {
      chEvtBroadcastFlags(&eventRadioFrame, EVT_RADIO_FRAME);
    }
  }
/*
  // Proper event based implementation
  event_listener_t elSBUSdata;
  eventflags_t flags;
  chEvtRegisterMask((event_source_t *)chnGetEventSource((SerialDriver*)dev->reg_addr), &elSBUSdata, EVENT_MASK(1));
  while (TRUE) {
    chEvtWaitAny(ALL_EVENTS);
    flags = chEvtGetAndClearFlags(&elSBUSdata);
    uart_receive_buffer(dev, flags, &sbus_parse);
    if (sbus.frame_available) {
      chEvtBroadcastFlags(&eventRadioFrame, EVT_RADIO_FRAME);
    }
  }
*/
}
#endif /* !USE_CHIBIOS_RTOS */
