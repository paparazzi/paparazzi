/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "lisa/lisa_overo_link.h"
#include "lisa/lisa_spistream.h"
#include "generated/airframe.h"
#include "mcu_periph/uart.h"
#include "led.h"

static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);
static inline void uart_transfer_event(void);

static inline void on_spistream_msg_received(uint8_t msg_id, uint8_t * data, uint16_t num_bytes);
static inline void on_spistream_msg_sent(uint8_t msg_id);

static inline void on_overo_link_msg_received(void);
static inline void on_overo_link_lost(void);
static inline void on_overo_link_crc_failed(void);

#ifdef SPISTREAM_DEBUG
static inline void uart_debug_transfer_event(void);
static inline void uart_debug_transfer_init(void);
#endif

struct __attribute__ ((packed)) spistream_uart_msg {
  uint8_t uart_id;
  uint8_t uart_data[SPISTREAM_MAX_MESSAGE_LENGTH];
};

// TODO
// Use 3 static instances of this struct in uart_transfer_event
// instead of a myriad of repetitive static vars:
struct uart_state {
  struct spistream_uart_msg * msg;
  uint32_t timeout;
  uint32_t num_rx_bytes;
  uint8_t enabled;
  uint8_t has_data;
  uint8_t sent;
};

static struct spistream_uart_msg spistream_uart1_msg;
static struct spistream_uart_msg spistream_uart2_msg;
static struct spistream_uart_msg spistream_uart3_msg;
#ifdef SPISTREAM_DEBUG
static struct spistream_uart_msg spistream_debug_msg;
#endif

// The number of current, unfinished spistream transfers.
// Incremented after sending an spistream message, decremented
// in spistream TX completion handler (here: on_spistream_msg_sent).
static uint8_t spistream_wait_for_num_transfers = 0;

int main(void) {

  main_init();

  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic();
    }
    main_event();
  }

  return 0;
}

static inline void main_init(void) {

  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  overo_link_init();

  uart1_init();
  uart2_init();
  uart3_init();

  spistream_uart1_msg.uart_id = 1;
  spistream_uart2_msg.uart_id = 2;
  spistream_uart3_msg.uart_id = 3;

#ifdef SPISTREAM_DEBUG
  uart_debug_transfer_init();
#endif

  spistream_init(&on_spistream_msg_received,
                 &on_spistream_msg_sent);

}

/**
 * Handler for commands (messages from Overo->STM).
 * Right now, it is just sending the command back for
 * debugging purposes.
 */
static inline void on_spistream_msg_received(uint8_t msg_id,
                                             uint8_t * data,
                                             uint16_t num_bytes)
{
  spistream_send_msg(data, num_bytes, SPISTREAM_NO_WAIT);
}

static inline void on_spistream_msg_sent(uint8_t msg_id) {
  if(spistream_wait_for_num_transfers > 0) {
  	spistream_wait_for_num_transfers--;
  }
}

static inline void main_periodic(void)
{
  OveroLinkPeriodic(on_overo_link_lost);

  RunOnceEvery(1, {
      LED_PERIODIC();
    });
}

/**
 * Every SPI transfer contains exactly two packages of type
 * AutopilotMessagePTStream, one for each direction
 * (up: STM->Overo, down: Overo->STM).
 * As we delegate SPI message handling to spistream, the SPI
 * event just passes both packages to
 * spistream_read_pkg(down_pkg) and spistream_write_pkg(up_pkg).
 * Apart from that, we just don't care about the SPI driver
 * itself.
 */
static inline void on_overo_link_msg_received(void)
{
  spistream_read_pkg(&overo_link.down.msg);
  spistream_write_pkg(&overo_link.up.msg);
}

static inline void on_overo_link_lost(void) {
}

static inline void on_overo_link_crc_failed(void) {
}

static inline void main_event(void)
{
  OveroLinkEvent(on_overo_link_msg_received, on_overo_link_crc_failed);

#ifdef SPISTREAM_DEBUG
  uart_debug_transfer_event();
#else
  uart_transfer_event();
#endif
}

#ifdef SPISTREAM_DEBUG
static inline void uart_debug_transfer_init(void) {
  uint16_t idx;
  for(idx = 1; idx < 700; idx++) {
    spistream_debug_msg.uart_data[idx] = idx % 40;
  }
}
#endif

/**
 * spistream stress test: Send big (500-700 bytes) messages
 * with different message lengths for every channel and
 * length varying in every message.
 * Fool around with timeout to increase/decrease message
 * rate to see when it can't keep up any more.
 */
#ifdef SPISTREAM_DEBUG
static inline void uart_debug_transfer_event(void) {
  static uint16_t len = 0;
  static uint16_t timeout = 1;

  if(timeout-- == 0) {
    timeout = 8000;
    if(spistream_wait_for_num_transfers == 0)
    {
      LED_OFF(6);
      len++;
      if(len > 700) { len = 500; }

      spistream_debug_msg.uart_id = 1;
      if(spistream_send_msg((uint8_t *)&spistream_debug_msg,
                            len+1-20,
                            SPISTREAM_WAIT_FOR_READ)) { // +1 for UART id byte
        spistream_wait_for_num_transfers++;
      }
      spistream_debug_msg.uart_id = 2;
      if(spistream_send_msg((uint8_t *)&spistream_debug_msg,
                            len+1,
                            SPISTREAM_WAIT_FOR_READ)) { // +1 for UART id byte
        spistream_wait_for_num_transfers++;
      }
      spistream_debug_msg.uart_id = 3;
      if(spistream_send_msg((uint8_t *)&spistream_debug_msg,
                            len+1+20,
                            SPISTREAM_WAIT_FOR_READ)) { // +1 for UART id byte
        spistream_wait_for_num_transfers++;
      }
    }
    else {
      LED_ON(6);
    }
  }
}
#endif

static inline void uart_transfer_event(void) {

  static uint16_t uart1_num_rx_bytes = 0;
  static uint16_t uart2_num_rx_bytes = 0;
  static uint16_t uart3_num_rx_bytes = 0;
  static uint32_t timeout_trig  = 2;
  static uint32_t timeout_uart1 = 0;
  static uint32_t timeout_uart2 = 0;
  static uint32_t timeout_uart3 = 0;
  static uint8_t uart1_sent = 0;
  static uint8_t uart2_sent = 0;
  static uint8_t uart3_sent = 0;
  static uint8_t uart1_has_data = 0;
  static uint8_t uart2_has_data = 0;
  static uint8_t uart3_has_data = 0;
  static uint8_t trigger_send = 0;

  static uint8_t uart1_enabled = 1;
  static uint8_t uart2_enabled = 1;
  static uint8_t uart3_enabled = 1;

  // We cache data availability, so it doesn't change between checks:
  uart1_has_data = uart_char_available(&uart1);
  uart2_has_data = uart_char_available(&uart2);
  uart3_has_data = uart_char_available(&uart3);

  // Fill stage: Read data from UARTs into buffers, or increment
  // their timeouts if no data is available:
  if(!uart1_sent && uart1_has_data) {
    spistream_uart1_msg.uart_data[uart1_num_rx_bytes] = uart_getch(&uart1);
    timeout_uart1 = 0;
    if(uart1_num_rx_bytes < SPISTREAM_MAX_MESSAGE_LENGTH)
    { uart1_num_rx_bytes++; }
  } else { if(timeout_uart1 < timeout_trig) { timeout_uart1++; } }

  if(!uart2_sent && uart2_has_data) {
    spistream_uart2_msg.uart_data[uart2_num_rx_bytes] = uart_getch(&uart2);
    timeout_uart2 = 0;
    if(uart2_num_rx_bytes < SPISTREAM_MAX_MESSAGE_LENGTH)
    { uart2_num_rx_bytes++; }
  } else { if(timeout_uart2 < timeout_trig) { timeout_uart2++; } }

  if(!uart3_sent && uart3_has_data) {
    spistream_uart3_msg.uart_data[uart3_num_rx_bytes] =  uart_getch(&uart3);
    timeout_uart3 = 0;
    if(uart3_num_rx_bytes < SPISTREAM_MAX_MESSAGE_LENGTH)
    { uart3_num_rx_bytes++; }
  } else { if(timeout_uart3 < timeout_trig) { timeout_uart3++; } }

  trigger_send = ((!uart1_enabled ||
                   (timeout_uart1 >= timeout_trig)) &&
                  (!uart2_enabled ||
                   (timeout_uart2 >= timeout_trig)) &&
                  (!uart3_enabled ||
                   (timeout_uart3 >= timeout_trig)));

  // Send stage: If all UART timeouts reach the timeout
  // trigger value and have accumulated data to send
  if(trigger_send) {

    // If there was no new data on any UART for some time
    // and there is data in every rx buffer:

    if(spistream_wait_for_num_transfers > 0)
    {
      // Warning LED: Could not finish all transactions
      // from last call.
      LED_ON(6);
    }
    else
    {
      LED_OFF(6);

      uart1_sent = !uart1_enabled; // If we set uartX_sent to 1 here, it
      uart2_sent = !uart2_enabled; // is just ignored for every read poll
      uart3_sent = !uart3_enabled; // as it seems to have been read already.

      if(!uart1_sent && uart1_num_rx_bytes > 0) {
        if(spistream_send_msg((uint8_t *)&spistream_uart1_msg,
                              uart1_num_rx_bytes+1, // +1 for UART id
                              SPISTREAM_WAIT_FOR_READ)) {
          uart1_sent = 1;
          spistream_wait_for_num_transfers++;
        }
      }

      if(!uart2_sent && uart1_num_rx_bytes > 0) {
        if(spistream_send_msg((uint8_t *)&spistream_uart2_msg,
                              uart1_num_rx_bytes+1, // +1 for UART id
                              SPISTREAM_WAIT_FOR_READ)) {
          uart2_sent = 1;
          spistream_wait_for_num_transfers++;
        }
      }

      if(!uart3_sent && uart3_num_rx_bytes > 0) {
        if(spistream_send_msg((uint8_t *)&spistream_uart3_msg,
                              uart3_num_rx_bytes+1, // +1 for UART id
                              SPISTREAM_WAIT_FOR_READ)) {
          uart3_sent = 1;
          spistream_wait_for_num_transfers++;
        }
      }

      // Transaction completed, reset state.
      // Note: Only reset when all uart buffers have been transmitted,
      // otherwise the timeout would start from the beginning and the
      // loop phase shifts (aka "you're in the deep").
      uart1_num_rx_bytes = 0;
      uart2_num_rx_bytes = 0;
      uart3_num_rx_bytes = 0;
      timeout_uart1 = 0;
      timeout_uart2 = 0;
      timeout_uart3 = 0;
      uart1_sent = 0;
      uart2_sent = 0;
      uart3_sent = 0;
    }
  }
}
