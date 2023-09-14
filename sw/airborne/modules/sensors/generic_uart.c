/*
 * Copyright (C) 2020 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi
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
 * @file "modules/sensors/generic_uart.c"
 * @author F. van Tienen
 * Generic UART sensor, forwarding towards the GCS through telemetry
 */

#include "modules/sensors/generic_uart.h"
#include "modules/datalink/telemetry.h"
#include "mcu_periph/uart.h"

/* Default end character */
#ifndef GENERIC_UART_ENDCHAR
#define GENERIC_UART_ENDCHAR '>'
#endif

/* Default max sending message */
#ifndef GENERIC_UART_MAX_SENDLEN
#define GENERIC_UART_MAX_SENDLEN 64
#endif

/* Default max buffer size */
#ifndef GENERIC_UART_MAX_BUFSIZE
#define GENERIC_UART_MAX_BUFSIZE 128
#endif

/* Main variables */
static struct link_device *gen_uart_dev = (&((GENERIC_UART_PORT).device));   ///< UART device for communication

#if USE_SHELL
#include "modules/core/shell.h"
#include "mcu_periph/sys_time.h"
#include "printf.h"
#include "string.h"

static void generic_uart_send(shell_stream_t *sh, int argc, const char *const argv[])
{
  (void) argv;
  if (argc != 1) {
    chprintf(sh, "Usage: generic_uart \"Messsage\"\r\n");
    return;
  }

  chprintf(sh, "Sending: \"%s\"\r\n", argv[0]);
  gen_uart_dev->put_buffer(gen_uart_dev->periph, 0, argv[0], len(argv[0]));
}

#endif

void generic_uart_init(void) {
#if USE_SHELL
  shell_add_entry("generic_uart", generic_uart_send);
#endif
}


/* Event function to read UART message and forward to downlink */
void generic_uart_event(void) {
  // Receive buffer
  static uint8_t gen_msg_buf[GENERIC_UART_MAX_BUFSIZE];
  static uint8_t gen_msg_cnt = 0;

  // Look for data on serial port and save it in the buffer
  while (gen_msg_cnt < GENERIC_UART_MAX_BUFSIZE && gen_uart_dev->char_available(gen_uart_dev->periph)) { 
    gen_msg_buf[gen_msg_cnt++] = gen_uart_dev->get_byte(gen_uart_dev->periph);
    
    if(gen_msg_buf[gen_msg_cnt-1] == GENERIC_UART_ENDCHAR)
      break;
  }

  // Forward the message to the GCS
  if(gen_msg_buf[gen_msg_cnt-1] == GENERIC_UART_ENDCHAR || gen_msg_cnt > GENERIC_UART_MAX_SENDLEN) {   
    DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, gen_msg_cnt, gen_msg_buf);
    
    gen_msg_buf[0] = 0;
    gen_msg_cnt = 0;
  }
}
