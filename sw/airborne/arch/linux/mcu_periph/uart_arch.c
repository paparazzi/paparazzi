/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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
 */

/** @file arch/linux/mcu_periph/uart_arch.c
 * linux uart handling
 */

#include BOARD_CONFIG

#include "mcu_periph/uart.h"

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "serial_port.h"

// #define TRACE(fmt,args...)    fprintf(stderr, fmt, args)
#define TRACE(fmt,args...)


void uart_periph_set_baudrate(struct uart_periph *periph, uint32_t baud)
{
  periph->baudrate = baud;

  struct SerialPort *port;
  // close serial port if already open
  if (periph->reg_addr != NULL) {
    port = (struct SerialPort *)(periph->reg_addr);
    serial_port_close(port);
    serial_port_free(port);
  }
  // open serial port
  port = serial_port_new();
  // use register address to store SerialPort structure pointer...
  periph->reg_addr = (void *)port;

  //TODO: set device name in application and pass as argument
  // FIXME: paparazzi baud is 9600 for B9600 while open_raw needs 12 for B9600
  // /printf("opening %s on uart0 at termios.h baud value=%d\n", periph->dev, baud);
  int ret = serial_port_open_raw(port, periph->dev, baud);
  if (ret != 0) {
    TRACE("Error opening %s code %d\n", periph->dev, ret);
  }
}

void uart_transmit(struct uart_periph *periph, uint8_t data)
{
  uint16_t temp = (periph->tx_insert_idx + 1) % UART_TX_BUFFER_SIZE;

  if (temp == periph->tx_extract_idx) {
    printf("uart tx queue full!\n");
    return;  // no room
  }

  // check if in process of sending data
  if (periph->tx_running) { // yes, add to queue
    periph->tx_buf[periph->tx_insert_idx] = data;
    periph->tx_insert_idx = temp;
  } else { // no, set running flag and write to output register
    periph->tx_running = TRUE;
    struct SerialPort *port = (struct SerialPort *)(periph->reg_addr);
    int ret = write((int)(port->fd), &data, 1);
    if (ret < 1) {
      TRACE("uart_transmit: write %d failed [%d: %s]\n", data, ret, strerror(errno));
      /* if write was interrupted, put data into queue */
      if (errno == EINTR) {
        periph->tx_buf[periph->tx_insert_idx] = data;
        periph->tx_insert_idx = temp;
      }
    }
  }
}


static inline void uart_handler(struct uart_periph *periph)
{
  unsigned char c = 'D';

  if (periph->reg_addr == NULL) { return; } // device not initialized ?

  struct SerialPort *port = (struct SerialPort *)(periph->reg_addr);
  int fd = port->fd;

  // check if more data to send
  if (periph->tx_insert_idx != periph->tx_extract_idx) {
    int ret = write(fd, &(periph->tx_buf[periph->tx_extract_idx]), 1);
    if (ret < 1) {
      TRACE("uart_handler: write %x failed [%d: %s]\n", periph->tx_buf[periph->tx_extract_idx], ret, strerror(errno));
    }
    else {
      periph->tx_extract_idx++;
      periph->tx_extract_idx %= UART_TX_BUFFER_SIZE;
    }
  } else {
    periph->tx_running = FALSE;   // clear running flag
  }

  if (read(fd, &c, 1) > 0) {
    //printf("r %x %c\n",c,c);
    uint16_t temp = (periph->rx_insert_idx + 1) % UART_RX_BUFFER_SIZE;
    periph->rx_buf[periph->rx_insert_idx] = c;
    // check for more room in queue
    if (temp != periph->rx_extract_idx) {
      periph->rx_insert_idx = temp;  // update insert index
    }
  }

}

void uart_event(void)
{
#if USE_UART0
  uart_handler(&uart0);
#endif
#if USE_UART1
  uart_handler(&uart1);
#endif
#if USE_UART2
  uart_handler(&uart2);
#endif
#if USE_UART3
  uart_handler(&uart3);
#endif
#if USE_UART4
  uart_handler(&uart4);
#endif
#if USE_UART5
  uart_handler(&uart5);
#endif
#if USE_UART6
  uart_handler(&uart6);
#endif
}

#if USE_UART0
void uart0_init(void)
{
  uart_periph_init(&uart0);
  strncpy(uart0.dev, UART0_DEV, UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart0, UART0_BAUD);
}
#endif /* USE_UART0 */

#if USE_UART1
void uart1_init(void)
{
  uart_periph_init(&uart1);
  strncpy(uart1.dev, UART1_DEV, UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart1, UART1_BAUD);
}
#endif /* USE_UART1 */

#if USE_UART2
void uart2_init(void)
{
  uart_periph_init(&uart2);
  strncpy(uart2.dev, UART2_DEV, UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart2, UART2_BAUD);
}
#endif /* USE_UART2 */

#if USE_UART3
void uart3_init(void)
{
  uart_periph_init(&uart3);
  strncpy(uart3.dev, UART3_DEV, UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart3, UART3_BAUD);
}
#endif /* USE_UART3 */

#if USE_UART4
void uart4_init(void)
{
  uart_periph_init(&uart4);
  strncpy(uart4.dev, UART4_DEV, UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart4, UART4_BAUD);
}
#endif /* USE_UART4 */

#if USE_UART5
void uart5_init(void)
{
  uart_periph_init(&uart5);
  strncpy(uart5.dev, UART5_DEV, UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart5, UART5_BAUD);
}
#endif /* USE_UART5 */

#if USE_UART6
void uart6_init(void)
{
  uart_periph_init(&uart6);
  strncpy(uart6.dev, UART6_DEV, UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart6, UART6_BAUD);
}
#endif /* USE_UART6 */
