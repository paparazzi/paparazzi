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

/** @file arch/omap/mcu_periph/uart_arch.c
 * omap uart handling
 */

#include "mcu_periph/uart.h"

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "serial_port.h"

// #define TRACE(fmt,args...)    fprintf(stderr, fmt, args)
#define TRACE(fmt,args...)


void uart_periph_set_baudrate(struct uart_periph* periph, uint32_t baud) {
  struct SerialPort* port;
  // close serial port if already open
  if (periph->reg_addr != NULL) {
    port = (struct SerialPort*)(periph->reg_addr);
    serial_port_close(port);
    serial_port_free(port);
  }
  // open serial port
  port = serial_port_new();
  // use register address to store SerialPort structure pointer...
  periph->reg_addr = (void*)port;

  //TODO: set device name in application and pass as argument
  // FIXME: paparazzi baud is 9600 for B9600 while open_raw needs 12 for B9600
  printf("opening %s on uart0 at termios.h baud value=%d\n", periph->dev, baud);
  int ret = serial_port_open_raw(port,periph->dev, baud);
  if (ret != 0)
  {
    TRACE("Error opening %s code %d\n",periph->dev,ret);
  }
}

void uart_transmit(struct uart_periph* periph, uint8_t data) {
  uint16_t temp = (periph->tx_insert_idx + 1) % UART_TX_BUFFER_SIZE;

  if (temp == periph->tx_extract_idx)
    return;                          // no room

  // check if in process of sending data
  if (periph->tx_running) { // yes, add to queue
    periph->tx_buf[periph->tx_insert_idx] = data;
    periph->tx_insert_idx = temp;
  }
  else { // no, set running flag and write to output register
    periph->tx_running = TRUE;
    struct SerialPort* port = (struct SerialPort*)(periph->reg_addr);
    int ret = write((int)(port->fd), &data, 1);
    if (ret < 1)
    {
      TRACE("w %x [%d]\n",data,ret);
    }
  }
}

#include <errno.h>

static inline void uart_handler(struct uart_periph* periph) {
  unsigned char c='D';

  if (periph->reg_addr == NULL) return; // device not initialized ?

  struct SerialPort* port = (struct SerialPort*)(periph->reg_addr);
  int fd = port->fd;

  // check if more data to send
  if (periph->tx_insert_idx != periph->tx_extract_idx) {
    int ret = write(fd, &(periph->tx_buf[periph->tx_extract_idx]), 1);
    if (ret < 1)
    {
      TRACE("w %x [%d: %s]\n", periph->tx_buf[periph->tx_extract_idx], ret, strerror(errno));
    }
    periph->tx_extract_idx++;
    periph->tx_extract_idx %= UART_TX_BUFFER_SIZE;
  }
  else {
    periph->tx_running = FALSE;   // clear running flag
  }

  if(read(fd,&c,1) > 0){
    //printf("r %x %c\n",c,c);
    uint16_t temp = (periph->rx_insert_idx + 1) % UART_RX_BUFFER_SIZE;
    periph->rx_buf[periph->rx_insert_idx] = c;
    // check for more room in queue
    if (temp != periph->rx_extract_idx)
      periph->rx_insert_idx = temp; // update insert index
  }

}

#ifdef USE_UART0

void uart0_init( void ) {
  uart_periph_init(&uart0);
  strcpy(uart0.dev, UART0_DEV);
  uart_periph_set_baudrate(&uart0, UART0_BAUD);
}


void uart0_handler(void) {
  uart_handler(&uart0);
}

#endif /* USE_UART0 */

#ifdef USE_UART1

void uart1_init( void ) {
  uart_periph_init(&uart1);
  strcpy(uart1.dev, UART1_DEV);
  uart_periph_set_baudrate(&uart1, UART1_BAUD);
}

void uart1_handler(void) {
  uart_handler(&uart1);
}

#endif /* USE_UART1 */

