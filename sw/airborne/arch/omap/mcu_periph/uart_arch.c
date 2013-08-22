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

// FIXME: uart.h defines B9600 as 9600
#include "mcu_periph/uart.h"

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// FIXME: fms_serial_port includes termios.h that with omap defines B9600 as 12
// Include termios.h AFTER uart.h. This OVERWRITES (without warning) the paparazzi uart.h B9600
#include "fms/fms_serial_port.h"

// #define TRACE(fmt,args...)    fprintf(stderr, fmt, args)
#define TRACE(fmt,args...)

// Convert the paparazzi B9600 which becomes 9600 to termios B9600 which becomes 12
// FIXME: not all possible baudrate are available yet.
speed_t baudconvert_drone(uint32_t baud);
speed_t baudconvert_drone(uint32_t baud)
{
  if (baud <= 4800)
    return B4800;
  else if (baud <= 9600)
    return B9600;
  else if (baud <= 19200)
    return B19200;
  else if (baud <= 38400)
    return B38400;
  else if (baud <= 57600)
    return B57600;
  return B115200;
}


void uart_periph_set_baudrate(struct uart_periph* p, uint32_t baud) {
  struct FmsSerialPort* fmssp;
  // close serial port if already open
  if (p->reg_addr != NULL) {
    fmssp = (struct FmsSerialPort*)(p->reg_addr);
    serial_port_close(fmssp);
    serial_port_free(fmssp);
  }
  // open serial port
  fmssp = serial_port_new();
  // use register address to store SerialPort structure pointer...
  p->reg_addr = (void*)fmssp;

  //TODO: set device name in application and pass as argument
  // FIXME: paparazzi baud is 9600 for B9600 while open_raw needs 12 for B9600
  printf("opening %s on uart0 at %d (termios.h value=%d)\n",p->dev,baud,baudconvert_drone(baud));
  int ret = serial_port_open_raw(fmssp,p->dev,baudconvert_drone(baud));
  if (ret != 0)
  {
    TRACE("Error opening %s code %d\n",p->dev,ret);
  }
}

void uart_transmit(struct uart_periph* p, uint8_t data ) {
  uint16_t temp = (p->tx_insert_idx + 1) % UART_TX_BUFFER_SIZE;

  if (temp == p->tx_extract_idx)
    return;                          // no room

  // check if in process of sending data
  if (p->tx_running) { // yes, add to queue
    p->tx_buf[p->tx_insert_idx] = data;
    p->tx_insert_idx = temp;
  }
  else { // no, set running flag and write to output register
    p->tx_running = TRUE;
    struct FmsSerialPort* fmssp = (struct FmsSerialPort*)(p->reg_addr);
    int ret = write((int)(fmssp->fd),&data,1);
    if (ret < 1)
    {
      TRACE("w %x [%d]\n",data,ret);
    }
  }
}

#include <errno.h>

static inline void uart_handler(struct uart_periph* p) {
  unsigned char c='D';

  if (p->reg_addr == NULL) return; // device not initialized ?

  struct FmsSerialPort* fmssp = (struct FmsSerialPort*)(p->reg_addr);
  int fd = fmssp->fd;

  // check if more data to send
  if (p->tx_insert_idx != p->tx_extract_idx) {
    int ret = write(fd,&(p->tx_buf[p->tx_extract_idx]),1);
    if (ret < 1)
    {
      TRACE("w %x [%d: %s]\n",p->tx_buf[p->tx_extract_idx],ret,strerror(errno));
    }
    p->tx_extract_idx++;
    p->tx_extract_idx %= UART_TX_BUFFER_SIZE;
  }
  else {
    p->tx_running = FALSE;   // clear running flag
  }

  if(read(fd,&c,1) > 0){
    //printf("r %x %c\n",c,c);
    uint16_t temp = (p->rx_insert_idx + 1) % UART_RX_BUFFER_SIZE;
    p->rx_buf[p->rx_insert_idx] = c;
    // check for more room in queue
    if (temp != p->rx_extract_idx)
      p->rx_insert_idx = temp; // update insert index
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

