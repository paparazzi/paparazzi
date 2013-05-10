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

#include "fms/fms_serial_port.h"


void uart_periph_set_baudrate(struct uart_periph* p, uint32_t baud, bool_t hw_flow_control __attribute__ ((unused))) {
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
  printf("opening %s on uart0 at %d\n",p->dev,baud);
  serial_port_open_raw(fmssp,p->dev,baud);
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
    write((int)(fmssp->fd),&data,1);
    //printf("w %x\n",data);
  }
}

static inline void uart_handler(struct uart_periph* p) {
  unsigned char c='D';

  if (p->reg_addr == NULL) return; // device not initialized ?

  struct FmsSerialPort* fmssp = (struct FmsSerialPort*)(p->reg_addr);
  int fd = fmssp->fd;

  // check if more data to send
  if (p->tx_insert_idx != p->tx_extract_idx) {
    write(fd,&(p->tx_buf[p->tx_extract_idx]),1);
    //printf("w %x\n",p->tx_buf[p->tx_extract_idx]);
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
  uart_periph_set_baudrate(&uart0,UART0_BAUD,FALSE);
}


void uart0_handler(void) {
  uart_handler(&uart0);
}

#endif /* USE_UART0 */

#ifdef USE_UART1

void uart1_init( void ) {
  uart_periph_init(&uart1);
  strcpy(uart1.dev, UART1_DEV);
  uart_periph_set_baudrate(&uart1,UART1_BAUD,FALSE);
}

void uart1_handler(void) {
  uart_handler(&uart1);
}

#endif /* USE_UART1 */

