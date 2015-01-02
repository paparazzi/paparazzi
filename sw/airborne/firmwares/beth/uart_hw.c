/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

#include "mcu_periph/uart.h"

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "serial_port.h"


#if USE_UART0

volatile uint16_t uart0_rx_insert_idx, uart0_rx_extract_idx;
uint8_t  uart0_rx_buffer[UART0_RX_BUFFER_SIZE];

volatile uint16_t uart0_tx_insert_idx, uart0_tx_extract_idx;
volatile bool_t uart0_tx_running;
uint8_t  uart0_tx_buffer[UART0_TX_BUFFER_SIZE];

struct SerialPort *fmssp0;
int uart0_fd;
extern uint8_t portnum;


//This function will close our UART and reopen with the new baud rate
#ifdef GPS_CONFIGURE
void uart0_init_param(uint16_t baud, uint8_t mode, uint8_t fmode)
{

  //serial_port_flush_output(fmssp0);
  serial_port_close(fmssp0);
  fmssp0 = serial_port_new();

  if (portnum == 0) {
    printf("opening ttyUSB0 on uart0 at %d\n", GPS_BAUD);
    serial_port_open_raw(fmssp0, "/dev/ttyUSB0", GPS_BAUD);
  }
  if (portnum == 1) {
    printf("opening ttyUSB1 on uart0 at %d\n", GPS_BAUD);
    serial_port_open_raw(fmssp0, "/dev/ttyUSB1", GPS_BAUD);
  }

  uart0_fd = (int)fmssp0->fd;

  // initialize the transmit data queue
  uart0_tx_extract_idx = 0;
  uart0_tx_insert_idx = 0;
  uart0_tx_running = FALSE;

  // initialize the receive data queue
  uart0_rx_extract_idx = 0;
  uart0_rx_insert_idx = 0;

}
#endif

void uart0_init(void)
{

  fmssp0 = serial_port_new();


//TODO: set device name in application and pass as argument
  if (portnum == 0) {
    printf("opening ttyUSB0 on uart0 at %d\n", UART0_BAUD);
    serial_port_open_raw(fmssp0, "/dev/ttyUSB0", UART0_BAUD);
  }
  if (portnum == 1) {
    printf("opening ttyUSB1 on uart0 at %d\n", UART0_BAUD);
    serial_port_open_raw(fmssp0, "/dev/ttyUSB1", UART0_BAUD);
  }
  uart0_fd = (int)fmssp0->fd;

  // initialize the transmit data queue
  uart0_tx_extract_idx = 0;
  uart0_tx_insert_idx = 0;
  uart0_tx_running = FALSE;

  // initialize the receive data queue
  uart0_rx_extract_idx = 0;
  uart0_rx_insert_idx = 0;

}

void uart0_transmit(uint8_t data)
{

  uint16_t temp = (uart0_tx_insert_idx + 1) % UART0_TX_BUFFER_SIZE;

  if (temp == uart0_tx_extract_idx) {
    return;  // no room
  }

  // check if in process of sending data
  if (uart0_tx_running) { // yes, add to queue
    uart0_tx_buffer[uart0_tx_insert_idx] = data;
    uart0_tx_insert_idx = temp;
  } else { // no, set running flag and write to output register
    uart0_tx_running = TRUE;
    write(uart0_fd, &data, 1);
    //printf("w %x\n",data);
  }

}

bool_t uart0_check_free_space(uint8_t len)
{
  int16_t space = uart0_tx_extract_idx - uart0_tx_insert_idx;
  if (space <= 0) {
    space += UART0_TX_BUFFER_SIZE;
  }
  return (uint16_t)(space - 1) >= len;
}

void uart0_handler(void)
{
  unsigned char c = 'D';

  // check if more data to send
  if (uart0_tx_insert_idx != uart0_tx_extract_idx) {
    write(uart0_fd, &uart0_tx_buffer[uart0_tx_extract_idx], 1);
    //printf("w %x\n",uart0_tx_buffer[uart0_tx_extract_idx]);
    uart0_tx_extract_idx++;
    uart0_tx_extract_idx %= UART0_TX_BUFFER_SIZE;
  } else {
    uart0_tx_running = FALSE;   // clear running flag
  }

  if (read(uart0_fd, &c, 1) > 0) {
    //printf("r %x %c\n",c,c);
    uint16_t temp = (uart0_rx_insert_idx + 1) % UART0_RX_BUFFER_SIZE;
    uart0_rx_buffer[uart0_rx_insert_idx] = c;
    // check for more room in queue
    if (temp != uart0_rx_extract_idx) {
      uart0_rx_insert_idx = temp;  // update insert index
    }
  }

}

#endif /* USE_UART0 */

#if USE_UART1

volatile uint16_t uart1_rx_insert_idx, uart1_rx_extract_idx;
uint8_t  uart1_rx_buffer[UART1_RX_BUFFER_SIZE];

volatile uint16_t uart1_tx_insert_idx, uart1_tx_extract_idx;
volatile bool_t uart1_tx_running;
uint8_t  uart1_tx_buffer[UART1_TX_BUFFER_SIZE];

struct SerialPort *fmssp1;
int uart1_fd;

void uart1_init(void)
{

  fmssp1 = serial_port_new();

  if (portnum == 0) {
    printf("opening ttyUSB1 on uart1 at %d\n", UART1_BAUD);
    serial_port_open_raw(fmssp1, "/dev/ttyUSB1", UART1_BAUD);
  }
  if (portnum == 1) {
    printf("opening ttyUSB0 on uart1 at %d\n", UART1_BAUD);
    serial_port_open_raw(fmssp1, "/dev/ttyUSB0", UART1_BAUD);
  }

  uart1_fd = (int)fmssp1->fd;

  // initialize the transmit data queue
  uart1_tx_extract_idx = 0;
  uart1_tx_insert_idx = 0;
  uart1_tx_running = FALSE;

  // initialize the receive data queue
  uart1_rx_extract_idx = 0;
  uart1_rx_insert_idx = 0;

}

void uart1_transmit(uint8_t data)
{

  uint16_t temp = (uart1_tx_insert_idx + 1) % UART1_TX_BUFFER_SIZE;

  if (temp == uart1_tx_extract_idx) {
    return;  // no room
  }

  // check if in process of sending data
  if (uart1_tx_running) { // yes, add to queue
    uart1_tx_buffer[uart1_tx_insert_idx] = data;
    uart1_tx_insert_idx = temp;
  } else { // no, set running flag and write to output register
    uart1_tx_running = TRUE;
    //printf("z %x\n",data);
    write(uart1_fd, &data, 1);
  }

}

bool_t uart1_check_free_space(uint8_t len)
{
  int16_t space = uart1_tx_extract_idx - uart1_tx_insert_idx;
  if (space <= 0) {
    space += UART1_TX_BUFFER_SIZE;
  }
  return (uint16_t)(space - 1) >= len;
}

void uart1_handler(void)
{
  unsigned char c = 'D';

  // check if more data to send
  if (uart1_tx_insert_idx != uart1_tx_extract_idx) {
    write(uart1_fd, &uart1_tx_buffer[uart1_tx_extract_idx], 1);
    //printf("z %x\n",uart1_tx_buffer[uart1_tx_extract_idx]);
    uart1_tx_extract_idx++;
    uart1_tx_extract_idx %= UART1_TX_BUFFER_SIZE;
  } else {
    uart1_tx_running = FALSE;   // clear running flag
  }

  if (read(uart1_fd, &c, 1) > 0) {
    //printf("s %x %c\n",c,c);
    uint16_t temp = (uart1_rx_insert_idx + 1) % UART1_RX_BUFFER_SIZE;;
    uart1_rx_buffer[uart1_rx_insert_idx] = c;
    // check for more room in queue
    if (temp != uart1_rx_extract_idx) {
      uart1_rx_insert_idx = temp;  // update insert index
    }
  }

}

#endif /* USE_UART1 */

void uart_init(void)
{
#if USE_UART0
  uart0_init();
#endif
#if USE_UART1
  uart1_init();
#endif
//TODO: add uart2 and greater
#if USE_UART2
  uart2_init();
#endif
#if USE_UART3
  uart3_init();
#endif
}
