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
#include "rt_priority.h"

#include <pthread.h>
#include <sys/select.h>

#ifndef UART_THREAD_PRIO
#define UART_THREAD_PRIO 11
#endif

static void uart_receive_handler(struct uart_periph *periph);
static void *uart_thread(void *data __attribute__((unused)));
static pthread_mutex_t uart_mutex = PTHREAD_MUTEX_INITIALIZER;

//#define TRACE(fmt,args...)    fprintf(stderr, fmt, args)
#define TRACE(fmt,args...)

void uart_arch_init(void)
{
  pthread_mutex_init(&uart_mutex, NULL);

  pthread_t tid;
  if (pthread_create(&tid, NULL, uart_thread, NULL) != 0) {
    fprintf(stderr, "uart_arch_init: Could not create UART reading thread.\n");
    return;
  }
  pthread_setname_np(tid, "pprz_uart_thread");
}

static void *uart_thread(void *data __attribute__((unused)))
{
  get_rt_prio(UART_THREAD_PRIO);

  /* file descriptor list */
  fd_set fds_master;
  /* maximum file descriptor number */
  int fdmax = 0;

  /* clear the fd list */
  FD_ZERO(&fds_master);
  /* add used fds */
  int __attribute__ ((unused)) fd;
#if USE_UART0
  if (uart0.reg_addr != NULL) {
    fd = ((struct SerialPort *)uart0.reg_addr)->fd;
    FD_SET(fd, &fds_master);
    if (fd > fdmax) {
      fdmax =fd;
    }
  }
#endif
#if USE_UART1
  if (uart1.reg_addr != NULL) {
    fd = ((struct SerialPort *)uart1.reg_addr)->fd;
    FD_SET(fd, &fds_master);
    if (fd > fdmax) {
      fdmax =fd;
    }
  }
#endif
#if USE_UART2
  if (uart2.reg_addr != NULL) {
    fd = ((struct SerialPort *)uart2.reg_addr)->fd;
    FD_SET(fd, &fds_master);
    if (fd > fdmax) {
      fdmax =fd;
    }
  }
#endif
#if USE_UART3
  if (uart3.reg_addr != NULL) {
    fd = ((struct SerialPort *)uart3.reg_addr)->fd;
    FD_SET(fd, &fds_master);
    if (fd > fdmax) {
      fdmax =fd;
    }
  }
#endif
#if USE_UART4
  if (uart4.reg_addr != NULL) {
    fd = ((struct SerialPort *)uart4.reg_addr)->fd;
    FD_SET(fd, &fds_master);
    if (fd > fdmax) {
      fdmax =fd;
    }
  }
#endif
#if USE_UART5
  if (uart5.reg_addr != NULL) {
    fd = ((struct SerialPort *)uart5.reg_addr)->fd;
    FD_SET(fd, &fds_master);
    if (fd > fdmax) {
      fdmax =fd;
    }
  }
#endif
#if USE_UART6
  if (uart6.reg_addr != NULL) {
    fd = ((struct SerialPort *)uart6.reg_addr)->fd;
    FD_SET(fd, &fds_master);
    if (fd > fdmax) {
      fdmax =fd;
    }
  }
#endif

  /* fds to be read, modified after each select */
  fd_set fds;

  while (1) {
    /* reset list of fds to check */
    fds = fds_master;

    if (select(fdmax + 1, &fds, NULL, NULL, NULL) < 0) {
      fprintf(stderr, "uart_thread: select failed!");
    }
    else {
#if USE_UART0
      if (uart0.reg_addr != NULL) {
        fd = ((struct SerialPort *)uart0.reg_addr)->fd;
        if (FD_ISSET(fd, &fds)) {
          uart_receive_handler(&uart0);
        }
      }
#endif
#if USE_UART1
      if (uart1.reg_addr != NULL) {
        fd = ((struct SerialPort *)uart1.reg_addr)->fd;
        if (FD_ISSET(fd, &fds)) {
          uart_receive_handler(&uart1);
        }
      }
#endif
#if USE_UART2
      if (uart2.reg_addr != NULL) {
        fd = ((struct SerialPort *)uart2.reg_addr)->fd;
        if (FD_ISSET(fd, &fds)) {
          uart_receive_handler(&uart2);
        }
      }
#endif
#if USE_UART3
      if (uart3.reg_addr != NULL) {
        fd = ((struct SerialPort *)uart3.reg_addr)->fd;
        if (FD_ISSET(fd, &fds)) {
          uart_receive_handler(&uart3);
        }
      }
#endif
#if USE_UART4
      if (uart4.reg_addr != NULL) {
        fd = ((struct SerialPort *)uart4.reg_addr)->fd;
        if (FD_ISSET(fd, &fds)) {
          uart_receive_handler(&uart4);
        }
      }
#endif
#if USE_UART5
      if (uart5.reg_addr != NULL) {
        fd = ((struct SerialPort *)uart5.reg_addr)->fd;
        if (FD_ISSET(fd, &fds)) {
          uart_receive_handler(&uart5);
        }
      }
#endif
#if USE_UART6
      if (uart6.reg_addr != NULL) {
        fd = ((struct SerialPort *)uart6.reg_addr)->fd;
        if (FD_ISSET(fd, &fds)) {
          uart_receive_handler(&uart6);
        }
      }
#endif
    }
  }

  return 0;
}

// open serial link
// close first if already openned
static void uart_periph_open(struct uart_periph *periph, uint32_t baud)
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
    serial_port_free(port);
    periph->reg_addr = NULL;
  }
}

void uart_periph_set_baudrate(struct uart_periph *periph, uint32_t baud)
{
  periph->baudrate = baud;

  // open serial port if not done
  if (periph->reg_addr == NULL) {
    uart_periph_open(periph, baud);
  }
  if (periph->reg_addr == NULL) {
    // periph not started, do nothiing
    return;
  }
  struct SerialPort *port = (struct SerialPort *)(periph->reg_addr);
  serial_port_set_baudrate(port, baud);
}

void uart_periph_set_bits_stop_parity(struct uart_periph *periph, uint8_t bits, uint8_t stop, uint8_t parity)
{
  if (periph->reg_addr == NULL) {
    // periph not started, do nothiing
    return;
  }
  struct SerialPort *port = (struct SerialPort *)(periph->reg_addr);
  serial_port_set_bits_stop_parity(port, bits, stop, parity);
}

void uart_put_byte(struct uart_periph *periph, long fd __attribute__((unused)), uint8_t data)
{
  if (periph->reg_addr == NULL) { return; } // device not initialized ?

  /* write single byte to serial port */
  struct SerialPort *port = (struct SerialPort *)(periph->reg_addr);

  int ret = 0;
  do{
    ret = write((int)(port->fd), &data, 1);
  } while(ret < 1 && errno == EAGAIN); //FIXME: max retry

  if (ret < 1) {
    TRACE("uart_put_byte: write %d failed [%d: %s]\n", data, ret, strerror(errno));
  }
}


static void __attribute__ ((unused)) uart_receive_handler(struct uart_periph *periph)
{
  unsigned char c = 'D';

  if (periph->reg_addr == NULL) { return; } // device not initialized ?

  struct SerialPort *port = (struct SerialPort *)(periph->reg_addr);
  int fd = port->fd;

  pthread_mutex_lock(&uart_mutex);

  if (read(fd, &c, 1) > 0) {
    //printf("r %x %c\n",c,c);
    uint16_t temp = (periph->rx_insert_idx + 1) % UART_RX_BUFFER_SIZE;
    // check for more room in queue
    if (temp != periph->rx_extract_idx) {
      periph->rx_buf[periph->rx_insert_idx] = c;
      periph->rx_insert_idx = temp;  // update insert index
    }
    else {
      TRACE("uart_receive_handler: rx_buf full! discarding received byte: %x %c\n", c, c);
    }
  }
  pthread_mutex_unlock(&uart_mutex);
}

uint8_t uart_getch(struct uart_periph *p)
{
  pthread_mutex_lock(&uart_mutex);
  uint8_t ret = p->rx_buf[p->rx_extract_idx];
  p->rx_extract_idx = (p->rx_extract_idx + 1) % UART_RX_BUFFER_SIZE;
  pthread_mutex_unlock(&uart_mutex);
  return ret;
}

uint16_t uart_char_available(struct uart_periph *p)
{
  pthread_mutex_lock(&uart_mutex);
  int16_t available = p->rx_insert_idx - p->rx_extract_idx;
  if (available < 0) {
    available += UART_RX_BUFFER_SIZE;
  }
  pthread_mutex_unlock(&uart_mutex);
  return (uint16_t)available;
}

#if USE_UART0
void uart0_init(void)
{
  uart_periph_init(&uart0);
  strncpy(uart0.dev, STRINGIFY(UART0_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart0, UART0_BAUD);
}
#endif /* USE_UART0 */

#if USE_UART1
void uart1_init(void)
{
  uart_periph_init(&uart1);
  strncpy(uart1.dev, STRINGIFY(UART1_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart1, UART1_BAUD);
}
#endif /* USE_UART1 */

#if USE_UART2
void uart2_init(void)
{
  uart_periph_init(&uart2);
  strncpy(uart2.dev, STRINGIFY(UART2_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart2, UART2_BAUD);
}
#endif /* USE_UART2 */

#if USE_UART3
void uart3_init(void)
{
  uart_periph_init(&uart3);
  strncpy(uart3.dev, STRINGIFY(UART3_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart3, UART3_BAUD);
}
#endif /* USE_UART3 */

#if USE_UART4
void uart4_init(void)
{
  uart_periph_init(&uart4);
  strncpy(uart4.dev, STRINGIFY(UART4_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart4, UART4_BAUD);
}
#endif /* USE_UART4 */

#if USE_UART5
void uart5_init(void)
{
  uart_periph_init(&uart5);
  strncpy(uart5.dev, STRINGIFY(UART5_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart5, UART5_BAUD);
}
#endif /* USE_UART5 */

#if USE_UART6
void uart6_init(void)
{
  uart_periph_init(&uart6);
  strncpy(uart6.dev, STRINGIFY(UART6_DEV), UART_DEV_NAME_SIZE);
  uart_periph_set_baudrate(&uart6, UART6_BAUD);
}
#endif /* USE_UART6 */
