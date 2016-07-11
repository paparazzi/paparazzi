/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
 * Alexandre Bustico <alexandre.bustico@enac.fr>
 * Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
/**
 * @file arch/chibios/mcu_periph/uart_arch.c
 * UART/Serial driver implementation for ChibiOS arch
 *
 * ChibiOS has a high level Serial Driver, for Paparazzi it is more convenient
 * than pure UART driver (which needs callbacks etc.). This implementation is
 * asynchronous and the RX thread has to use event flags. See ChibiOS documen-
 * tation.
 */
#include "mcu_periph/uart_arch.h"
#include <ch.h>
#include <hal.h>

struct SerialInit {
  semaphore_t *rx_sem;
  semaphore_t *tx_sem;
  mutex_t *rx_mtx;
  mutex_t *tx_mtx;
};

/**
 * RX handler
 */
static void handle_uart_rx(struct uart_periph *p)
{
  // wait for next incoming byte
  uint8_t c = sdGet((SerialDriver*)(p->reg_addr));

  struct SerialInit *init_struct = (struct SerialInit*)(p->init_struct);
  chMtxLock(init_struct->rx_mtx);
  uint16_t temp = (p->rx_insert_idx + 1) % UART_RX_BUFFER_SIZE;;
  // insert new byte
  p->rx_buf[p->rx_insert_idx] = c;
  // check for more room in queue
  if (temp != p->rx_extract_idx) {
    p->rx_insert_idx = temp;  // update insert index
  }
  chMtxUnlock(init_struct->rx_mtx);
  chSemSignal(init_struct->rx_sem);
}

/**
 * TX handler
 */
static void handle_uart_tx(struct uart_periph *p)
{
  // check if more data to send
  struct SerialInit *init_struct = (struct SerialInit*)(p->init_struct);
  chSemWait (init_struct->tx_sem);
  while (p->tx_insert_idx != p->tx_extract_idx) {
    uint8_t data = p->tx_buf[p->tx_extract_idx];
    p->tx_running = true;
    sdWrite((SerialDriver *)p->reg_addr, &data, sizeof(data));
    p->tx_running = false;
    // TODO send by block (be careful with circular buffer)
    chMtxLock(init_struct->tx_mtx);
    p->tx_extract_idx++;
    p->tx_extract_idx %= UART_TX_BUFFER_SIZE;
    chMtxUnlock(init_struct->tx_mtx);
  }
}

#if USE_UART1

#ifndef UART1_BAUD
#define UART1_BAUD SERIAL_DEFAULT_BITRATE
#endif

/* by default enable UART Tx and Rx */
#ifndef USE_UART1_TX
#define USE_UART1_TX TRUE
#endif
#ifndef USE_UART1_RX
#define USE_UART1_RX TRUE
#endif

static const SerialConfig usart1_config = {
  UART1_BAUD,                                             /*     BITRATE    */
  0,                                                      /*    USART CR1   */
  USART_CR2_STOP1_BITS,                                   /*    USART CR2   */
  0                                                       /*    USART CR3   */
};

static struct SerialInit uart1_init_struct = { NULL, NULL, NULL, NULL };

// Threads RX and TX
#if USE_UART1_RX
static MUTEX_DECL(uart1_rx_mtx);
static SEMAPHORE_DECL(uart1_rx_sem, 0);

static __attribute__((noreturn)) void thd_uart1_rx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart1_rx");

  while (TRUE) {
    handle_uart_rx(&uart1);
  }
}

static THD_WORKING_AREA(wa_thd_uart1_rx, 1024);
#endif

#if USE_UART1_TX
static MUTEX_DECL(uart1_tx_mtx);
static SEMAPHORE_DECL(uart1_tx_sem, 0);

static __attribute__((noreturn)) void thd_uart1_tx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart1_tx");

  while (TRUE) {
    handle_uart_tx(&uart1);
  }
}
static THD_WORKING_AREA(wa_thd_uart1_tx, 1024);
#endif

void uart1_init(void)
{
  uart_periph_init(&uart1);
  sdStart(&SD1, &usart1_config);
  uart1.reg_addr = &SD1;
  uart1.init_struct = &uart1_init_struct;

  // Create threads
#if USE_UART1_RX
  uart1_init_struct.rx_mtx = &uart1_rx_mtx;
  uart1_init_struct.rx_sem = &uart1_rx_sem;
  chThdCreateStatic(wa_thd_uart1_rx, sizeof(wa_thd_uart1_rx),
      NORMALPRIO+1, thd_uart1_rx, NULL);
#endif
#if USE_UART1_TX
  uart1_init_struct.tx_mtx = &uart1_tx_mtx;
  uart1_init_struct.tx_sem = &uart1_tx_sem;
  chThdCreateStatic(wa_thd_uart1_tx, sizeof(wa_thd_uart1_tx),
      NORMALPRIO+1, thd_uart1_tx, NULL);
#endif
}

#endif


#if USE_UART2

#ifndef UART2_BAUD
#define UART2_BAUD SERIAL_DEFAULT_BITRATE
#endif

/* by default enable UART Tx and Rx */
#ifndef USE_UART2_TX
#define USE_UART2_TX TRUE
#endif
#ifndef USE_UART2_RX
#define USE_UART2_RX TRUE
#endif

static const SerialConfig usart2_config = {
  UART2_BAUD,                                               /*     BITRATE    */
  0,                                                        /*    USART CR1   */
  USART_CR2_STOP1_BITS,                                     /*    USART CR2   */
  0                                                         /*    USART CR3   */
};

static struct SerialInit uart2_init_struct = { NULL, NULL, NULL, NULL };

// Threads RX and TX
#if USE_UART2_RX
static MUTEX_DECL(uart2_rx_mtx);
static SEMAPHORE_DECL(uart2_rx_sem, 0);

static __attribute__((noreturn)) void thd_uart2_rx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart2_rx");

  while (TRUE) {
    handle_uart_rx(&uart2);
  }
}

static THD_WORKING_AREA(wa_thd_uart2_rx, 1024);
#endif

#if USE_UART2_TX
static MUTEX_DECL(uart2_tx_mtx);
static SEMAPHORE_DECL(uart2_tx_sem, 0);

static __attribute__((noreturn)) void thd_uart2_tx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart2_tx");

  while (TRUE) {
    handle_uart_tx(&uart2);
  }
}
static THD_WORKING_AREA(wa_thd_uart2_tx, 1024);
#endif

void uart2_init(void)
{
  uart_periph_init(&uart2);
  sdStart(&SD2, &usart2_config);
  uart2.reg_addr = &SD2;
  uart2.init_struct = &uart2_init_struct;

  // Create threads
#if USE_UART2_RX
  uart2_init_struct.rx_mtx = &uart2_rx_mtx;
  uart2_init_struct.rx_sem = &uart2_rx_sem;
  chThdCreateStatic(wa_thd_uart2_rx, sizeof(wa_thd_uart2_rx),
      NORMALPRIO, thd_uart2_rx, NULL);
#endif
#if USE_UART2_TX
  uart2_init_struct.tx_mtx = &uart2_tx_mtx;
  uart2_init_struct.tx_sem = &uart2_tx_sem;
  chThdCreateStatic(wa_thd_uart2_tx, sizeof(wa_thd_uart2_tx),
      NORMALPRIO, thd_uart2_tx, NULL);
#endif
}

#endif

#if USE_UART3

#ifndef UART3_BAUD
#define UART3_BAUD SERIAL_DEFAULT_BITRATE
#endif

/* by default enable UART Tx and Rx */
#ifndef USE_UART3_TX
#define USE_UART3_TX TRUE
#endif
#ifndef USE_UART3_RX
#define USE_UART3_RX TRUE
#endif

static const SerialConfig usart3_config = {
  UART3_BAUD,                                             /*     BITRATE    */
  0,                                                      /*    USART CR1   */
  USART_CR2_STOP1_BITS,                                   /*    USART CR2   */
  0                                                       /*    USART CR3   */
};

static struct SerialInit uart3_init_struct = { NULL, NULL, NULL, NULL };

// Threads RX and TX
#if USE_UART3_RX
static MUTEX_DECL(uart3_rx_mtx);
static SEMAPHORE_DECL(uart3_rx_sem, 0);

static __attribute__((noreturn)) void thd_uart3_rx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart3_rx");

  while (TRUE) {
    handle_uart_rx(&uart3);
  }
}

static THD_WORKING_AREA(wa_thd_uart3_rx, 1024);
#endif

#if USE_UART3_TX
static MUTEX_DECL(uart3_tx_mtx);
static SEMAPHORE_DECL(uart3_tx_sem, 0);

static __attribute__((noreturn)) void thd_uart3_tx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart3_tx");

  while (TRUE) {
    handle_uart_tx(&uart3);
  }
}
static THD_WORKING_AREA(wa_thd_uart3_tx, 1024);
#endif

void uart3_init(void)
{
  uart_periph_init(&uart3);
  sdStart(&SD3, &usart3_config);
  uart3.reg_addr = &SD3;
  uart3.init_struct = &uart3_init_struct;

  // Create threads
#if USE_UART3_RX
  uart3_init_struct.rx_mtx = &uart3_rx_mtx;
  uart3_init_struct.rx_sem = &uart3_rx_sem;
  chThdCreateStatic(wa_thd_uart3_rx, sizeof(wa_thd_uart3_rx),
      NORMALPRIO, thd_uart3_rx, NULL);
#endif
#if USE_UART3_TX
  uart3_init_struct.tx_mtx = &uart3_tx_mtx;
  uart3_init_struct.tx_sem = &uart3_tx_sem;
  chThdCreateStatic(wa_thd_uart3_tx, sizeof(wa_thd_uart3_tx),
      NORMALPRIO, thd_uart3_tx, NULL);
#endif
}

#endif

#if USE_UART4

#ifndef UART4_BAUD
#define UART4_BAUD SERIAL_DEFAULT_BITRATE
#endif

/* by default enable UART Tx and Rx */
#ifndef USE_UART4_TX
#define USE_UART4_TX TRUE
#endif
#ifndef USE_UART4_RX
#define USE_UART4_RX TRUE
#endif

static const SerialConfig usart4_config = {
  UART4_BAUD,                                             /*     BITRATE    */
  0,                                                      /*    USART CR1   */
  USART_CR2_STOP1_BITS,                                   /*    USART CR2   */
  0                                                       /*    USART CR3   */
};

static struct SerialInit uart4_init_struct = { NULL, NULL, NULL, NULL };

// Threads RX and TX
#if USE_UART4_RX
static MUTEX_DECL(uart4_rx_mtx);
static SEMAPHORE_DECL(uart4_rx_sem, 0);

static __attribute__((noreturn)) void thd_uart4_rx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart4_rx");

  while (TRUE) {
    handle_uart_rx(&uart4);
  }
}

static THD_WORKING_AREA(wa_thd_uart4_rx, 1024);
#endif

#if USE_UART4_TX
static MUTEX_DECL(uart4_tx_mtx);
static SEMAPHORE_DECL(uart4_tx_sem, 0);

static __attribute__((noreturn)) void thd_uart4_tx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart4_tx");

  while (TRUE) {
    handle_uart_tx(&uart4);
  }
}
static THD_WORKING_AREA(wa_thd_uart4_tx, 1024);
#endif

void uart4_init(void)
{
  uart_periph_init(&uart4);
  sdStart(&SD4, &usart4_config);
  uart4.reg_addr = &SD4;
  uart4.init_struct = &uart4_init_struct;

  // Create threads
#if USE_UART4_RX
  uart4_init_struct.rx_mtx = &uart4_rx_mtx;
  uart4_init_struct.rx_sem = &uart4_rx_sem;
  chThdCreateStatic(wa_thd_uart4_rx, sizeof(wa_thd_uart4_rx),
      NORMALPRIO, thd_uart4_rx, NULL);
#endif
#if USE_UART4_TX
  uart4_init_struct.tx_mtx = &uart4_tx_mtx;
  uart4_init_struct.tx_sem = &uart4_tx_sem;
  chThdCreateStatic(wa_thd_uart4_tx, sizeof(wa_thd_uart4_tx),
      NORMALPRIO, thd_uart4_tx, NULL);
#endif
}

#endif

#if USE_UART5

#ifndef UART5_BAUD
#define UART5_BAUD SERIAL_DEFAULT_BITRATE
#endif

/* by default enable UART Tx and Rx */
#ifndef USE_UART5_TX
#define USE_UART5_TX TRUE
#endif
#ifndef USE_UART5_RX
#define USE_UART5_RX TRUE
#endif

static const SerialConfig usart5_config = {
  UART5_BAUD,                                             /*     BITRATE    */
  0,                                                      /*    USART CR1   */
  USART_CR2_STOP1_BITS,                                   /*    USART CR2   */
  0                                                       /*    USART CR3   */
};

static struct SerialInit uart5_init_struct = { NULL, NULL, NULL, NULL };

// Threads RX and TX
#if USE_UART5_RX
static MUTEX_DECL(uart5_rx_mtx);
static SEMAPHORE_DECL(uart5_rx_sem, 0);

static __attribute__((noreturn)) void thd_uart5_rx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart5_rx");

  while (TRUE) {
    handle_uart_rx(&uart5);
  }
}

static THD_WORKING_AREA(wa_thd_uart5_rx, 1024);
#endif

#if USE_UART5_TX
static MUTEX_DECL(uart5_tx_mtx);
static SEMAPHORE_DECL(uart5_tx_sem, 0);

static __attribute__((noreturn)) void thd_uart5_tx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart5_tx");

  while (TRUE) {
    handle_uart_tx(&uart5);
  }
}
static THD_WORKING_AREA(wa_thd_uart5_tx, 1024);
#endif

void uart5_init(void)
{
  uart_periph_init(&uart5);
  sdStart(&SD5, &usart5_config);
  uart5.reg_addr = &SD5;
  uart5.init_struct = &uart5_init_struct;

  // Create threads
#if USE_UART5_RX
  uart5_init_struct.rx_mtx = &uart5_rx_mtx;
  uart5_init_struct.rx_sem = &uart5_rx_sem;
  chThdCreateStatic(wa_thd_uart5_rx, sizeof(wa_thd_uart5_rx),
      NORMALPRIO, thd_uart5_rx, NULL);
#endif
#if USE_UART5_TX
  uart5_init_struct.tx_mtx = &uart5_tx_mtx;
  uart5_init_struct.tx_sem = &uart5_tx_sem;
  chThdCreateStatic(wa_thd_uart5_tx, sizeof(wa_thd_uart5_tx),
      NORMALPRIO, thd_uart5_tx, NULL);
#endif
}

#endif

#if USE_UART6

#ifndef UART6_BAUD
#define UART6_BAUD SERIAL_DEFAULT_BITRATE
#endif

/* by default enable UART Tx and Rx */
#ifndef USE_UART6_TX
#define USE_UART6_TX TRUE
#endif
#ifndef USE_UART6_RX
#define USE_UART6_RX TRUE
#endif

static const SerialConfig usart6_config = {
  UART6_BAUD,                                             /*     BITRATE    */
  0,                                                      /*    USART CR1   */
  USART_CR2_STOP1_BITS,                                   /*    USART CR2   */
  0                                                       /*    USART CR3   */
};

static struct SerialInit uart6_init_struct = { NULL, NULL, NULL, NULL };

// Threads RX and TX
#if USE_UART6_RX
static MUTEX_DECL(uart6_rx_mtx);
static SEMAPHORE_DECL(uart6_rx_sem, 0);

static __attribute__((noreturn)) void thd_uart6_rx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart6_rx");

  while (TRUE) {
    handle_uart_rx(&uart6);
  }
}

static THD_WORKING_AREA(wa_thd_uart6_rx, 1024);
#endif

#if USE_UART6_TX
static MUTEX_DECL(uart6_tx_mtx);
static SEMAPHORE_DECL(uart6_tx_sem, 0);

static __attribute__((noreturn)) void thd_uart6_tx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart6_tx");

  while (TRUE) {
    handle_uart_tx(&uart6);
  }
}
static THD_WORKING_AREA(wa_thd_uart6_tx, 1024);
#endif

void uart6_init(void)
{
  uart_periph_init(&uart6);
  sdStart(&SD6, &usart6_config);
  uart6.reg_addr = &SD6;
  uart6.init_struct = &uart6_init_struct;

  // Create threads
#if USE_UART6_RX
  uart6_init_struct.rx_mtx = &uart6_rx_mtx;
  uart6_init_struct.rx_sem = &uart6_rx_sem;
  chThdCreateStatic(wa_thd_uart6_rx, sizeof(wa_thd_uart6_rx),
      NORMALPRIO, thd_uart6_rx, NULL);
#endif
#if USE_UART6_TX
  uart6_init_struct.tx_mtx = &uart6_tx_mtx;
  uart6_init_struct.tx_sem = &uart6_tx_sem;
  chThdCreateStatic(wa_thd_uart6_tx, sizeof(wa_thd_uart6_tx),
      NORMALPRIO, thd_uart6_tx, NULL);
#endif
}

#endif


uint8_t uart_getch(struct uart_periph *p)
{
  //to keep compatibility with loop oriented paparazzi architecture, read is not blocking
  //struct SerialInit *init_struct = (struct SerialInit*)(p->init_struct);
  //chSemWait (init_struct->rx_sem);
  struct SerialInit *init_struct = (struct SerialInit*)(p->init_struct);
  chMtxLock(init_struct->rx_mtx);
  uint8_t ret = p->rx_buf[p->rx_extract_idx];
  p->rx_extract_idx = (p->rx_extract_idx + 1) % UART_RX_BUFFER_SIZE;
  chMtxUnlock(init_struct->rx_mtx);
  return ret;
}

/**
 * Set baudrate (from the serialConfig)
 * @note Baudrate is set in sdStart, no need for implementation
 */
void uart_periph_set_baudrate(struct uart_periph *p __attribute__((unused)), uint32_t baud __attribute__((unused))) {}

/**
 * Set mode (not necessary, or can be set by SerialConfig)
 */
void uart_periph_set_mode(struct uart_periph *p __attribute__((unused)), bool tx_enabled __attribute__((unused)),
                          bool rx_enabled __attribute__((unused)), bool hw_flow_control __attribute__((unused))) {}

void uart_periph_set_bits_stop_parity(struct uart_periph *p __attribute__((unused)),
                                      uint8_t bits __attribute__((unused)), uint8_t stop __attribute__((unused)), uint8_t __attribute__((unused)) parity)
{
  // TBD
}

// Check free space and set a positive value for fd if valid
// and lock driver with mutex
bool uart_check_free_space(struct uart_periph *p, long *fd, uint16_t len)
{
  struct SerialInit *init_struct = (struct SerialInit*)(p->init_struct);
  int16_t space = p->tx_extract_idx - p->tx_insert_idx;
  if (space <= 0) {
    space += UART_TX_BUFFER_SIZE;
  }
  if ((uint16_t)(space - 1) >= len) {
    *fd = 1;
    chMtxLock(init_struct->tx_mtx);
    return true;
  }
  return false;
}

/**
* Uart transmit implementation
*/
void uart_put_byte(struct uart_periph *p, long fd, uint8_t data)
{
  struct SerialInit *init_struct = (struct SerialInit*)(p->init_struct);
  if (fd == 0) {
    // if fd is zero, assume the driver is not already locked
    chMtxLock(init_struct->tx_mtx);
    uint16_t temp = (p->tx_insert_idx + 1) % UART_TX_BUFFER_SIZE;
    if (temp == p->tx_extract_idx) {
      chMtxUnlock(init_struct->tx_mtx);
      return;  // no room
    }
    p->tx_buf[p->tx_insert_idx] = data;
    p->tx_insert_idx = temp;

    chMtxUnlock(init_struct->tx_mtx);
    // send signal to start transmission
    chSemSignal (init_struct->tx_sem);
  }
  else {
    // assume driver is locked and available space have been checked
    p->tx_buf[p->tx_insert_idx] = data;
    p->tx_insert_idx = (p->tx_insert_idx + 1) % UART_TX_BUFFER_SIZE;
  }
}

/**
 * Uart transmit buffer implementation
 */
void uart_put_buffer(struct uart_periph *p, long fd, const uint8_t *data, uint16_t len)
{
  struct SerialInit *init_struct = (struct SerialInit*)(p->init_struct);
  if (fd == 0) {
    // if fd is zero, assume the driver is not already locked
    // and available space should be checked
    chMtxLock(init_struct->tx_mtx);
    int16_t space = p->tx_extract_idx - p->tx_insert_idx;
    if (space <= 0) {
      space += UART_TX_BUFFER_SIZE;
    }
    if ((uint16_t)(space - 1) < len) {
      chMtxUnlock(init_struct->tx_mtx);
      return;  // no room
    }
  }
  // insert data into buffer
  int i;
  for (i = 0; i < len; i++) {
    p->tx_buf[p->tx_insert_idx] = data[i];
    p->tx_insert_idx = (p->tx_insert_idx + 1) % UART_TX_BUFFER_SIZE;
  }
  // unlock if needed
  if (fd == 0) {
    chMtxUnlock(init_struct->tx_mtx);
    // send signal to start transmission
    chSemSignal (init_struct->tx_sem);
  }
}

void uart_send_message(struct uart_periph *p, long fd)
{
  struct SerialInit *init_struct = (struct SerialInit*)(p->init_struct);
  // unlock driver in case it is not done (fd > 0)
  if (fd != 0) {
    chMtxUnlock(init_struct->tx_mtx);
  }
  // send signal to start transmission
  chSemSignal (init_struct->tx_sem);
}

