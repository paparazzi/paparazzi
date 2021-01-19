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
#include "mcu_periph/gpio.h"
#include BOARD_CONFIG

// Default stack size
#ifndef UART_THREAD_STACK_SIZE
#define UART_THREAD_STACK_SIZE 512
#endif

struct SerialInit {
  SerialConfig *conf;
  semaphore_t *rx_sem;
  semaphore_t *tx_sem;
  mutex_t *rx_mtx;
  mutex_t *tx_mtx;
  ioportid_t cts_port;
  uint16_t cts_pin;
};

#define SERIAL_INIT_NULL { NULL, NULL, NULL, NULL, NULL, 0, 0 }

/**
 * RX handler
 */
static void handle_uart_rx(struct uart_periph *p)
{
  // wait for next incoming byte
  uint8_t c = sdGet((SerialDriver *)(p->reg_addr));

  struct SerialInit *init_struct = (struct SerialInit *)(p->init_struct);
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
  // TODO send by block with sdWrite (be careful with circular buffer)
  // not compatible with soft flow control
  struct SerialInit *init_struct = (struct SerialInit *)(p->init_struct);
  chSemWait(init_struct->tx_sem);
  p->tx_running = true;
  while (p->tx_insert_idx != p->tx_extract_idx) {
#if USE_UART_SOFT_FLOW_CONTROL
    if (init_struct->cts_port != 0) {
      // wait for CTS line to be set to send next byte
      while (gpio_get(init_struct->cts_port, init_struct->cts_pin) == 1) ;
    }
#endif
    uint8_t data = p->tx_buf[p->tx_extract_idx];
    sdPut((SerialDriver *)p->reg_addr, data);
    chMtxLock(init_struct->tx_mtx);
    p->tx_extract_idx++;
    p->tx_extract_idx %= UART_TX_BUFFER_SIZE;
    chMtxUnlock(init_struct->tx_mtx);
#if USE_UART_SOFT_FLOW_CONTROL
    if (init_struct->cts_port != 0) {
      // wait for physical transfer to be completed
      while ((((SerialDriver *)p->reg_addr)->usart->SR & USART_SR_TC) == 0) ;
    }
#endif
  }
  p->tx_running = false;
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

#ifndef UART1_CR1
#define UART1_CR1 0
#endif

#ifndef UART1_CR2
#define UART1_CR2 USART_CR2_STOP1_BITS
#endif

#ifndef UART1_CR3
#define UART1_CR3 0
#endif

static SerialConfig usart1_config = {
  UART1_BAUD,                                  /*     BITRATE    */
  UART1_CR1,                                   /*    USART CR1   */
  UART1_CR2,                                   /*    USART CR2   */
  UART1_CR3                                    /*    USART CR3   */
};

static struct SerialInit uart1_init_struct = SERIAL_INIT_NULL;

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

static THD_WORKING_AREA(wa_thd_uart1_rx, UART_THREAD_STACK_SIZE);
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
static THD_WORKING_AREA(wa_thd_uart1_tx, UART_THREAD_STACK_SIZE);
#endif

void uart1_init(void)
{
  uart_periph_init(&uart1);

  // Only set pin if enabled and not statically defined in board file
#if USE_UART1_TX && defined UART1_GPIO_PORT_TX
  gpio_setup_pin_af(UART1_GPIO_PORT_TX, UART1_GPIO_TX, UART1_GPIO_AF, TRUE);
#endif
#if USE_UART1_RX && defined UART1_GPIO_PORT_RX
  gpio_setup_pin_af(UART1_GPIO_PORT_RX, UART1_GPIO_RX, UART1_GPIO_AF, FALSE);
#endif

  sdStart(&SD1, &usart1_config);
  uart1.reg_addr = &SD1;
  uart1.baudrate = UART1_BAUD;
  uart1.init_struct = &uart1_init_struct;
  uart1_init_struct.conf = &usart1_config;

  // Create threads
#if USE_UART1_RX
  uart1_init_struct.rx_mtx = &uart1_rx_mtx;
  uart1_init_struct.rx_sem = &uart1_rx_sem;
  chThdCreateStatic(wa_thd_uart1_rx, sizeof(wa_thd_uart1_rx),
                    NORMALPRIO + 1, thd_uart1_rx, NULL);
#endif
#if USE_UART1_TX
  uart1_init_struct.tx_mtx = &uart1_tx_mtx;
  uart1_init_struct.tx_sem = &uart1_tx_sem;
  chThdCreateStatic(wa_thd_uart1_tx, sizeof(wa_thd_uart1_tx),
                    NORMALPRIO + 1, thd_uart1_tx, NULL);
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


/* by default disable HW flow control */
#ifndef UART2_HW_FLOW_CONTROL
#define UART2_HW_FLOW_CONTROL FALSE
#endif

#if UART2_HW_FLOW_CONTROL && defined(UART2_CR3)
#warning "UART2_CR3 reset to your value, HW flow control not enabled! You may want to set USART_CR3_CTSE | USART_CR3_RTSE yourself."
#endif

#ifndef UART2_CR1
#define UART2_CR1 0
#endif

#ifndef UART2_CR2
#define UART2_CR2 USART_CR2_STOP1_BITS
#endif

#ifndef UART2_CR3
#if UART2_HW_FLOW_CONTROL
#define UART2_CR3 USART_CR3_CTSE | USART_CR3_RTSE
#else
#define UART2_CR3 0
#endif
#endif

static SerialConfig usart2_config = {
  UART2_BAUD,                         /*     BITRATE    */
  UART2_CR1,                          /*    USART CR1   */
  UART2_CR2,                          /*    USART CR2   */
  UART2_CR3                           /*    USART CR3   */
};

static struct SerialInit uart2_init_struct = SERIAL_INIT_NULL;

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

static THD_WORKING_AREA(wa_thd_uart2_rx, UART_THREAD_STACK_SIZE);
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
static THD_WORKING_AREA(wa_thd_uart2_tx, UART_THREAD_STACK_SIZE);
#endif

void uart2_init(void)
{
  uart_periph_init(&uart2);

  // Only set pin if enabled and not statically defined in board file
#if USE_UART2_TX && defined UART2_GPIO_PORT_TX
  gpio_setup_pin_af(UART2_GPIO_PORT_TX, UART2_GPIO_TX, UART2_GPIO_AF, TRUE);
#endif
#if USE_UART2_RX && defined UART2_GPIO_PORT_RX
  gpio_setup_pin_af(UART2_GPIO_PORT_RX, UART2_GPIO_RX, UART2_GPIO_AF, FALSE);
#endif

  sdStart(&SD2, &usart2_config);
  uart2.reg_addr = &SD2;
  uart2.baudrate = UART2_BAUD;
  uart2.init_struct = &uart2_init_struct;
  uart2_init_struct.conf = &usart2_config;

  // Create threads
#if USE_UART2_RX
  uart2_init_struct.rx_mtx = &uart2_rx_mtx;
  uart2_init_struct.rx_sem = &uart2_rx_sem;
  chThdCreateStatic(wa_thd_uart2_rx, sizeof(wa_thd_uart2_rx),
                    NORMALPRIO + 1, thd_uart2_rx, NULL);
#endif
#if USE_UART2_TX
  uart2_init_struct.tx_mtx = &uart2_tx_mtx;
  uart2_init_struct.tx_sem = &uart2_tx_sem;
  chThdCreateStatic(wa_thd_uart2_tx, sizeof(wa_thd_uart2_tx),
                    NORMALPRIO + 1, thd_uart2_tx, NULL);
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

#ifndef UART3_CR1
#define UART3_CR1 0
#endif

#ifndef UART3_CR2
#define UART3_CR2 USART_CR2_STOP1_BITS
#endif

#ifndef UART3_CR3
#define UART3_CR3 0
#endif

static SerialConfig usart3_config = {
  UART3_BAUD,                        /*     BITRATE    */
  UART3_CR1,                         /*    USART CR1   */
  UART3_CR2,                         /*    USART CR2   */
  UART3_CR3                          /*    USART CR3   */
};

static struct SerialInit uart3_init_struct = SERIAL_INIT_NULL;

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

static THD_WORKING_AREA(wa_thd_uart3_rx, UART_THREAD_STACK_SIZE);
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
static THD_WORKING_AREA(wa_thd_uart3_tx, UART_THREAD_STACK_SIZE);
#endif

void uart3_init(void)
{
  uart_periph_init(&uart3);

  // Only set pin if enabled and not statically defined in board file
#if USE_UART3_TX && defined UART3_GPIO_PORT_TX
  gpio_setup_pin_af(UART3_GPIO_PORT_TX, UART3_GPIO_TX, UART3_GPIO_AF, TRUE);
#endif
#if USE_UART3_RX && defined UART3_GPIO_PORT_RX
  gpio_setup_pin_af(UART3_GPIO_PORT_RX, UART3_GPIO_RX, UART3_GPIO_AF, FALSE);
#endif

  sdStart(&SD3, &usart3_config);
  uart3.reg_addr = &SD3;
  uart3.baudrate = UART3_BAUD;
  uart3.init_struct = &uart3_init_struct;
  uart3_init_struct.conf = &usart3_config;

  // Create threads
#if USE_UART3_RX
  uart3_init_struct.rx_mtx = &uart3_rx_mtx;
  uart3_init_struct.rx_sem = &uart3_rx_sem;
  chThdCreateStatic(wa_thd_uart3_rx, sizeof(wa_thd_uart3_rx),
                    NORMALPRIO + 1, thd_uart3_rx, NULL);
#endif
#if USE_UART3_TX
  uart3_init_struct.tx_mtx = &uart3_tx_mtx;
  uart3_init_struct.tx_sem = &uart3_tx_sem;
  chThdCreateStatic(wa_thd_uart3_tx, sizeof(wa_thd_uart3_tx),
                    NORMALPRIO + 1, thd_uart3_tx, NULL);
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

#ifndef UART4_CR1
#define UART4_CR1 0
#endif

#ifndef UART4_CR2
#define UART4_CR2 USART_CR2_STOP1_BITS
#endif

#ifndef UART4_CR3
#define UART4_CR3 0
#endif

static SerialConfig usart4_config = {
  UART4_BAUD,                        /*     BITRATE    */
  UART4_CR1,                         /*    USART CR1   */
  UART4_CR2,                         /*    USART CR2   */
  UART4_CR3                          /*    USART CR3   */
};

static struct SerialInit uart4_init_struct = SERIAL_INIT_NULL;

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

static THD_WORKING_AREA(wa_thd_uart4_rx, UART_THREAD_STACK_SIZE);
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
static THD_WORKING_AREA(wa_thd_uart4_tx, UART_THREAD_STACK_SIZE);
#endif

void uart4_init(void)
{
  uart_periph_init(&uart4);

  // Only set pin if enabled and not statically defined in board file
#if USE_UART4_TX && defined UART4_GPIO_PORT_TX
  gpio_setup_pin_af(UART4_GPIO_PORT_TX, UART4_GPIO_TX, UART4_GPIO_AF, TRUE);
#endif
#if USE_UART4_RX && defined UART4_GPIO_PORT_RX
  gpio_setup_pin_af(UART4_GPIO_PORT_RX, UART4_GPIO_RX, UART4_GPIO_AF, FALSE);
#endif

  sdStart(&SD4, &usart4_config);
  uart4.reg_addr = &SD4;
  uart4.baudrate = UART4_BAUD;
  uart4.init_struct = &uart4_init_struct;
  uart4_init_struct.conf = &usart4_config;

  // Create threads
#if USE_UART4_RX
  uart4_init_struct.rx_mtx = &uart4_rx_mtx;
  uart4_init_struct.rx_sem = &uart4_rx_sem;
  chThdCreateStatic(wa_thd_uart4_rx, sizeof(wa_thd_uart4_rx),
                    NORMALPRIO + 1, thd_uart4_rx, NULL);
#endif
#if USE_UART4_TX
  uart4_init_struct.tx_mtx = &uart4_tx_mtx;
  uart4_init_struct.tx_sem = &uart4_tx_sem;
  chThdCreateStatic(wa_thd_uart4_tx, sizeof(wa_thd_uart4_tx),
                    NORMALPRIO + 1, thd_uart4_tx, NULL);
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

#ifndef UART5_CR1
#define UART5_CR1 0
#endif

#ifndef UART5_CR2
#define UART5_CR2 USART_CR2_STOP1_BITS
#endif

#ifndef UART5_CR3
#define UART5_CR3 0
#endif

static SerialConfig usart5_config = {
  UART5_BAUD,                        /*     BITRATE    */
  UART5_CR1,                         /*    USART CR1   */
  UART5_CR2,                         /*    USART CR2   */
  UART5_CR3                          /*    USART CR3   */
};

static struct SerialInit uart5_init_struct = SERIAL_INIT_NULL;

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

static THD_WORKING_AREA(wa_thd_uart5_rx, UART_THREAD_STACK_SIZE);
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
static THD_WORKING_AREA(wa_thd_uart5_tx, UART_THREAD_STACK_SIZE);
#endif

void uart5_init(void)
{
  uart_periph_init(&uart5);

  // Only set pin if enabled and not statically defined in board file
#if USE_UART5_TX && defined UART5_GPIO_PORT_TX
  gpio_setup_pin_af(UART5_GPIO_PORT_TX, UART5_GPIO_TX, UART5_GPIO_AF, TRUE);
#endif
#if USE_UART5_RX && defined UART5_GPIO_PORT_RX
  gpio_setup_pin_af(UART5_GPIO_PORT_RX, UART5_GPIO_RX, UART5_GPIO_AF, FALSE);
#endif

  sdStart(&SD5, &usart5_config);
  uart5.reg_addr = &SD5;
  uart5.baudrate = UART5_BAUD;
  uart5.init_struct = &uart5_init_struct;
  uart5_init_struct.conf = &usart5_config;

  // Create threads
#if USE_UART5_RX
  uart5_init_struct.rx_mtx = &uart5_rx_mtx;
  uart5_init_struct.rx_sem = &uart5_rx_sem;
  chThdCreateStatic(wa_thd_uart5_rx, sizeof(wa_thd_uart5_rx),
                    NORMALPRIO + 1, thd_uart5_rx, NULL);
#endif
#if USE_UART5_TX
  uart5_init_struct.tx_mtx = &uart5_tx_mtx;
  uart5_init_struct.tx_sem = &uart5_tx_sem;
  chThdCreateStatic(wa_thd_uart5_tx, sizeof(wa_thd_uart5_tx),
                    NORMALPRIO + 1, thd_uart5_tx, NULL);
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

#ifndef UART6_CR1
#define UART6_CR1 0
#endif

#ifndef UART6_CR2
#define UART6_CR2 USART_CR2_STOP1_BITS
#endif

#ifndef UART6_CR3
#define UART6_CR3 0
#endif

static SerialConfig usart6_config = {
  UART6_BAUD,                        /*     BITRATE    */
  UART6_CR1,                         /*    USART CR1   */
  UART6_CR2,                         /*    USART CR2   */
  UART6_CR3                          /*    USART CR3   */
};

static struct SerialInit uart6_init_struct = SERIAL_INIT_NULL;

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

static THD_WORKING_AREA(wa_thd_uart6_rx, UART_THREAD_STACK_SIZE);
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
static THD_WORKING_AREA(wa_thd_uart6_tx, UART_THREAD_STACK_SIZE);
#endif

void uart6_init(void)
{
  uart_periph_init(&uart6);

  // Only set pin if enabled and not statically defined in board file
#if USE_UART6_TX && defined UART6_GPIO_PORT_TX
  gpio_setup_pin_af(UART6_GPIO_PORT_TX, UART6_GPIO_TX, UART6_GPIO_AF, TRUE);
#endif
#if USE_UART6_RX && defined UART6_GPIO_PORT_RX
  gpio_setup_pin_af(UART6_GPIO_PORT_RX, UART6_GPIO_RX, UART6_GPIO_AF, FALSE);
#endif

  sdStart(&SD6, &usart6_config);
  uart6.reg_addr = &SD6;
  uart6.baudrate = UART6_BAUD;
  uart6.init_struct = &uart6_init_struct;
  uart6_init_struct.conf = &usart6_config;

  // Create threads
#if USE_UART6_RX
  uart6_init_struct.rx_mtx = &uart6_rx_mtx;
  uart6_init_struct.rx_sem = &uart6_rx_sem;
  chThdCreateStatic(wa_thd_uart6_rx, sizeof(wa_thd_uart6_rx),
                    NORMALPRIO + 1, thd_uart6_rx, NULL);
#endif
#if USE_UART6_TX
  uart6_init_struct.tx_mtx = &uart6_tx_mtx;
  uart6_init_struct.tx_sem = &uart6_tx_sem;
  chThdCreateStatic(wa_thd_uart6_tx, sizeof(wa_thd_uart6_tx),
                    NORMALPRIO + 1, thd_uart6_tx, NULL);
#endif

#if defined UART6_GPIO_CTS && defined UART6_GPIO_PORT_CTS
  uart6_init_struct.cts_pin = UART6_GPIO_CTS;
  uart6_init_struct.cts_port = UART6_GPIO_PORT_CTS;
#endif
}

#endif

#if USE_UART7

#ifndef UART7_BAUD
#define UART7_BAUD SERIAL_DEFAULT_BITRATE
#endif

/* by default enable UART Tx and Rx */
#ifndef USE_UART7_TX
#define USE_UART7_TX TRUE
#endif
#ifndef USE_UART7_RX
#define USE_UART7_RX TRUE
#endif

#ifndef UART7_CR1
#define UART7_CR1 0
#endif

#ifndef UART7_CR2
#define UART7_CR2 USART_CR2_STOP1_BITS
#endif

#ifndef UART7_CR3
#define UART7_CR3 0
#endif

static SerialConfig usart7_config = {
  UART7_BAUD,                        /*     BITRATE    */
  UART7_CR1,                         /*    USART CR1   */
  UART7_CR2,                         /*    USART CR2   */
  UART7_CR3                          /*    USART CR3   */
};

static struct SerialInit uart7_init_struct = SERIAL_INIT_NULL;

// Threads RX and TX
#if USE_UART7_RX
static MUTEX_DECL(uart7_rx_mtx);
static SEMAPHORE_DECL(uart7_rx_sem, 0);

static __attribute__((noreturn)) void thd_uart7_rx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart7_rx");

  while (TRUE) {
    handle_uart_rx(&uart7);
  }
}

static THD_WORKING_AREA(wa_thd_uart7_rx, UART_THREAD_STACK_SIZE);
#endif

#if USE_UART7_TX
static MUTEX_DECL(uart7_tx_mtx);
static SEMAPHORE_DECL(uart7_tx_sem, 0);

static __attribute__((noreturn)) void thd_uart7_tx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart7_tx");

  while (TRUE) {
    handle_uart_tx(&uart7);
  }
}
static THD_WORKING_AREA(wa_thd_uart7_tx, UART_THREAD_STACK_SIZE);
#endif

void uart7_init(void)
{
  uart_periph_init(&uart7);

  // Only set pin if enabled and not statically defined in board file
#if USE_UART7_TX && defined UART7_GPIO_PORT_TX
  gpio_setup_pin_af(UART7_GPIO_PORT_TX, UART7_GPIO_TX, UART7_GPIO_AF, TRUE);
#endif
#if USE_UART7_RX && defined UART7_GPIO_PORT_RX
  gpio_setup_pin_af(UART7_GPIO_PORT_RX, UART7_GPIO_RX, UART7_GPIO_AF, FALSE);
#endif

  sdStart(&SD7, &usart7_config);
  uart7.reg_addr = &SD7;
  uart7.baudrate = UART7_BAUD;
  uart7.init_struct = &uart7_init_struct;
  uart7_init_struct.conf = &usart7_config;

  // Create threads
#if USE_UART7_RX
  uart7_init_struct.rx_mtx = &uart7_rx_mtx;
  uart7_init_struct.rx_sem = &uart7_rx_sem;
  chThdCreateStatic(wa_thd_uart7_rx, sizeof(wa_thd_uart7_rx),
                    NORMALPRIO + 1, thd_uart7_rx, NULL);
#endif
#if USE_UART7_TX
  uart7_init_struct.tx_mtx = &uart7_tx_mtx;
  uart7_init_struct.tx_sem = &uart7_tx_sem;
  chThdCreateStatic(wa_thd_uart7_tx, sizeof(wa_thd_uart7_tx),
                    NORMALPRIO + 1, thd_uart7_tx, NULL);
#endif
}

#endif

#if USE_UART8

#ifndef UART8_BAUD
#define UART8_BAUD SERIAL_DEFAULT_BITRATE
#endif

/* by default enable UART Tx and Rx */
#ifndef USE_UART8_TX
#define USE_UART8_TX TRUE
#endif
#ifndef USE_UART8_RX
#define USE_UART8_RX TRUE
#endif

#ifndef UART8_CR1
#define UART8_CR1 0
#endif

#ifndef UART8_CR2
#define UART8_CR2 USART_CR2_STOP1_BITS
#endif

#ifndef UART8_CR3
#define UART8_CR3 0
#endif

static SerialConfig usart8_config = {
  UART8_BAUD,                        /*     BITRATE    */
  UART8_CR1,                         /*    USART CR1   */
  UART8_CR2,                         /*    USART CR2   */
  UART8_CR3                          /*    USART CR3   */
};

static struct SerialInit uart8_init_struct = SERIAL_INIT_NULL;

// Threads RX and TX
#if USE_UART8_RX
static MUTEX_DECL(uart8_rx_mtx);
static SEMAPHORE_DECL(uart8_rx_sem, 0);

static __attribute__((noreturn)) void thd_uart8_rx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart8_rx");

  while (TRUE) {
    handle_uart_rx(&uart8);
  }
}

static THD_WORKING_AREA(wa_thd_uart8_rx, UART_THREAD_STACK_SIZE);
#endif

#if USE_UART8_TX
static MUTEX_DECL(uart8_tx_mtx);
static SEMAPHORE_DECL(uart8_tx_sem, 0);

static __attribute__((noreturn)) void thd_uart8_tx(void *arg)
{
  (void) arg;
  chRegSetThreadName("uart8_tx");

  while (TRUE) {
    handle_uart_tx(&uart8);
  }
}
static THD_WORKING_AREA(wa_thd_uart8_tx, UART_THREAD_STACK_SIZE);
#endif

void uart8_init(void)
{
  uart_periph_init(&uart8);

  // Only set pin if enabled and not statically defined in board file
#if USE_UART8_TX && defined UART8_GPIO_PORT_TX
  gpio_setup_pin_af(UART8_GPIO_PORT_TX, UART8_GPIO_TX, UART8_GPIO_AF, TRUE);
#endif
#if USE_UART8_RX && defined UART8_GPIO_PORT_RX
  gpio_setup_pin_af(UART8_GPIO_PORT_RX, UART8_GPIO_RX, UART8_GPIO_AF, FALSE);
#endif

  sdStart(&SD8, &usart8_config);
  uart8.reg_addr = &SD8;
  uart8.baudrate = UART8_BAUD;
  uart8.init_struct = &uart8_init_struct;
  uart8_init_struct.conf = &usart8_config;

  // Create threads
#if USE_UART8_RX
  uart8_init_struct.rx_mtx = &uart8_rx_mtx;
  uart8_init_struct.rx_sem = &uart8_rx_sem;
  chThdCreateStatic(wa_thd_uart8_rx, sizeof(wa_thd_uart8_rx),
                    NORMALPRIO + 1, thd_uart8_rx, NULL);
#endif
#if USE_UART8_TX
  uart8_init_struct.tx_mtx = &uart8_tx_mtx;
  uart8_init_struct.tx_sem = &uart8_tx_sem;
  chThdCreateStatic(wa_thd_uart8_tx, sizeof(wa_thd_uart8_tx),
                    NORMALPRIO + 1, thd_uart8_tx, NULL);
#endif
}

#endif


uint8_t uart_getch(struct uart_periph *p)
{
  //to keep compatibility with loop oriented paparazzi architecture, read is not blocking
  //struct SerialInit *init_struct = (struct SerialInit*)(p->init_struct);
  //chSemWait (init_struct->rx_sem);
  struct SerialInit *init_struct = (struct SerialInit *)(p->init_struct);
  chMtxLock(init_struct->rx_mtx);
  uint8_t ret = p->rx_buf[p->rx_extract_idx];
  p->rx_extract_idx = (p->rx_extract_idx + 1) % UART_RX_BUFFER_SIZE;
  chMtxUnlock(init_struct->rx_mtx);
  return ret;
}

/**
 * Set baudrate
 */
void uart_periph_set_baudrate(struct uart_periph *p, uint32_t baud)
{
  struct SerialInit *init_struct = (struct SerialInit *)(p->init_struct);
  SerialConfig *conf = init_struct->conf;
  // set new baudrate
  conf->speed = baud;
  p->baudrate = baud;
  // restart periph
  sdStop((SerialDriver *)(p->reg_addr));
  sdStart((SerialDriver *)(p->reg_addr), conf);
}

/**
 * Set mode (not necessary, or can be set by SerialConfig)
 */
void uart_periph_set_mode(struct uart_periph *p __attribute__((unused)), bool tx_enabled __attribute__((unused)),
                          bool rx_enabled __attribute__((unused)), bool hw_flow_control __attribute__((unused))) {}

#if defined STM32F7
#define __USART_CR1_M USART_CR1_M_0
#elif defined STM32F1 || defined STM32F4 || defined STM32F3
#define __USART_CR1_M USART_CR1_M
#else
#error unsupported board
#endif

/**
 * Set parity and stop bits
 */
void uart_periph_set_bits_stop_parity(struct uart_periph *p,
                                      uint8_t bits, uint8_t stop, uint8_t parity)
{
  struct SerialInit *init_struct = (struct SerialInit *)(p->init_struct);
  SerialConfig *conf = init_struct->conf;

  /* Configure USART parity and data bits */
  if (parity == UPARITY_EVEN) {
    conf->cr1 |= USART_CR1_PCE; // set parity control bit
    conf->cr1 &= ~USART_CR1_PS; // clear parity selection bit
    if (bits == UBITS_7) {
      conf->cr1 &= ~__USART_CR1_M; // clear word length bit
    } else { // 8 data bits by default
      conf->cr1 |= __USART_CR1_M; // set word length bit
    }
  } else if (parity == UPARITY_ODD) {
    conf->cr1 |= USART_CR1_PCE; // set parity control bit
    conf->cr1 |= USART_CR1_PS; // set parity selection bit
    if (bits == UBITS_7) {
      conf->cr1 &= ~__USART_CR1_M; // clear word length bit
    } else { // 8 data bits by default
      conf->cr1 |= __USART_CR1_M; // set word length bit
    }
  } else { // 8 data bist, NO_PARITY by default
    conf->cr1 &= ~USART_CR1_PCE; // clear parity control bit
    conf->cr1 &= ~__USART_CR1_M; // clear word length bit
  }
  /* Configure USART stop bits */
  conf->cr2 &= ~USART_CR2_STOP; // clear stop bits
  if (stop == USTOP_2) {
    conf-> cr2 |= USART_CR2_STOP2_BITS; // set bits for 2 stops
  } else { // 1 stop bit by default
    conf-> cr2 |= USART_CR2_STOP1_BITS; // set bits for 1 stop
  }

  sdStop((SerialDriver *)(p->reg_addr));
  sdStart((SerialDriver *)(p->reg_addr), conf);
}

#ifdef STM32F7
/**
 * Invert data logic
 */
void uart_periph_invert_data_logic(struct uart_periph *p, bool invert_rx, bool invert_tx)
{
  struct SerialInit *init_struct = (struct SerialInit *)(p->init_struct);
  SerialConfig *conf = init_struct->conf;
  if (invert_rx) {
    conf->cr2 |= USART_CR2_RXINV; // set rxinv bit
  } else {
    conf->cr2 &= ~USART_CR2_RXINV; // clear rxinv bit
  }
  if (invert_tx) {
    conf->cr2 |= USART_CR2_TXINV; // set txinv bit
  } else {
    conf->cr2 &= ~USART_CR2_TXINV; // clear txinv bit
  }
  sdStop((SerialDriver *)(p->reg_addr));
  sdStart((SerialDriver *)(p->reg_addr), conf);
}
#endif

// Check free space and set a positive value for fd if valid
// and lock driver with mutex
int uart_check_free_space(struct uart_periph *p, long *fd, uint16_t len)
{
  struct SerialInit *init_struct = (struct SerialInit *)(p->init_struct);
  int space = p->tx_extract_idx - p->tx_insert_idx - 1;
  if (space < 0) {
    space += UART_TX_BUFFER_SIZE;
  }
  if (space >= len) {
    *fd = 1;
    chMtxLock(init_struct->tx_mtx);
    return space;
  }
  return 0;
}

/**
* Uart transmit implementation
*/
void uart_put_byte(struct uart_periph *p, long fd, uint8_t data)
{
  struct SerialInit *init_struct = (struct SerialInit *)(p->init_struct);
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
    chSemSignal(init_struct->tx_sem);
  } else {
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
  struct SerialInit *init_struct = (struct SerialInit *)(p->init_struct);
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
    chSemSignal(init_struct->tx_sem);
  }
}

void uart_send_message(struct uart_periph *p, long fd)
{
  struct SerialInit *init_struct = (struct SerialInit *)(p->init_struct);
  // unlock driver in case it is not done (fd > 0)
  if (fd != 0) {
    chMtxUnlock(init_struct->tx_mtx);
  }
  // send signal to start transmission
  chSemSignal(init_struct->tx_sem);
}

