/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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

#if USE_UART1
#ifndef UART1_BAUD
#define UART1_BAUD SERIAL_DEFAULT_BITRATE
#endif
static const SerialConfig usart1_config = {
  UART1_BAUD,                                             /*     BITRATE    */
  0,                                                      /*    USART CR1   */
  USART_CR2_STOP1_BITS,                                   /*    USART CR2   */
  0                                                       /*    USART CR3   */
};
void uart1_init(void)
{
  uart_periph_init(&uart1);
  sdStart(&SD1, &usart1_config);
  uart1.reg_addr = &SD1;
}
#endif


#if USE_UART2
#ifndef UART2_BAUD
#define UART2_BAUD SERIAL_DEFAULT_BITRATE
#endif
static const SerialConfig usart2_config = {
  UART2_BAUD,                                               /*     BITRATE    */
  0,                                                        /*    USART CR1   */
  USART_CR2_STOP1_BITS,                                     /*    USART CR2   */
  0                                                         /*    USART CR3   */
};
void uart2_init(void)
{
  uart_periph_init(&uart2);
  sdStart(&SD2, &usart2_config);
  uart2.reg_addr = &SD2;
}
#endif

#if USE_UART3
#ifndef UART3_BAUD
#define UART3_BAUD SERIAL_DEFAULT_BITRATE
#endif
static const SerialConfig usart3_config = {
  UART3_BAUD,                                             /*     BITRATE    */
  0,                                                      /*    USART CR1   */
  USART_CR2_STOP1_BITS,                                   /*    USART CR2   */
  0                                                       /*    USART CR3   */
};
void uart3_init(void)
{
  uart_periph_init(&uart3);
  sdStart(&SD3, &usart3_config);
  uart3.reg_addr = &SD3;
}
#endif

#if USE_UART4
#ifndef UART4_BAUD
#define UART4_BAUD SERIAL_DEFAULT_BITRATE
#endif
static const SerialConfig usart4_config = {
  UART4_BAUD,                                             /*     BITRATE    */
  0,                                                      /*    USART CR1   */
  USART_CR2_STOP1_BITS,                                   /*    USART CR2   */
  0                                                       /*    USART CR3   */
};
void uart4_init(void)
{
  uart_periph_init(&uart4);
  sdStart(&SD4, &usart4_config);
  uart4.reg_addr = &SD4;
}
#endif

#if USE_UART5
#ifndef UART5_BAUD
#define UART5_BAUD SERIAL_DEFAULT_BITRATE
#endif
static const SerialConfig usart5_config = {
  UART5_BAUD,                                             /*     BITRATE    */
  0,                                                      /*    USART CR1   */
  USART_CR2_STOP1_BITS,                                   /*    USART CR2   */
  0                                                       /*    USART CR3   */
};
void uart5_init(void)
{
  uart_periph_init(&uart5);
  sdStart(&SD5, &usart5_config);
  uart5.reg_addr = &SD5;
}
#endif

/**
 * Set stop bits and parity
 * @note Not necessary, use SerialConfig
 * @param p
 * @param bits
 * @param stop
 * @param parity
 */
void uart_periph_set_bits_stop_parity(struct uart_periph* p __attribute__((unused)),
                                      uint8_t bits __attribute__((unused)),
                                      uint8_t stop __attribute__((unused)),
                                      uint8_t parity __attribute__((unused))){}



/**
 * Get byte from serial port
 * @param p
 * @return
 */
uint8_t uart_getch(struct uart_periph *p) {
  return sdGet((SerialDriver *)p->reg_addr);
}

/**
 * Set baudrate (from the serialConfig)
 * @note Baudrate is set in sdStart, no need for implementation
 */
void uart_periph_set_baudrate(struct uart_periph *p __attribute__((unused)), uint32_t baud __attribute__((unused))) {}

/**
 * Set mode (not necessary, or can be set by SerialConfig)
 */
void uart_periph_set_mode(struct uart_periph *p __attribute__((unused)), bool_t tx_enabled __attribute__((unused)),
                          bool_t rx_enabled __attribute__((unused)), bool_t hw_flow_control __attribute__((unused))) {}

/**
* Uart transmit implementation
*/
void uart_put_byte(struct uart_periph *p, uint8_t data)
{
  sdPut((SerialDriver *)p->reg_addr, data);
}

/**
* Uart/SerialDriver transmit buffer implementation
*
* Typical use:
* uint8_t tx_switch[10] = { 0x01, 0x08, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, '\r' };
* uart_transmit_buffer(&uart2, tx_switch, sizeof(tx_switch));
*/
void uart_transmit_buffer(struct uart_periph *p, uint8_t *data_buffer, uint16_t length)
{
  sdWrite((SerialDriver *)p->reg_addr, data_buffer, (size_t)length);
}

/**
 * Uart/SerialDriver receive loop implementation
 *
 * @param[in] p pointer to a @p uart_periph object
 * @param[in] flags flagmask for SD event flags
 * @param[in] on_receive_callback pointer to a callback function
 */
void uart_receive_buffer(struct uart_periph *p, eventflags_t flags, void *on_receive_callback)
{
  if ((flags & (SD_FRAMING_ERROR | SD_OVERRUN_ERROR | SD_NOISE_ERROR)) != 0) {
    if (flags & SD_OVERRUN_ERROR) {
      p->ore++;
    }
    if (flags & SD_NOISE_ERROR) {
      p->ne_err++;
    }
    if (flags & SD_FRAMING_ERROR) {
      p->fe_err++;
    }
  }

  if (flags & CHN_INPUT_AVAILABLE) {
    msg_t charbuf;
    do {
      charbuf = sdGetTimeout((SerialDriver *)p->reg_addr, TIME_IMMEDIATE);
      if (charbuf != Q_TIMEOUT) {
        if (on_receive_callback != NULL) {
          ((void( *)(uint8_t))on_receive_callback)((uint8_t) charbuf);
        }
      }
    } while (charbuf != Q_TIMEOUT);
  }

}
