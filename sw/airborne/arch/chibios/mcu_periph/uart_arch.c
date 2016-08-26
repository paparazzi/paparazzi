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
 * @brief chibios arch dependant implementation of uart/serial drivers
 * @details Partially implemented (no error callbacks yet). Since ChibiOs
 * 			has SerialDriver, it is better to use that one instead of Uart
 * 			drivers, although it makes backward compatibility more difficutl.
 * 			DMA is already implemented.
 * 	@note   The peripheral settings should use config (baudrate etc) from
 * 			makefiles, some makro should be used.
 */
#include "mcu_periph/uart.h"
#include "hal.h"


#ifdef USE_UART1
static const SerialConfig usart1_config =
{
57600,                                         		  	  /*     BITRATE    */
0,                                                        /*    USART CR1   */
USART_CR2_STOP1_BITS | USART_CR2_LINEN,                   /*    USART CR2   */
0                                                         /*    USART CR3   */
};
void uart1_init(void) {
  sdStart(&SD1, &usart1_config);
  uart1.reg_addr = &SD1;
}
#endif


#ifdef USE_UART2
static const SerialConfig usart2_config =
{
57600,                                         		  	  /*     BITRATE    */
0,                                                        /*    USART CR1   */
USART_CR2_STOP1_BITS | USART_CR2_LINEN,                   /*    USART CR2   */
0                                                         /*    USART CR3   */
};
void uart2_init(void) {
  sdStart(&SD2, &usart2_config);
  uart2.reg_addr = &SD2;
}
#endif

#ifdef USE_UART3
static const SerialConfig usart3_config =
{
57600,                                         		  	  /*     BITRATE    */
0,                                                        /*    USART CR1   */
USART_CR2_STOP1_BITS | USART_CR2_LINEN,                   /*    USART CR2   */
0                                                         /*    USART CR3   */
};
void uart3_init(void) {
  sdStart(&SD3, &usart3_config);
  uart3.reg_addr = &SD3;
}
#endif

#ifdef USE_UART4
static const SerialConfig usart4_config =
{
57600,                                         		  	  /*     BITRATE    */
0,                                                        /*    USART CR1   */
USART_CR2_STOP1_BITS | USART_CR2_LINEN,                   /*    USART CR2   */
0                                                         /*    USART CR3   */
};
void uart4_init(void) {
  sdStart(&SD4, &usart4_config);
  uart4.reg_addr = &SD4;
}
#endif

#ifdef USE_UART5
static const SerialConfig usart5_config =
{
115200,                                         		  /*     BITRATE    */
0,                                                        /*    USART CR1   */
USART_CR2_STOP1_BITS | USART_CR2_LINEN,                   /*    USART CR2   */
0                                                         /*    USART CR3   */
};
void uart5_init(void) {
  sdStart(&SD5, &usart5_config);
  uart5.reg_addr = &SD5;
}
#endif

/*
 * Set baudrate (from the serialConfig)
 * @note Baudrate is set in sdStart, no need for implementation
 */
void uart_periph_set_baudrate(struct uart_periph* p, uint32_t baud) {
  (void*) p;
  (void) baud;
}

/*
 * Set mode (not necessary, or can be set by SerialConfig)
 */
void uart_periph_set_mode(struct uart_periph* p, bool_t tx_enabled, bool_t rx_enabled, bool_t hw_flow_control) {
  (void*) p;
  (void) tx_enabled;
  (void) rx_enabled;
  (void) hw_flow_control;
}

/*
* Uart transmit implementation
*/
void uart_transmit(struct uart_periph* p, uint8_t data ) {
  sdWrite((SerialDriver*)p->reg_addr, &data, sizeof(data));
}

/*
* Uart transmit buffer implementation
* Typical use:
* uint8_t tx_switch[10] = { 0x01, 0x08, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, '\r' };
* uart_transmit_buffer(&uart2, tx_switch, sizeof(tx_switch));
*/
void uart_transmit_buffer(struct uart_periph* p, uint8_t* data_buffer, size_t length) {
  sdWrite((SerialDriver*)p->reg_addr, data_buffer, length);
}
