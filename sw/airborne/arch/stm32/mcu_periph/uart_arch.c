/*
 * Paparazzi $Id$
 *
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

#include <stm32/rcc.h>
#include <stm32/misc.h>
#include <stm32/usart.h>
#include <stm32/gpio.h>
#include "std.h"
#include "pprz_baudrate.h"

void uart_periph_set_baudrate(struct uart_periph* p, uint32_t baud) {

  /* Configure USART */
  USART_InitTypeDef usart;
  usart.USART_BaudRate            = baud;
  usart.USART_WordLength          = USART_WordLength_8b;
  usart.USART_StopBits            = USART_StopBits_1;
  usart.USART_Parity              = USART_Parity_No;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(p->reg_addr, &usart);
  /* Enable USART1 Receive interrupts */
  USART_ITConfig(p->reg_addr, USART_IT_RXNE, ENABLE);

  pprz_usart_set_baudrate(p->reg_addr, baud);

  /* Enable the USART */
  USART_Cmd(p->reg_addr, ENABLE);

}
// TODO set_mode function

void uart_transmit(struct uart_periph* p, uint8_t data ) {

  uint16_t temp = (p->tx_insert_idx + 1) % UART_TX_BUFFER_SIZE;

  if (temp == p->tx_extract_idx)
    return;                          // no room

  USART_ITConfig(p->reg_addr, USART_IT_TXE, DISABLE);

  // check if in process of sending data
  if (p->tx_running) { // yes, add to queue
    p->tx_buf[p->tx_insert_idx] = data;
    p->tx_insert_idx = temp;
  }
  else { // no, set running flag and write to output register
    p->tx_running = TRUE;
    USART_SendData(p->reg_addr, data);
  }

  USART_ITConfig(p->reg_addr, USART_IT_TXE, ENABLE);

}

static inline void usart_irq_handler(struct uart_periph* p) {

  if(USART_GetITStatus(p->reg_addr, USART_IT_TXE) != RESET){
    // check if more data to send
    if (p->tx_insert_idx != p->tx_extract_idx) {
      USART_SendData(p->reg_addr,p->tx_buf[p->tx_extract_idx]);
      p->tx_extract_idx++;
      p->tx_extract_idx %= UART_TX_BUFFER_SIZE;
    }
    else {
      p->tx_running = FALSE;   // clear running flag
      USART_ITConfig(p->reg_addr, USART_IT_TXE, DISABLE);
    }
  }

  if(USART_GetITStatus(p->reg_addr, USART_IT_RXNE) != RESET){
    uint16_t temp = (p->rx_insert_idx + 1) % UART_RX_BUFFER_SIZE;;
    p->rx_buf[p->rx_insert_idx] = USART_ReceiveData(p->reg_addr);
    // check for more room in queue
    if (temp != p->rx_extract_idx)
      p->rx_insert_idx = temp; // update insert index
  }

}

static inline void usart_enable_irq(IRQn_Type IRQn) {
  /* Enable USART interrupts */
  NVIC_InitTypeDef nvic;
  nvic.NVIC_IRQChannel = IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 2;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
}

#ifdef USE_UART1

void uart1_init( void ) {

  uart_periph_init(&uart1);
  uart1.reg_addr = USART1;

  /* init RCC */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB2PeriphClockCmd(UART1_Periph, ENABLE);

  /* Enable USART1 interrupts */
  usart_enable_irq(USART1_IRQn);

  /* Init GPIOS */
  GPIO_InitTypeDef gpio;
  /* GPIOA: GPIO_Pin_9 USART1 Tx push-pull */
  gpio.GPIO_Pin   = UART1_TxPin;
  gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(UART1_TxPort, &gpio);
  /* GPIOA: GPIO_Pin_10 USART1 Rx pin as floating input */
  gpio.GPIO_Pin   = UART1_RxPin;
  gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(UART1_RxPort, &gpio);

  /* Configure USART1 */
  uart_periph_set_baudrate(&uart1, UART1_BAUD);
}

void usart1_irq_handler(void) { usart_irq_handler(&uart1); }

#endif /* USE_UART1 */

#ifdef USE_UART2

void uart2_init( void ) {

  uart_periph_init(&uart2);
  uart2.reg_addr = USART2;

  /* init RCC */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(UART2_Periph, ENABLE);

  /* Enable USART2 interrupts */
  usart_enable_irq(USART2_IRQn);

  /* Init GPIOS */
  GPIO_InitTypeDef gpio;
  /* GPIOA: GPIO_Pin_2 USART2 Tx push-pull */
  gpio.GPIO_Pin   = UART2_TxPin; // ;
  gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(UART2_TxPort, &gpio);
  /* GPIOA: GPIO_Pin_3 USART2 Rx pin as floating input */
  gpio.GPIO_Pin   = UART2_RxPin; // ;
  gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(UART2_RxPort, &gpio);

  /* Configure USART2 */
  uart_periph_set_baudrate(&uart2, UART2_BAUD);
}

void usart2_irq_handler(void) { usart_irq_handler(&uart2); }

#endif /* USE_UART2 */

#ifdef USE_UART3

void uart3_init( void ) {

  uart_periph_init(&uart3);
  uart3.reg_addr = USART3;

  /* init RCC */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(UART3_Periph, ENABLE);

  /* Enable USART3 interrupts */
  usart_enable_irq(USART3_IRQn);

  /* Init GPIOS */
  GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
  GPIO_InitTypeDef gpio;
  /* GPIOC: GPIO_Pin_10 USART3 Tx push-pull */
  gpio.GPIO_Pin   = UART3_TxPin;
  gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(UART3_TxPort, &gpio);
  /* GPIOC: GPIO_Pin_11 USART3 Rx pin as floating input */
  gpio.GPIO_Pin   = UART3_RxPin;
  gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(UART3_RxPort, &gpio);

  /* Configure USART3 */
  uart_periph_set_baudrate(&uart3, UART3_BAUD);
}

void usart3_irq_handler(void) { usart_irq_handler(&uart3); }

#endif /* USE_UART3 */

#ifdef USE_UART5

void uart5_init( void ) {

  uart_periph_init(&uart5);
  uart5.reg_addr = USART5;

  /* init RCC */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
  RCC_APB2PeriphClockCmd(UART5_PeriphTx, ENABLE);
  RCC_APB2PeriphClockCmd(UART5_PeriphRx, ENABLE);

  /* Enable UART5 interrupts */
  usart_enable_irq(USART5_IRQn);

  /* Init GPIOS */
  GPIO_InitTypeDef gpio;
  /* GPIOC: GPIO_Pin_10 UART5 Tx push-pull */
  gpio.GPIO_Pin   = UART5_TxPin;
  gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(UART5_TxPort, &gpio);
  /* GPIOC: GPIO_Pin_11 UART5 Rx pin as floating input */
  gpio.GPIO_Pin   = UART5_RxPin;
  gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(UART5_RxPort, &gpio);

  /* Configure UART5 */
  uart_periph_set_baudrate(&uart5, UART5_BAUD);
}

void usart5_irq_handler(void) { usart_irq_handler(&uart5); }

#endif /* USE_UART5 */

