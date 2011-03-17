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

#include BOARD_CONFIG

#ifdef USE_OPENCM3
void usart_set_baudrate(uint32_t usart, uint32_t baud);
#define pprz_usart_set_baudrate(x, y) usart_set_baudrate(x, y)
#else
#define pprz_usart_set_baudrate(x, y) do { } while(0);
#endif

#ifdef USE_UART1

volatile uint16_t uart1_rx_insert_idx, uart1_rx_extract_idx;
uint8_t  uart1_rx_buffer[UART1_RX_BUFFER_SIZE];

volatile uint16_t uart1_tx_insert_idx, uart1_tx_extract_idx;
volatile bool_t uart1_tx_running;
uint8_t  uart1_tx_buffer[UART1_TX_BUFFER_SIZE];


void uart1_init( void ) {
  /* init RCC */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB2PeriphClockCmd(UART1_Periph, ENABLE);

  /* Enable USART1 interrupts */
  NVIC_InitTypeDef nvic;
  nvic.NVIC_IRQChannel = USART1_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 2;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

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
  USART_InitTypeDef usart;
  usart.USART_BaudRate            = UART1_BAUD;
  usart.USART_WordLength          = USART_WordLength_8b;
  usart.USART_StopBits            = USART_StopBits_1;
  usart.USART_Parity              = USART_Parity_No;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &usart);
  /* Enable USART1 Receive interrupts */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  pprz_usart_set_baudrate(USART1, UART1_BAUD);

  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);

  // initialize the transmit data queue
  uart1_tx_extract_idx = 0;
  uart1_tx_insert_idx = 0;
  uart1_tx_running = FALSE;

  // initialize the receive data queue
  uart1_rx_extract_idx = 0;
  uart1_rx_insert_idx = 0;

}

void uart1_transmit( uint8_t data ) {

  uint16_t temp = (uart1_tx_insert_idx + 1) % UART1_TX_BUFFER_SIZE;

  if (temp == uart1_tx_extract_idx)
    return;                          // no room

  USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

  // check if in process of sending data
  if (uart1_tx_running) { // yes, add to queue
    uart1_tx_buffer[uart1_tx_insert_idx] = data;
    uart1_tx_insert_idx = temp;
  }
  else { // no, set running flag and write to output register
    uart1_tx_running = TRUE;
    USART_SendData(USART1, data);
  }

  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

}

bool_t uart1_check_free_space( uint8_t len) {
  int16_t space = uart1_tx_extract_idx - uart1_tx_insert_idx;
  if (space <= 0)
    space += UART1_TX_BUFFER_SIZE;
  return (uint16_t)(space - 1) >= len;
}

void usart1_irq_handler(void) {

  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET){
    // check if more data to send
    if (uart1_tx_insert_idx != uart1_tx_extract_idx) {
      USART_SendData(USART1,uart1_tx_buffer[uart1_tx_extract_idx]);
      uart1_tx_extract_idx++;
      uart1_tx_extract_idx %= UART1_TX_BUFFER_SIZE;
    }
    else {
      uart1_tx_running = FALSE;   // clear running flag
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }
  }

  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
    uint16_t temp = (uart1_rx_insert_idx + 1) % UART1_RX_BUFFER_SIZE;;
    uart1_rx_buffer[uart1_rx_insert_idx] = USART_ReceiveData(USART1);
    // check for more room in queue
    if (temp != uart1_rx_extract_idx)
      uart1_rx_insert_idx = temp; // update insert index
  }

}


#endif /* USE_UART1 */







#ifdef USE_UART2

volatile uint16_t uart2_rx_insert_idx, uart2_rx_extract_idx;
uint8_t  uart2_rx_buffer[UART2_RX_BUFFER_SIZE];

volatile uint16_t uart2_tx_insert_idx, uart2_tx_extract_idx;
volatile bool_t uart2_tx_running;
uint8_t  uart2_tx_buffer[UART2_TX_BUFFER_SIZE];


void uart2_init( void ) {
  /* init RCC */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(UART2_Periph, ENABLE);

  /* Enable USART2 interrupts */
  NVIC_InitTypeDef nvic;
  nvic.NVIC_IRQChannel = USART2_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 2;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

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
  USART_InitTypeDef usart;
  usart.USART_BaudRate            = UART2_BAUD;
  usart.USART_WordLength          = USART_WordLength_8b;
  usart.USART_StopBits            = USART_StopBits_1;
  usart.USART_Parity              = USART_Parity_No;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &usart);
  /* Enable USART2 Receive interrupts */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  pprz_usart_set_baudrate(USART2, UART2_BAUD);

  /* Enable the USART2 */
  USART_Cmd(USART2, ENABLE);

  // initialize the transmit data queue
  uart2_tx_extract_idx = 0;
  uart2_tx_insert_idx = 0;
  uart2_tx_running = FALSE;

  // initialize the receive data queue
  uart2_rx_extract_idx = 0;
  uart2_rx_insert_idx = 0;

}

void uart2_transmit( uint8_t data ) {

  uint16_t temp = (uart2_tx_insert_idx + 1) % UART2_TX_BUFFER_SIZE;

  if (temp == uart2_tx_extract_idx)
    return;                          // no room

  USART_ITConfig(USART2, USART_IT_TXE, DISABLE);

  // check if in process of sending data
  if (uart2_tx_running) { // yes, add to queue
    uart2_tx_buffer[uart2_tx_insert_idx] = data;
    uart2_tx_insert_idx = temp;
  }
  else { // no, set running flag and write to output register
    uart2_tx_running = TRUE;
    USART_SendData(USART2, data);
  }

  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

}

bool_t uart2_check_free_space( uint8_t len) {
  int16_t space = uart2_tx_extract_idx - uart2_tx_insert_idx;
  if (space <= 0)
    space += UART2_TX_BUFFER_SIZE;
  return (uint16_t)(space - 1) >= len;
}

void usart2_irq_handler(void) {
  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET){
    // check if more data to send
    if (uart2_tx_insert_idx != uart2_tx_extract_idx) {
      USART_SendData(USART2,uart2_tx_buffer[uart2_tx_extract_idx]);
      uart2_tx_extract_idx++;
      uart2_tx_extract_idx %= UART2_TX_BUFFER_SIZE;
    }
    else {
      uart2_tx_running = FALSE;   // clear running flag
      USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }
  }

  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
    uint16_t temp = (uart2_rx_insert_idx + 1) % UART2_RX_BUFFER_SIZE;;
    uart2_rx_buffer[uart2_rx_insert_idx] = USART_ReceiveData(USART2);
    // check for more room in queue
    if (temp != uart2_rx_extract_idx)
      uart2_rx_insert_idx = temp; // update insert index
  }

}


#endif /* USE_UART2 */






#ifdef USE_UART3

volatile uint16_t uart3_rx_insert_idx, uart3_rx_extract_idx;
uint8_t  uart3_rx_buffer[UART3_RX_BUFFER_SIZE];

volatile uint16_t uart3_tx_insert_idx, uart3_tx_extract_idx;
volatile bool_t uart3_tx_running;
uint8_t  uart3_tx_buffer[UART3_TX_BUFFER_SIZE];

void uart3_init( void ) {

  /* init RCC */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(UART3_Periph, ENABLE);

  /* Enable USART3 interrupts */
  NVIC_InitTypeDef nvic;
  nvic.NVIC_IRQChannel = USART3_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 2;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

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
  USART_InitTypeDef usart;
  usart.USART_BaudRate            = UART3_BAUD;
  usart.USART_WordLength          = USART_WordLength_8b;
  usart.USART_StopBits            = USART_StopBits_1;
  usart.USART_Parity              = USART_Parity_No;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &usart);
  /* Enable USART3 Receive interrupts */
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  pprz_usart_set_baudrate(USART3, UART3_BAUD);

  /* Enable the USART3 */
  USART_Cmd(USART3, ENABLE);

  // initialize the transmit data queue
  uart3_tx_extract_idx = 0;
  uart3_tx_insert_idx = 0;
  uart3_tx_running = FALSE;

  // initialize the receive data queuenn
  uart3_rx_extract_idx = 0;
  uart3_rx_insert_idx = 0;

}

void uart3_transmit( uint8_t data ) {

  uint16_t temp = (uart3_tx_insert_idx + 1) % UART3_TX_BUFFER_SIZE;

  if (temp == uart3_tx_extract_idx)
    return;                          // no room

  USART_ITConfig(USART3, USART_IT_TXE, DISABLE);

  // check if in process of sending data
  if (uart3_tx_running) { // yes, add to queue
    uart3_tx_buffer[uart3_tx_insert_idx] = data;
    uart3_tx_insert_idx = temp;
  }
  else { // no, set running flag and write to output register
    uart3_tx_running = TRUE;
    USART_SendData(USART3, data);
  }
  USART_ITConfig(USART3, USART_IT_TXE, ENABLE);

}

bool_t uart3_check_free_space( uint8_t len) {
  int16_t space = uart3_tx_extract_idx - uart3_tx_insert_idx;
  if (space <= 0)
    space += UART3_TX_BUFFER_SIZE;
  return (uint16_t)(space - 1) >= len;
}


void usart3_irq_handler(void) {

  if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET){
    // check if more data to send
    if (uart3_tx_insert_idx != uart3_tx_extract_idx) {
      USART_SendData(USART3,uart3_tx_buffer[uart3_tx_extract_idx]);
      uart3_tx_extract_idx++;
      uart3_tx_extract_idx %= UART3_TX_BUFFER_SIZE;
    }
    else {
      uart3_tx_running = FALSE; // clear running flag
      USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
    }
  }

  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){
    uint16_t temp = (uart3_rx_insert_idx + 1) % UART3_RX_BUFFER_SIZE;;
    uart3_rx_buffer[uart3_rx_insert_idx] = USART_ReceiveData(USART3);
    // check for more room in queue
    if (temp != uart3_rx_extract_idx)
      uart3_rx_insert_idx = temp; // update insert index
  }

}


#endif /* USE_UART3 */

#ifdef USE_UART5

volatile uint16_t uart5_rx_insert_idx, uart5_rx_extract_idx;
uint8_t  uart5_rx_buffer[UART5_RX_BUFFER_SIZE];

volatile uint16_t uart5_tx_insert_idx, uart5_tx_extract_idx;
volatile bool_t uart5_tx_running;
uint8_t  uart5_tx_buffer[UART5_TX_BUFFER_SIZE];

void uart5_init( void ) {

  /* init RCC */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
  RCC_APB2PeriphClockCmd(UART5_PeriphTx, ENABLE);
  RCC_APB2PeriphClockCmd(UART5_PeriphRx, ENABLE);

  /* Enable UART5 interrupts */
  NVIC_InitTypeDef nvic;
  nvic.NVIC_IRQChannel = UART5_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 2;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

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
  USART_InitTypeDef usart;
  usart.USART_BaudRate            = UART5_BAUD;
  usart.USART_WordLength          = USART_WordLength_8b;
  usart.USART_StopBits            = USART_StopBits_1;
  usart.USART_Parity              = USART_Parity_No;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART5, &usart);
  /* Enable UART5 Receive interrupts */
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

  pprz_usart_set_baudrate(UART5, UART5_BAUD);

  /* Enable the UART5 */
  USART_Cmd(UART5, ENABLE);

  // initialize the transmit data queue
  uart5_tx_extract_idx = 0;
  uart5_tx_insert_idx = 0;
  uart5_tx_running = FALSE;

  // initialize the receive data queuenn
  uart5_rx_extract_idx = 0;
  uart5_rx_insert_idx = 0;

}

void uart5_transmit( uint8_t data ) {

  uint16_t temp = (uart5_tx_insert_idx + 1) % UART5_TX_BUFFER_SIZE;

  if (temp == uart5_tx_extract_idx)
    return;                          // no room

  USART_ITConfig(USART5, USART_IT_TXE, DISABLE);

  // check if in process of sending data
  if (uart5_tx_running) { // yes, add to queue
    uart5_tx_buffer[uart5_tx_insert_idx] = data;
    uart5_tx_insert_idx = temp;
  }
  else { // no, set running flag and write to output register
    uart5_tx_running = TRUE;
    USART_SendData(USART5, data);
  }
  USART_ITConfig(USART5, USART_IT_TXE, ENABLE);

}

bool_t uart5_check_free_space( uint8_t len) {
  int16_t space = uart5_tx_extract_idx - uart5_tx_insert_idx;
  if (space <= 0)
    space += UART5_TX_BUFFER_SIZE;
  return (uint16_t)(space - 1) >= len;
}


void usart5_irq_handler(void) {

  if(USART_GetITStatus(USART5, USART_IT_TXE) != RESET){
    // check if more data to send
    if (uart5_tx_insert_idx != uart5_tx_extract_idx) {
      USART_SendData(USART5,uart5_tx_buffer[uart5_tx_extract_idx]);
      uart5_tx_extract_idx++;
      uart5_tx_extract_idx %= UART5_TX_BUFFER_SIZE;
    }
    else {
      uart5_tx_running = FALSE; // clear running flag
      USART_ITConfig(USART5, USART_IT_TXE, DISABLE);
    }
  }

  if(USART_GetITStatus(USART5, USART_IT_RXNE) != RESET){
    uint16_t temp = (uart5_rx_insert_idx + 1) % UART5_RX_BUFFER_SIZE;;
    uart5_rx_buffer[uart5_rx_insert_idx] = USART_ReceiveData(USART5);
    // check for more room in queue
    if (temp != uart5_rx_extract_idx)
      uart5_rx_insert_idx = temp; // update insert index
  }

}


#endif /* USE_UART5 */

void uart_init( void )
{
#ifdef USE_UART1
  uart1_init();
#endif
#ifdef USE_UART2
  uart2_init();
#endif
#ifdef USE_UART3
  uart3_init();
#endif
#ifdef USE_UART5
  uart5_init();
#endif
}


