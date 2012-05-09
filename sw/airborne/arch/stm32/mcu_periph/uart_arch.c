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

#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f1/gpio.h>
#include "std.h"

void uart_periph_set_baudrate(struct uart_periph* p, uint32_t baud) {

  /* Configure USART */
  usart_set_baudrate((u32)p->reg_addr, baud);
  usart_set_databits((u32)p->reg_addr, 8);
  usart_set_stopbits((u32)p->reg_addr, USART_STOPBITS_1);
  usart_set_parity((u32)p->reg_addr, USART_PARITY_NONE);
  usart_set_flow_control((u32)p->reg_addr, USART_FLOWCONTROL_NONE);
  usart_set_mode((u32)p->reg_addr, USART_MODE_TX_RX);

  /* Enable USART1 Receive interrupts */
  USART_CR1((u32)p->reg_addr) |= USART_CR1_RXNEIE;

  /* Enable the USART */
  usart_enable((u32)p->reg_addr);

}
// XXX: TODO set_mode function

void uart_transmit(struct uart_periph* p, uint8_t data ) {

  uint16_t temp = (p->tx_insert_idx + 1) % UART_TX_BUFFER_SIZE;

  if (temp == p->tx_extract_idx)
    return;                          // no room

  USART_CR1((u32)p->reg_addr) &= ~USART_CR1_TXEIE; // Disable TX interrupt

  // check if in process of sending data
  if (p->tx_running) { // yes, add to queue
    p->tx_buf[p->tx_insert_idx] = data;
    p->tx_insert_idx = temp;
  }
  else { // no, set running flag and write to output register
    p->tx_running = TRUE;
    usart_send((u32)p->reg_addr, data);
  }

  USART_CR1((u32)p->reg_addr) |= USART_CR1_TXEIE; // Enable TX interrupt

}

static inline void usart_isr(struct uart_periph* p) {

  if (((USART_CR1((u32)p->reg_addr) & USART_CR1_TXEIE) != 0) &&
	  ((USART_SR((u32)p->reg_addr) & USART_SR_TXE) != 0)) {
    // check if more data to send
    if (p->tx_insert_idx != p->tx_extract_idx) {
      usart_send((u32)p->reg_addr,p->tx_buf[p->tx_extract_idx]);
      p->tx_extract_idx++;
      p->tx_extract_idx %= UART_TX_BUFFER_SIZE;
    }
    else {
      p->tx_running = FALSE;   // clear running flag
      USART_CR1((u32)p->reg_addr) &= ~USART_CR1_TXEIE; // Disable TX interrupt
    }
  }

  if (((USART_CR1((u32)p->reg_addr) & USART_CR1_RXNEIE) != 0) &&
		  ((USART_SR((u32)p->reg_addr) & USART_SR_RXNE) != 0)) {
    uint16_t temp = (p->rx_insert_idx + 1) % UART_RX_BUFFER_SIZE;;
    p->rx_buf[p->rx_insert_idx] = usart_recv((u32)p->reg_addr);
    // check for more room in queue
    if (temp != p->rx_extract_idx)
      p->rx_insert_idx = temp; // update insert index
  }

}

static inline void usart_enable_irq(u8 IRQn) {
  /* Note:
   * In libstm32 times the priority of this interrupt was set to
   * preemption priority 2 and sub priority 1
   */
  /* Enable USART interrupts */
  nvic_enable_irq(IRQn);
}

#ifdef USE_UART1

void uart1_init( void ) {

  uart_periph_init(&uart1);
  uart1.reg_addr = (void *)USART1;

  /* init RCC */
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);

  /* Enable USART interrupts in the interrupt controller */
  usart_enable_irq(NVIC_USART1_IRQ);

  /* Init GPIOS */
  gpio_set_mode(GPIO_BANK_USART1_TX, GPIO_MODE_OUTPUT_50_MHZ,
	  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
  gpio_set_mode(GPIO_BANK_USART1_RX, GPIO_MODE_INPUT,
	  GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

  /* Configure USART */
  uart_periph_set_baudrate(&uart1, UART1_BAUD);

}

void usart1_isr(void) { usart_isr(&uart1); }

#endif /* USE_UART1 */

#ifdef USE_UART2

void uart2_init( void ) {

  uart_periph_init(&uart2);
  uart2.reg_addr = (void *)USART2;

  /* init RCC */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);

  /* Enable USART interrupts in the interrupt controller */
  usart_enable_irq(NVIC_USART2_IRQ);

  /* Init GPIOS */
  gpio_set_mode(GPIO_BANK_USART2_TX, GPIO_MODE_OUTPUT_50_MHZ,
	  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
  gpio_set_mode(GPIO_BANK_USART2_RX, GPIO_MODE_INPUT,
	  GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

  /* Configure USART */
  uart_periph_set_baudrate(&uart2, UART2_BAUD);

}

void usart2_isr(void) { usart_isr(&uart2); }

#endif /* USE_UART2 */

#ifdef USE_UART3

void uart3_init( void ) {

  uart_periph_init(&uart3);
  uart3.reg_addr = (void *)USART3;

  /* init RCC */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART3EN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);

  /* Enable USART interrupts in the interrupt controller */
  usart_enable_irq(NVIC_USART3_IRQ);

  /* Init GPIOS */
  AFIO_MAPR |= AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP;
  gpio_set_mode(GPIO_BANK_USART3_PR_TX, GPIO_MODE_OUTPUT_50_MHZ,
	  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_PR_TX);
  gpio_set_mode(GPIO_BANK_USART3_PR_RX, GPIO_MODE_INPUT,
	  GPIO_CNF_INPUT_FLOAT, GPIO_USART3_PR_RX);

  /* Configure USART */
  uart_periph_set_baudrate(&uart3, UART3_BAUD);

}

void usart3_isr(void) { usart_isr(&uart3); }

#endif /* USE_UART3 */

#ifdef USE_UART5

void uart5_init( void ) {

  uart_periph_init(&uart5);
  uart5.reg_addr = (void *)UART5;

  /* init RCC */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_UART5EN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPDEN);

  /* Enable USART interrupts in the interrupt controller */
  usart_enable_irq(NVIC_UART5_IRQ);

  /* Init GPIOS */
  gpio_set_mode(GPIO_BANK_UART5_TX, GPIO_MODE_OUTPUT_50_MHZ,
	  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_UART5_TX);
  gpio_set_mode(GPIO_BANK_UART5_RX, GPIO_MODE_INPUT,
	  GPIO_CNF_INPUT_FLOAT, GPIO_UART5_RX);

  /* Configure USART */
  uart_periph_set_baudrate(&uart5, UART5_BAUD);

}

void uart5_isr(void) { usart_isr(&uart5); }

#endif /* USE_UART5 */

