/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file arch/stm32/mcu_periph/uart_arch.c
 * @ingroup stm32_arch
 *
 * Handling of UART hardware for STM32.
 */

#include "mcu_periph/uart.h"
#include "mcu_periph/gpio.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include "std.h"

#include BOARD_CONFIG

void uart_periph_set_baudrate(struct uart_periph *p, uint32_t baud)
{
  p->baudrate = baud;

  /* Configure USART baudrate */
  usart_set_baudrate((uint32_t)p->reg_addr, baud);

  /* Disable Idle Line interrupt */
  USART_CR1((uint32_t)p->reg_addr) &= ~USART_CR1_IDLEIE;

  /* Disable LIN break detection interrupt */
  USART_CR2((uint32_t)p->reg_addr) &= ~USART_CR2_LBDIE;

  /* Enable USART1 Receive interrupts */
  USART_CR1((uint32_t)p->reg_addr) |= USART_CR1_RXNEIE;

  /* Enable the USART */
  usart_enable((uint32_t)p->reg_addr);

}

void uart_periph_set_bits_stop_parity(struct uart_periph *p, uint8_t bits, uint8_t stop, uint8_t parity)
{
  /* Configure USART parity and data bits */
  if (parity == UPARITY_EVEN) {
    usart_set_parity((uint32_t)p->reg_addr, USART_PARITY_EVEN);
    if (bits == UBITS_7) {
      usart_set_databits((uint32_t)p->reg_addr, 8);
    } else { // 8 data bits by default
      usart_set_databits((uint32_t)p->reg_addr, 9);
    }
  } else if (parity == UPARITY_ODD) {
    usart_set_parity((uint32_t)p->reg_addr, USART_PARITY_ODD);
    if (bits == UBITS_7) {
      usart_set_databits((uint32_t)p->reg_addr, 8);
    } else { // 8 data bits by default
      usart_set_databits((uint32_t)p->reg_addr, 9);
    }
  } else { // 8 data bist, NO_PARITY by default
    usart_set_parity((uint32_t)p->reg_addr, USART_PARITY_NONE);
    usart_set_databits((uint32_t)p->reg_addr, 8); // is 7bits without parity possible ?
  }
  /* Configure USART stop bits */
  if (stop == USTOP_2) {
    usart_set_stopbits((uint32_t)p->reg_addr, USART_STOPBITS_2);
  } else { // 1 stop bit by default
    usart_set_stopbits((uint32_t)p->reg_addr, USART_STOPBITS_1);
  }
}

void uart_periph_set_mode(struct uart_periph *p, bool_t tx_enabled, bool_t rx_enabled, bool_t hw_flow_control)
{
  uint32_t mode = 0;
  if (tx_enabled) {
    mode |= USART_MODE_TX;
  }
  if (rx_enabled) {
    mode |= USART_MODE_RX;
  }

  /* set mode to Tx, Rx or TxRx */
  usart_set_mode((uint32_t)p->reg_addr, mode);

  if (hw_flow_control) {
    usart_set_flow_control((uint32_t)p->reg_addr, USART_FLOWCONTROL_RTS_CTS);
  } else {
    usart_set_flow_control((uint32_t)p->reg_addr, USART_FLOWCONTROL_NONE);
  }
}

void uart_transmit(struct uart_periph *p, uint8_t data)
{

  uint16_t temp = (p->tx_insert_idx + 1) % UART_TX_BUFFER_SIZE;

  if (temp == p->tx_extract_idx) {
    return;  // no room
  }

  USART_CR1((uint32_t)p->reg_addr) &= ~USART_CR1_TXEIE; // Disable TX interrupt

  // check if in process of sending data
  if (p->tx_running) { // yes, add to queue
    p->tx_buf[p->tx_insert_idx] = data;
    p->tx_insert_idx = temp;
  } else { // no, set running flag and write to output register
    p->tx_running = TRUE;
    usart_send((uint32_t)p->reg_addr, data);
  }

  USART_CR1((uint32_t)p->reg_addr) |= USART_CR1_TXEIE; // Enable TX interrupt

}

static inline void usart_isr(struct uart_periph *p)
{

  if (((USART_CR1((uint32_t)p->reg_addr) & USART_CR1_TXEIE) != 0) &&
      ((USART_SR((uint32_t)p->reg_addr) & USART_SR_TXE) != 0)) {
    // check if more data to send
    if (p->tx_insert_idx != p->tx_extract_idx) {
      usart_send((uint32_t)p->reg_addr, p->tx_buf[p->tx_extract_idx]);
      p->tx_extract_idx++;
      p->tx_extract_idx %= UART_TX_BUFFER_SIZE;
    } else {
      p->tx_running = FALSE;   // clear running flag
      USART_CR1((uint32_t)p->reg_addr) &= ~USART_CR1_TXEIE; // Disable TX interrupt
    }
  }

  if (((USART_CR1((uint32_t)p->reg_addr) & USART_CR1_RXNEIE) != 0) &&
      ((USART_SR((uint32_t)p->reg_addr) & USART_SR_RXNE) != 0) &&
      ((USART_SR((uint32_t)p->reg_addr) & USART_SR_ORE) == 0) &&
      ((USART_SR((uint32_t)p->reg_addr) & USART_SR_NE) == 0) &&
      ((USART_SR((uint32_t)p->reg_addr) & USART_SR_FE) == 0)) {
    uint16_t temp = (p->rx_insert_idx + 1) % UART_RX_BUFFER_SIZE;;
    p->rx_buf[p->rx_insert_idx] = usart_recv((uint32_t)p->reg_addr);
    // check for more room in queue
    if (temp != p->rx_extract_idx) {
      p->rx_insert_idx = temp;  // update insert index
    }
  } else {
    /* ORE, NE or FE error - read USART_DR reg and log the error */
    if (((USART_CR1((uint32_t)p->reg_addr) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR((uint32_t)p->reg_addr) & USART_SR_ORE) != 0)) {
      usart_recv((uint32_t)p->reg_addr);
      p->ore++;
    }
    if (((USART_CR1((uint32_t)p->reg_addr) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR((uint32_t)p->reg_addr) & USART_SR_NE) != 0)) {
      usart_recv((uint32_t)p->reg_addr);
      p->ne_err++;
    }
    if (((USART_CR1((uint32_t)p->reg_addr) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR((uint32_t)p->reg_addr) & USART_SR_FE) != 0)) {
      usart_recv((uint32_t)p->reg_addr);
      p->fe_err++;
    }
  }
}

static inline void usart_enable_irq(uint8_t IRQn)
{
  /* Note:
   * In libstm32 times the priority of this interrupt was set to
   * preemption priority 2 and sub priority 1
   */
  /* Enable USART interrupts */
  nvic_enable_irq(IRQn);
}


#if USE_UART1

/* by default enable UART Tx and Rx */
#ifndef USE_UART1_TX
#define USE_UART1_TX TRUE
#endif
#ifndef USE_UART1_RX
#define USE_UART1_RX TRUE
#endif

#ifndef UART1_HW_FLOW_CONTROL
#define UART1_HW_FLOW_CONTROL FALSE
#endif

#ifndef UART1_BITS
#define UART1_BITS UBITS_8
#endif

#ifndef UART1_STOP
#define UART1_STOP USTOP_1
#endif

#ifndef UART1_PARITY
#define UART1_PARITY UPARITY_NO
#endif

void uart1_init(void)
{

  uart_periph_init(&uart1);
  uart1.reg_addr = (void *)USART1;

  /* init RCC and GPIOs */
  rcc_periph_clock_enable(RCC_USART1);

#if USE_UART1_TX
  gpio_setup_pin_af(UART1_GPIO_PORT_TX, UART1_GPIO_TX, UART1_GPIO_AF, TRUE);
#endif
#if USE_UART1_RX
  gpio_setup_pin_af(UART1_GPIO_PORT_RX, UART1_GPIO_RX, UART1_GPIO_AF, FALSE);
#endif

  /* Enable USART interrupts in the interrupt controller */
  usart_enable_irq(NVIC_USART1_IRQ);

#if UART1_HW_FLOW_CONTROL
#warning "USING UART1 FLOW CONTROL. Make sure to pull down CTS if you are not connecting any flow-control-capable hardware."
  /* setup CTS and RTS gpios */
  gpio_setup_pin_af(UART1_GPIO_PORT_CTS, UART1_GPIO_CTS, UART1_GPIO_AF, FALSE);
  gpio_setup_pin_af(UART1_GPIO_PORT_RTS, UART1_GPIO_RTS, UART1_GPIO_AF, TRUE);
#endif

  /* Configure USART1, enable hardware flow control*/
  uart_periph_set_mode(&uart1, USE_UART1_TX, USE_UART1_RX, UART1_HW_FLOW_CONTROL);

  /* Set USART1 parameters and enable interrupt */
  uart_periph_set_bits_stop_parity(&uart1, UART1_BITS, UART1_STOP, UART1_PARITY);
  uart_periph_set_baudrate(&uart1, UART1_BAUD);
}

void usart1_isr(void) { usart_isr(&uart1); }

#endif /* USE_UART1 */


#if USE_UART2

/* by default enable UART Tx and Rx */
#ifndef USE_UART2_TX
#define USE_UART2_TX TRUE
#endif
#ifndef USE_UART2_RX
#define USE_UART2_RX TRUE
#endif

#ifndef UART2_HW_FLOW_CONTROL
#define UART2_HW_FLOW_CONTROL FALSE
#endif

#ifndef UART2_BITS
#define UART2_BITS UBITS_8
#endif

#ifndef UART2_STOP
#define UART2_STOP USTOP_1
#endif

#ifndef UART2_PARITY
#define UART2_PARITY UPARITY_NO
#endif

void uart2_init(void)
{

  uart_periph_init(&uart2);
  uart2.reg_addr = (void *)USART2;

  /* init RCC and GPIOs */
  rcc_periph_clock_enable(RCC_USART2);

#if USE_UART2_TX
  gpio_setup_pin_af(UART2_GPIO_PORT_TX, UART2_GPIO_TX, UART2_GPIO_AF, TRUE);
#endif
#if USE_UART2_RX
  gpio_setup_pin_af(UART2_GPIO_PORT_RX, UART2_GPIO_RX, UART2_GPIO_AF, FALSE);
#endif

  /* Enable USART interrupts in the interrupt controller */
  usart_enable_irq(NVIC_USART2_IRQ);

#if UART2_HW_FLOW_CONTROL && defined(STM32F4)
#warning "USING UART2 FLOW CONTROL. Make sure to pull down CTS if you are not connecting any flow-control-capable hardware."
  /* setup CTS and RTS pins */
  gpio_setup_pin_af(UART2_GPIO_PORT_CTS, UART2_GPIO_CTS, UART2_GPIO_AF, FALSE);
  gpio_setup_pin_af(UART2_GPIO_PORT_RTS, UART2_GPIO_RTS, UART2_GPIO_AF, TRUE);
#endif

  /* Configure USART Tx,Rx, and hardware flow control*/
  uart_periph_set_mode(&uart2, USE_UART2_TX, USE_UART2_RX, UART2_HW_FLOW_CONTROL);

  /* Configure USART */
  uart_periph_set_bits_stop_parity(&uart2, UART2_BITS, UART2_STOP, UART2_PARITY);
  uart_periph_set_baudrate(&uart2, UART2_BAUD);
}

void usart2_isr(void) { usart_isr(&uart2); }

#endif /* USE_UART2 */


#if USE_UART3

/* by default enable UART Tx and Rx */
#ifndef USE_UART3_TX
#define USE_UART3_TX TRUE
#endif
#ifndef USE_UART3_RX
#define USE_UART3_RX TRUE
#endif

#ifndef UART3_HW_FLOW_CONTROL
#define UART3_HW_FLOW_CONTROL FALSE
#endif

#ifndef UART3_BITS
#define UART3_BITS UBITS_8
#endif

#ifndef UART3_STOP
#define UART3_STOP USTOP_1
#endif

#ifndef UART3_PARITY
#define UART3_PARITY UPARITY_NO
#endif

void uart3_init(void)
{

  uart_periph_init(&uart3);
  uart3.reg_addr = (void *)USART3;

  /* init RCC */
  rcc_periph_clock_enable(RCC_USART3);

#if USE_UART3_TX
  gpio_setup_pin_af(UART3_GPIO_PORT_TX, UART3_GPIO_TX, UART3_GPIO_AF, TRUE);
#endif
#if USE_UART3_RX
  gpio_setup_pin_af(UART3_GPIO_PORT_RX, UART3_GPIO_RX, UART3_GPIO_AF, FALSE);
#endif

  /* Enable USART interrupts in the interrupt controller */
  usart_enable_irq(NVIC_USART3_IRQ);

#if UART3_HW_FLOW_CONTROL && defined(STM32F4)
#warning "USING UART3 FLOW CONTROL. Make sure to pull down CTS if you are not connecting any flow-control-capable hardware."
  /* setup CTS and RTS pins */
  gpio_setup_pin_af(UART3_GPIO_PORT_CTS, UART3_GPIO_CTS, UART3_GPIO_AF, FALSE);
  gpio_setup_pin_af(UART3_GPIO_PORT_RTS, UART3_GPIO_RTS, UART3_GPIO_AF, TRUE);
#endif

  /* Configure USART Tx,Rx, and hardware flow control*/
  uart_periph_set_mode(&uart3, USE_UART3_TX, USE_UART3_RX, UART3_HW_FLOW_CONTROL);

  /* Configure USART */
  uart_periph_set_bits_stop_parity(&uart3, UART3_BITS, UART3_STOP, UART3_PARITY);
  uart_periph_set_baudrate(&uart3, UART3_BAUD);
}

void usart3_isr(void) { usart_isr(&uart3); }

#endif /* USE_UART3 */


#if USE_UART4 && defined STM32F4

/* by default enable UART Tx and Rx */
#ifndef USE_UART4_TX
#define USE_UART4_TX TRUE
#endif
#ifndef USE_UART4_RX
#define USE_UART4_RX TRUE
#endif

#ifndef UART4_BITS
#define UART4_BITS UBITS_8
#endif

#ifndef UART4_STOP
#define UART4_STOP USTOP_1
#endif

#ifndef UART4_PARITY
#define UART4_PARITY UPARITY_NO
#endif

void uart4_init(void)
{

  uart_periph_init(&uart4);
  uart4.reg_addr = (void *)UART4;

  /* init RCC and GPIOs */
  rcc_periph_clock_enable(RCC_UART4);

#if USE_UART4_TX
  gpio_setup_pin_af(UART4_GPIO_PORT_TX, UART4_GPIO_TX, UART4_GPIO_AF, TRUE);
#endif
#if USE_UART4_RX
  gpio_setup_pin_af(UART4_GPIO_PORT_RX, UART4_GPIO_RX, UART4_GPIO_AF, FALSE);
#endif

  /* Enable USART interrupts in the interrupt controller */
  usart_enable_irq(NVIC_UART4_IRQ);

  /* Configure USART */
  uart_periph_set_mode(&uart4, USE_UART4_TX, USE_UART4_RX, FALSE);
  uart_periph_set_bits_stop_parity(&uart4, UART4_BITS, UART4_STOP, UART4_PARITY);
  uart_periph_set_baudrate(&uart4, UART4_BAUD);
}

void uart4_isr(void) { usart_isr(&uart4); }

#endif /* USE_UART4 */


#if USE_UART5

/* by default enable UART Tx and Rx */
#ifndef USE_UART5_TX
#define USE_UART5_TX TRUE
#endif
#ifndef USE_UART5_RX
#define USE_UART5_RX TRUE
#endif

#ifndef UART5_BITS
#define UART5_BITS UBITS_8
#endif

#ifndef UART5_STOP
#define UART5_STOP USTOP_1
#endif

#ifndef UART5_PARITY
#define UART5_PARITY UPARITY_NO
#endif

void uart5_init(void)
{

  uart_periph_init(&uart5);
  uart5.reg_addr = (void *)UART5;

  /* init RCC and GPIOs */
  rcc_periph_clock_enable(RCC_UART5);

#if USE_UART5_TX
  gpio_setup_pin_af(UART5_GPIO_PORT_TX, UART5_GPIO_TX, UART5_GPIO_AF, TRUE);
#endif
#if USE_UART5_RX
  gpio_setup_pin_af(UART5_GPIO_PORT_RX, UART5_GPIO_RX, UART5_GPIO_AF, FALSE);
#endif

  /* Enable USART interrupts in the interrupt controller */
  usart_enable_irq(NVIC_UART5_IRQ);

  /* Configure USART */
  uart_periph_set_mode(&uart5, USE_UART5_TX, USE_UART5_RX, FALSE);
  uart_periph_set_bits_stop_parity(&uart5, UART5_BITS, UART5_STOP, UART5_PARITY);
  uart_periph_set_baudrate(&uart5, UART5_BAUD);
}

void uart5_isr(void) { usart_isr(&uart5); }

#endif /* USE_UART5 */


#if USE_UART6 && defined STM32F4

/* by default enable UART Tx and Rx */
#ifndef USE_UART6_TX
#define USE_UART6_TX TRUE
#endif
#ifndef USE_UART6_RX
#define USE_UART6_RX TRUE
#endif

#ifndef UART6_HW_FLOW_CONTROL
#define UART6_HW_FLOW_CONTROL FALSE
#endif

#ifndef UART6_BITS
#define UART6_BITS UBITS_8
#endif

#ifndef UART6_STOP
#define UART6_STOP USTOP_1
#endif

#ifndef UART6_PARITY
#define UART6_PARITY UPARITY_NO
#endif

void uart6_init(void)
{

  uart_periph_init(&uart6);
  uart6.reg_addr = (void *)USART6;

  /* enable uart clock */
  rcc_periph_clock_enable(RCC_USART6);

  /* init RCC and GPIOs */
#if USE_UART6_TX
  gpio_setup_pin_af(UART6_GPIO_PORT_TX, UART6_GPIO_TX, UART6_GPIO_AF, TRUE);
#endif
#if USE_UART6_RX
  gpio_setup_pin_af(UART6_GPIO_PORT_RX, UART6_GPIO_RX, UART6_GPIO_AF, FALSE);
#endif

  /* Enable USART interrupts in the interrupt controller */
  usart_enable_irq(NVIC_USART6_IRQ);

#if UART6_HW_FLOW_CONTROL
#warning "USING UART6 FLOW CONTROL. Make sure to pull down CTS if you are not connecting any flow-control-capable hardware."
  /* setup CTS and RTS pins */
  gpio_setup_pin_af(UART6_GPIO_PORT_CTS, UART6_GPIO_CTS, UART6_GPIO_AF, FALSE);
  gpio_setup_pin_af(UART6_GPIO_PORT_RTS, UART6_GPIO_RTS, UART6_GPIO_AF, TRUE);
#endif

  /* Configure USART Tx,Rx and hardware flow control*/
  uart_periph_set_mode(&uart6, USE_UART6_TX, USE_UART6_RX, UART6_HW_FLOW_CONTROL);
  uart_periph_set_bits_stop_parity(&uart6, UART6_BITS, UART6_STOP, UART6_PARITY);
  uart_periph_set_baudrate(&uart6, UART6_BAUD);
}

void usart6_isr(void) { usart_isr(&uart6); }

#endif /* USE_UART6 */
