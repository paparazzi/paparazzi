/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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
 *
 */

/**
 * @file arch/lpc21/mcu_periph/uart_arch.c
 * @ingroup lpc21_arch
 *
 * Handling of UART hardware for lpc21xx.
 */


#include "mcu_periph/uart.h"
#include "armVIC.h"

static inline void uart_disable_interrupts(struct uart_periph *p)
{
  // disable interrups
  ((uartRegs_t *)(p->reg_addr))->ier = 0x00;  // disable all interrupts
  ((uartRegs_t *)(p->reg_addr))->iir;         // clear interrupt ID
  ((uartRegs_t *)(p->reg_addr))->rbr;         // clear receive register
  ((uartRegs_t *)(p->reg_addr))->lsr;         // clear line status register
}

static inline void uart_enable_interrupts(struct uart_periph *p)
{
  // enable receiver interrupts
  ((uartRegs_t *)(p->reg_addr))->ier = UIER_ERBFI;
}

static inline void uart_set_baudrate(struct uart_periph *p, uint32_t baud)
{
  p->baudrate = baud;

  /* calculate the baudrate */
  uint32_t _baud_reg_val = (uint16_t)((PCLK / (((float)baud) * 16.0)) + 0.5);
  /* select divisor latches */
  ((uartRegs_t *)(p->reg_addr))->lcr = ULCR_DLAB_ENABLE;
  /* set for baud low byte */
  ((uartRegs_t *)(p->reg_addr))->dll = (uint8_t)_baud_reg_val;
  /* set for baud high byte */
  ((uartRegs_t *)(p->reg_addr))->dlm = (uint8_t)(_baud_reg_val >> 8);

  // set the number of characters and other
  // user specified operating parameters
  // For now: hard wired configuration 8 bits 1 stop no parity
  //          fifo triger -> 8 bytes
  ((uartRegs_t *)(p->reg_addr))->lcr = (UART_8N1 & ~ULCR_DLAB_ENABLE);
  ((uartRegs_t *)(p->reg_addr))->fcr = UART_FIFO_8;
}

void uart_periph_set_baudrate(struct uart_periph *p, uint32_t baud)
{
  uart_disable_interrupts(p);
  uart_set_baudrate(p, baud);
  uart_enable_interrupts(p);
}

void uart_periph_set_bits_stop_parity(struct uart_periph *p __attribute__((unused)),
                                      uint8_t bits __attribute__((unused)), uint8_t stop __attribute__((unused)), uint8_t __attribute__((unused)) parity)
{
  // TBD
}

void uart_put_byte(struct uart_periph *p, uint8_t data)
{
  uint16_t temp;
  unsigned cpsr;

  temp = (p->tx_insert_idx + 1) % UART_TX_BUFFER_SIZE;

  if (temp == p->tx_extract_idx) {
    return;                          // no room
  }

  cpsr = disableIRQ();                                // disable global interrupts
  ((uartRegs_t *)(p->reg_addr))->ier &= ~UIER_ETBEI;  // disable TX interrupts
  restoreIRQ(cpsr);                                   // restore global interrupts

  // check if in process of sending data
  if (p->tx_running) {
    // add to queue
    p->tx_buf[p->tx_insert_idx] = data;
    p->tx_insert_idx = temp;
  } else {
    // set running flag and write to output register
    p->tx_running = 1;
    ((uartRegs_t *)(p->reg_addr))->thr = data;
  }

  cpsr = disableIRQ();                              // disable global interrupts
  ((uartRegs_t *)(p->reg_addr))->ier |= UIER_ETBEI; // enable TX interrupts
  restoreIRQ(cpsr);                                 // restore global interrupts
}

static inline void uart_ISR(struct uart_periph *p)
{
  uint8_t iid;
  // loop until not more interrupt sources
  while (((iid = ((uartRegs_t *)(p->reg_addr))->iir) & UIIR_NO_INT) == 0) {
    // identify & process the highest priority interrupt
    switch (iid & UIIR_ID_MASK) {
      case UIIR_RLS_INT:                // Receive Line Status
        ((uartRegs_t *)(p->reg_addr))->lsr; // read LSR to clear
        break;

      case UIIR_CTI_INT:                // Character Timeout Indicator
      case UIIR_RDA_INT:                // Receive Data Available
        do {
          uint16_t temp;

          // calc next insert index & store character
          temp = (p->rx_insert_idx + 1) % UART_RX_BUFFER_SIZE;
          p->rx_buf[p->rx_insert_idx] = ((uartRegs_t *)(p->reg_addr))->rbr;

          // check for more room in queue
          if (temp != p->rx_extract_idx) {
            p->rx_insert_idx = temp;  // update insert index
          }
        } while (((uartRegs_t *)(p->reg_addr))->lsr & ULSR_RDR);

        break;

      case UIIR_THRE_INT:               // Transmit Holding Register Empty
        while (((uartRegs_t *)(p->reg_addr))->lsr & ULSR_THRE) {
          // check if more data to send
          if (p->tx_insert_idx != p->tx_extract_idx) {
            ((uartRegs_t *)(p->reg_addr))->thr = p->tx_buf[p->tx_extract_idx];
            p->tx_extract_idx++;
            p->tx_extract_idx %= UART_TX_BUFFER_SIZE;
          } else {
            // no
            p->tx_running = 0;       // clear running flag
            break;
          }
        }

        break;

      default:                          // Unknown
        ((uartRegs_t *)(p->reg_addr))->lsr;
        ((uartRegs_t *)(p->reg_addr))->rbr;
        break;
    }
  }
}

#if USE_UART0

#ifndef UART0_VIC_SLOT
#define UART0_VIC_SLOT 5
#endif

/* by default enable UART Tx and Rx */
#ifndef USE_UART0_TX
#ifdef USE_UART0_RX_ONLY
#warning "USE_UART0_RX_ONLY is deprecated, please set USE_UART0_TX=FALSE instead"
#define USE_UART0_TX FALSE
#else
#define USE_UART0_TX TRUE
#endif
#endif
#ifndef USE_UART0_RX
#define USE_UART0_RX TRUE
#endif

void uart0_ISR(void) __attribute__((naked));

void uart0_ISR(void)
{
  // perform proper ISR entry so thumb-interwork works properly
  ISR_ENTRY();

  uart_ISR(&uart0);

  VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
  ISR_EXIT();                           // recover registers and return
}

void uart0_init(void)
{

  uart_periph_init(&uart0);
  uart0.reg_addr = UART0_BASE;

#if USE_UART0_RX && USE_UART0_TX
  // set port pins for UART0
  PINSEL0 = (PINSEL0 & ~U0_PINMASK) | U0_PINSEL;
#elif USE_UART0_RX
  // only use the RX0 P0.1 pin, no TX
  PINSEL0 = (PINSEL0 & ~U0_PINMASK_RX) | U0_PINSEL_RX;
#elif USE_UART0_TX
  // only use tx
  PINSEL0 = (PINSEL0 & ~U0_PINMASK_TX) | U0_PINSEL_TX;
#endif

  // initialize uart parameters
  uart_set_baudrate(&uart0, UART0_BAUD);

  // initialize the interrupt vector
  VICIntSelect &= ~VIC_BIT(VIC_UART0);                // UART0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_UART0);                  // UART0 interrupt enabled
  _VIC_CNTL(UART0_VIC_SLOT) = VIC_ENABLE | VIC_UART0;
  _VIC_ADDR(UART0_VIC_SLOT) = (uint32_t)uart0_ISR;    // address of the ISR

  uart_enable_interrupts(&uart0);
}

#endif /* USE_UART0 */

#if USE_UART1

#ifndef UART1_VIC_SLOT
#define UART1_VIC_SLOT 6
#endif

/* by default enable UART Tx and Rx */
#ifndef USE_UART1_TX
#ifdef USE_UART1_RX_ONLY
#warning "USE_UART1_RX_ONLY is deprecated, please set USE_UART1_TX=FALSE instead"
#define USE_UART1_TX FALSE
#else
#define USE_UART1_TX TRUE
#endif
#endif
#ifndef USE_UART1_RX
#define USE_UART1_RX TRUE
#endif

void uart1_ISR(void) __attribute__((naked));

void uart1_ISR(void)
{
  // perform proper ISR entry so thumb-interwork works properly
  ISR_ENTRY();

  uart_ISR(&uart1);

  VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
  ISR_EXIT();                           // recover registers and return
}

void uart1_init(void)
{

  uart_periph_init(&uart1);
  uart1.reg_addr = UART1_BASE;

#if USE_UART1_RX && USE_UART0_TX
  // set port pins for UART1
  PINSEL0 = (PINSEL0 & ~U1_PINMASK) | U1_PINSEL;
#elif USE_UART1_RX
  // only use the RX0 P0.1 pin, no TX
  PINSEL0 = (PINSEL0 & ~U1_PINMASK_RX) | U1_PINSEL_RX;
#elif USE_UART1_TX
  // only use tx
  PINSEL0 = (PINSEL0 & ~U1_PINMASK_TX) | U1_PINSEL_TX;
#endif

  uart_set_baudrate(&uart1, UART1_BAUD);

  // initialize the interrupt vector
  VICIntSelect &= ~VIC_BIT(VIC_UART1);                // UART1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_UART1);                  // UART1 interrupt enabled
  _VIC_CNTL(UART1_VIC_SLOT) = VIC_ENABLE | VIC_UART1;
  _VIC_ADDR(UART1_VIC_SLOT) = (uint32_t)uart1_ISR;    // address of the ISR

  // enable receiver interrupts
  uart_enable_interrupts(&uart1);
}

#endif

