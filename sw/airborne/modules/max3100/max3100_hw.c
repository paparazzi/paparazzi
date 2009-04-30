/*
 * $Id$
 *  
 * Copyright (C) 2009  ENAC
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

#include "LPC21xx.h"
#include "interrupt_hw.h"  
#include "max3100_hw.h"
#include "led.h"

uint8_t max3100_status;
bool max3100_data_available;

uint8_t max3100_tx_insert_idx, max3100_tx_extract_idx;
uint8_t max3100_rx_insert_idx, max3100_rx_extract_idx;

uint8_t max3100_tx_buf[MAX3100_TX_BUF_LEN];
uint8_t max3100_rx_buf[MAX3100_RX_BUF_LEN];


static void EXTINT0_ISR(void) __attribute__((naked));
static void SPI1_ISR(void) __attribute__((naked));


void max3100_init( void ) {

  max3100_status = MAX3100_STATUS_IDLE;
  max3100_data_available = false;
  max3100_tx_insert_idx = 0;
  max3100_tx_extract_idx = 0;
  max3100_rx_insert_idx = 0;
  max3100_rx_extract_idx = 0;

  /* From arm7/max1167_hw.c */
  
  /* SS pin is output */
  SetBit(MAX3100_SS_IODIR, MAX3100_SS_PIN);
  /* unselected max3100 */
  Max3100Unselect();

  /* connect P0.16 to extint0 (IRQ) */
  MAX3100_IRQ_PINSEL |= MAX3100_IRQ_PINSEL_VAL << MAX3100_IRQ_PINSEL_BIT;
  /* extint0 is edge trigered */
  SetBit(EXTMODE, MAX3100_IRQ_EINT);
  /* extint0 is trigered on falling edge */
  ClearBit(EXTPOLAR, MAX3100_IRQ_EINT);
  /* clear pending extint0 before enabling interrupts */
  SetBit(EXTINT, MAX3100_IRQ_EINT);

   /* Configure interrupt vector for external pin interrupt */
  VICIntSelect &= ~VIC_BIT( VIC_EINT0 );                     // EXTINT0 selected as IRQ
  VICIntEnable = VIC_BIT( VIC_EINT0 );                       // EXTINT0 interrupt enabled
  VICVectCntl8 = VIC_ENABLE | VIC_EINT0;
  VICVectAddr8 = (uint32_t)EXTINT0_ISR;   // address of the ISR 

  /* Configure interrupt vector for SPI */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT(VIC_SPI1);     /* SPI1 interrupt enabled */
  VICVectCntl7 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr7 = (uint32_t)SPI1_ISR;    /* address of the ISR */

  /* Write configuration */
  Max3100TransmitConf(MAX3100_BAUD_RATE | MAX3100_BIT_NOT_RM);
}


void EXTINT0_ISR(void) {
  ISR_ENTRY();

  max3100_data_available = true;
  
  SetBit(EXTINT, MAX3100_IRQ_EINT);   /* clear extint0 */
  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */

  ISR_EXIT();
}

void SPI1_ISR(void) {
  uint8_t read_byte;

  ISR_ENTRY();

  Max3100Unselect();
  max3100_status = MAX3100_STATUS_IDLE;

  switch (max3100_status) {
  case MAX3100_STATUS_READING:
    SpiRead(read_byte); 
    SpiRead(max3100_rx_buf[max3100_rx_insert_idx]);
    max3100_rx_insert_idx++;  // automatic overflow because len=256
    max3100_data_available = Max3100BitR(read_byte);
    break;

  case MAX3100_STATUS_WRITING:
    SpiRead(read_byte); 
    SpiRead(read_byte);
    break;
  }

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}



void max3100_test_write( void ) {
  max3100_putchar('a');
}
