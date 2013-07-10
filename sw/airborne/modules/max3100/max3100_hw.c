/*
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

#include "subsystems/datalink/downlink.h"
#include "mcu_periph/uart.h"


uint8_t volatile max3100_status;
bool volatile max3100_data_available;
bool volatile max3100_transmit_buffer_empty;

uint8_t volatile max3100_tx_insert_idx, max3100_tx_extract_idx;
uint8_t volatile max3100_rx_insert_idx, max3100_rx_extract_idx;

uint8_t volatile max3100_tx_buf[MAX3100_TX_BUF_LEN];
uint8_t volatile max3100_rx_buf[MAX3100_RX_BUF_LEN];


bool read_bytes = false;


static void EXTINT_ISR(void) __attribute__((naked));
static void SPI1_ISR(void) __attribute__((naked));

#define PINSEL1_SCK  (2 << 2)
#define PINSEL1_MISO (2 << 4)
#define PINSEL1_MOSI (2 << 6)
#define PINSEL1_SSEL (2 << 8)

/* SSPCR0 settings */
#define SSP_DSS  0x0F << 0  /* data size            : 16 bits   */
// #define SSP_DSS  0x07 << 0  /* data size            : 8 bits   */
#define SSP_FRF  0x00 << 4  /* frame format         : SPI      */
#define SSP_CPOL 0x00 << 6  /* clock polarity       : idle low */
#define SSP_CPHA 0x00 << 7  /* clock phase          : 0        */
#define SSP_SCR  0x0F << 8  /* serial clock rate    : 29.3kHz, SSP input clock / 16 */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE  0x00 << 1  /* SSP enable        : disabled                  */
#define SSP_MS   0x00 << 2  /* master slave mode : master                    */
#define SSP_SOD  0x00 << 3  /* slave output disable : don't care when master */

#ifndef SSPCPSR_VAL
#define SSPCPSR_VAL 0x04
#endif

#warning "This driver should be updated to use the new SPI peripheral"

#ifndef SPI1_VIC_SLOT
#define SPI1_VIC_SLOT 7
#endif


void max3100_init( void ) {
  max3100_status = MAX3100_STATUS_IDLE;
  max3100_data_available = false;
  max3100_transmit_buffer_empty = true;
  max3100_tx_insert_idx = 0;
  max3100_tx_extract_idx = 0;
  max3100_rx_insert_idx = 0;
  max3100_rx_extract_idx = 0;

  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= PINSEL1_SCK | PINSEL1_MISO | PINSEL1_MOSI;

  /* setup SSP */
  SSPCR0 = SSP_DSS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = SSPCPSR_VAL; /* Prescaler */


  /* From arm7/max1167_hw.c */

  /* SS pin is output */
  SetBit(MAX3100_SS_IODIR, MAX3100_SS_PIN);
  /* unselected max3100 */
  Max3100Unselect();

  /* connect extint (IRQ) */
  MAX3100_IRQ_PINSEL |= MAX3100_IRQ_PINSEL_VAL << MAX3100_IRQ_PINSEL_BIT;
  /* extint is edge trigered */
  SetBit(EXTMODE, MAX3100_IRQ_EINT);
  /* extint is trigered on falling edge */
  ClearBit(EXTPOLAR, MAX3100_IRQ_EINT);
  /* clear pending extint0 before enabling interrupts */
  SetBit(EXTINT, MAX3100_IRQ_EINT);

  /* Configure interrupt vector for external pin interrupt */
  VICIntSelect &= ~VIC_BIT( MAX3100_VIC_EINT );    // EXTINT selected as IRQ
  VICIntEnable = VIC_BIT( MAX3100_VIC_EINT );             // EXTINT interrupt enabled
  VICVectCntl8 = VIC_ENABLE | MAX3100_VIC_EINT;
  VICVectAddr8 = (uint32_t)EXTINT_ISR;   // address of the ISR

  /* Configure interrupt vector for SPI */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT(VIC_SPI1);     /* SPI1 interrupt enabled */
  _VIC_CNTL(SPI1_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
  _VIC_CNTL(SPI1_VIC_SLOT) = (uint32_t)SPI1_ISR;    /* address of the ISR */

  /* Write configuration */
  //Max3100TransmitConf(MAX3100_BAUD_RATE | MAX3100_BIT_NOT_TM);
  Max3100TransmitConf(MAX3100_BAUD_RATE | MAX3100_BIT_NOT_RM | MAX3100_BIT_NOT_TM);
}


/******* External interrupt: Data input available ***********/
void EXTINT_ISR(void) {
  ISR_ENTRY();

  max3100_data_available = true;

  SetBit(EXTINT, MAX3100_IRQ_EINT);   /* clear extint */
  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */

  ISR_EXIT();
}

void SPI1_ISR(void) {
  ISR_ENTRY();

  while (bit_is_set(SSPSR, RNE)) {
    uint16_t data = SSPDR;

    if (bit_is_set(data, MAX3100_R_BIT)) { /* Data available */
      max3100_rx_buf[max3100_rx_insert_idx] = data & 0xff;
      max3100_rx_insert_idx++;  // automatic overflow because len=256
      read_bytes = true;
    }
    if (bit_is_set(data, MAX3100_T_BIT) && (max3100_status == MAX3100_STATUS_READING)) { /* transmit buffer empty */
      max3100_transmit_buffer_empty = true;
    }
  }
  SpiClearRti();                  /* clear interrupt */
  SpiDisableRti();
  SpiDisable ();
  Max3100Unselect();
  max3100_status = MAX3100_STATUS_IDLE;

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

void max3100_debug(void) {
  /***     DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, 16, max3100_rx_buf); ***/
}
