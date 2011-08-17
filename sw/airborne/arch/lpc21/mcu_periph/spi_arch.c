/*  $Id$
 *
 * Copyright (C) 2003-2005  Pascal Brisset, Antoine Drouin
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

/** \brief handling of arm7 SPI hardware
 *  for now only SPI1 ( aka SSP )
 */

#include "mcu_periph/spi.h"

#include "std.h"
#include "LPC21xx.h"
#include "armVIC.h"

volatile uint8_t spi_tx_idx;
volatile uint8_t spi_rx_idx;

/* SSP (SPI1) pins (UM10120_1.pdf page 76)
   P0.17 SCK    PINSEL1 2 << 2
   P0.18 MISO   PINSEL1 2 << 4
   P0.19 MOSI   PINSEL1 2 << 6
   P0.20 SS     PINSEL1 2 << 8
*/
#define PINSEL1_SCK  (2 << 2)
#define PINSEL1_MISO (2 << 4)
#define PINSEL1_MOSI (2 << 6)
#define PINSEL1_SSEL (2 << 8)

#ifdef SPI_SLAVE
void SPI1_ISR(void) __attribute__((naked));

/* set SSP input clock, PCLK / CPSDVSR = 468.75kHz */

#if (PCLK == 15000000)
#define CPSDVSR    32
#else

#if (PCLK == 30000000)
#define CPSDVSR    64
#else

#if (PCLK == 60000000)
#define CPSDVSR    128
#else

#error unknown PCLK frequency
#endif
#endif
#endif

/* SSPCR0 settings */
#define SSP_DSS  0x07 << 0  /* data size            : 8 bits   */
#define SSP_FRF  0x00 << 4  /* frame format         : SPI      */
#define SSP_CPOL 0x00 << 6  /* clock polarity       : idle low */
#define SSP_CPHA 0x01 << 7  /* clock phase          : 1        */
#define SSP_SCR  0x0F << 8  /* serial clock rate    : 29.3kHz, SSP input clock / 16 */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode        : disabled */
#define SSP_SSE  0x00 << 1  /* SSP enable           : disabled */
#define SSP_MS   0x01 << 2  /* master slave mode    : slave    */
#define SSP_SOD  0x00 << 3  /* slave output disable : disabled */

void spi_init( void ) {
  /* setup pins for SSP (SCK, MISO, MOSI, SS) */
  PINSEL1 |= PINSEL1_SCK | PINSEL1_MISO | PINSEL1_MOSI | PINSEL1_SSEL;

  /* setup SSP  */
  SSPCR0 = SSP_DSS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = CPSDVSR; /* Prescaler, UM10120_1.pdf page 167 */

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);     // SPI1 interrupt enabled
  VICVectCntl7 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr7 = (uint32_t)SPI1_ISR;    // address of the ISR

  /* enable SPI */
  //  SpiEnable();
}

void SPI1_ISR(void) {
 ISR_ENTRY();

 if (bit_is_set(SSPMIS, TXMIS)) {  /*  Tx half empty */
   SpiTransmit();
   SpiReceive();
   SpiEnableRti();
 }

 if ( bit_is_set(SSPMIS, RTMIS)) { /* Rx timeout      */
   SpiReceive();
   SpiClearRti();                  /* clear interrupt */
   SpiDisableRti();
   SpiDisable();
   spi_message_received = TRUE;
 }

 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}

#endif /* SPI_SLAVE */


/*
 *
 * SPI Master code
 *
 *
 */

#ifdef SPI_MASTER

#include "led.h"  /* FIXME remove that */

/* interrupt handler */
void SPI1_ISR(void) __attribute__((naked));

/* SSPCR0 settings */
#define SSP_DSS  0x07 << 0  /* data size         : 8 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : SCK idles low */
#define SSP_CPHA 0x01 << 7  /* clock phase       : data captured on second clock transition */
#define SSP_SCR  0x00 << 8  /* serial clock rate   */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE  0x00 << 1  /* SSP enable        : disabled                  */
#define SSP_MS   0x00 << 2  /* master slave mode : master                    */
#define SSP_SOD  0x00 << 3  /* slave output disable : don't care when master */

#ifndef SSPCPSR_VAL
#define SSPCPSR_VAL 0x20
#endif

void spi_init( void ) {
  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= PINSEL1_SCK | PINSEL1_MISO | PINSEL1_MOSI;

#if defined USE_SPI_SLAVE0
  /* setup slave0_select pin */
  SPI_SELECT_SLAVE0_IODIR |= 1 << SPI_SELECT_SLAVE0_PIN;  /* slave0_select is output */
  SpiUnselectSlave0();  /* slave0 is unselected    */
#endif

#if defined USE_SPI_SLAVE1
  /* setup slave1_select pin */
  PINSEL2 &= ~(_BV(3)); /* P1.25-16 are used as GPIO */
  SPI_SELECT_SLAVE1_IODIR |= 1 << SPI_SELECT_SLAVE1_PIN;    /* slave1_select is output   */
  SpiUnselectSlave1();    /* slave1 is unselected      */
#endif

  /* setup SSP */
  SSPCR0 = SSP_DSS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = SSPCPSR_VAL; /* Prescaler */

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT(VIC_SPI1);     /* SPI1 interrupt enabled */
  VICVectCntl7 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr7 = (uint32_t)SPI1_ISR;    /* address of the ISR */
}

void SPI1_ISR(void) {
  ISR_ENTRY();

  if (bit_is_set(SSPMIS, TXMIS)) {  /*  Tx fifo is half empty */
    SpiTransmit();
    SpiReceive();
    SpiEnableRti();
  }

  if (bit_is_set(SSPMIS, RTMIS)) { /* Rx fifo is not empty and no receive took place in the last 32 bits period */
    SpiUnselectCurrentSlave();
    SpiReceive();
    SpiDisableRti();
    SpiClearRti();                /* clear interrupt */
    SpiDisable();
    spi_message_received = TRUE;
  }

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

#endif /** SPI_MASTER */
