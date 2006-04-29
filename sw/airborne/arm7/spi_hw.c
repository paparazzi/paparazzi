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

#include "spi.h"

#include "std.h"
#include "LPC21xx.h"
#include "armVIC.h"

#include "link_mcu.h"

#ifdef FBW
void SPI1_ISR(void) __attribute__((naked));

/* SSPCR0 settings */
#define SSP_DDS  0x07 << 0  /* data size            : 8 bits   */
#define SSP_FRF  0x00 << 4  /* frame format         : SPI      */
#define SSP_CPOL 0x00 << 6  /* clock polarity       : idle low */  
#define SSP_CPHA 0x01 << 7  /* clock phase          : 1        */
#define SSP_SCR  0x0F << 8  /* serial clock rate    :          */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode        : disabled */
#define SSP_SSE  0x01 << 0  /* SSP enable           : disabled */
#define SSP_MS   0x01 << 2  /* master slave mode    : slave    */
#define SSP_SOD  0x00 << 3  /* slave output disable : disabled */

/* SSP (SPI1) pins
   P0.17 SCK    PINSEL1 2 << 2
   P0.18 MISO   PINSEL1 2 << 4
   P0.19 MOSI   PINSEL1 2 << 6
   P0.20 SS     PINSEL1 2 << 8 
*/

void spi_init( void ) {

  /* setup pins for SSP (SCK, MISO, MOSI, SS) */
  PINSEL1 |= 2 << 2 | 2 << 4 | 2 << 6 | 2 << 8;

  /* setup SSP  */
  SSPCR0 = SSP_DDS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = 0x20;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);     // SPI1 interrupt enabled
  VICVectCntl7 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr7 = (uint32_t)SPI1_ISR;    // address of the ISR

  /* enable half empty tx interrupt */
  SpiEnableTxi();
  /* enable SPI */
  SpiStart();

}

void SPI1_ISR(void) {
 ISR_ENTRY();

 if (bit_is_set(SSPMIS, TXMIS)) {  /*  Tx half empty */
   LinkMcuTransmit();
   LinkMcuReceive();
   SpiEnableRti();
 }
 
 if ( bit_is_set(SSPMIS, RTMIS)) { /* Rx timeout     */ 
   //   LED_ON(2);
   LinkMcuReceive();
   SpiDisableRti();
   SpiClearRti();                /* clear interrupt */
   link_mcu_is_busy = FALSE;
   link_mcu_was_busy = TRUE;
   //   LED_OFF(2);
}
 
 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}

#endif /* FBW */


/*
 *
 * AP MCU code
 *
 *
 */

#ifdef AP

#include "led.h"  /* FIXME remove that */

/* interrupt handler */
void SPI1_ISR(void) __attribute__((naked));

/* SSPCR0 settings */
#define SSP_DDS  0x07 << 0  /* data size         : 8 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : SCK idles low */  
#define SSP_CPHA 0x01 << 7  /* clock phase       : data captured on second clock transition */
#define SSP_SCR  0x0F << 8  /* serial clock rate : divide by 16  */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE  0x01 << 0  /* SSP enable        : disabled                  */
#define SSP_MS   0x00 << 2  /* master slave mode : master                    */
#define SSP_SOD  0x00 << 3  /* slave output disable : don't care when master */

void spi_init( void ) {

  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= 2 << 2 | 2 << 4 | 2 << 6;

  /* setup slave0_select pin */
  IO0DIR |= 1 << 20;  /* slave0_select is output */
  IO0SET |= 1 << 20;  /* slave0 is unselected    */

  /* setup slave1_select pin */
  PINSEL2 &= ~(_BV(3)); /* P1.25-16 are used as GPIO */
  IO1DIR |= 1 << 20;    /* slave1_select is output   */
  IO1SET |= 1 << 20;    /* slave1 is unselected      */

  /* setup SSP */
  SSPCR0 = SSP_DDS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_SSE | SSP_MS | SSP_SOD;
  SSPCPSR = 0x20;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);     // SPI1 interrupt enabled
  VICVectCntl7 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr7 = (uint32_t)SPI1_ISR;    // address of the ISR 

}

void SPI1_ISR(void) {
 ISR_ENTRY();
 // LED_TOGGLE(2);

 // SPI_SELECT_SLAVE1(); /* debug */
 if (bit_is_set(SSPMIS, TXMIS)) {  /*  Tx fifo is half empty */
   LinkMcuTransmit();
   LinkMcuReceive();
 }

 if ( bit_is_set(SSPMIS, RTMIS)) { /* Rx fifo is not empty and no receive took place in the last 32 bits period */ 
   SpiUnselectSlave0();
   LinkMcuReceive();
   SpiStop();
   SpiDisableRti();
   SpiClearRti();                /* clear interrupt */
 }

 // SPI_UNSELECT_SLAVE1(); /* debug */
 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}

#endif
