/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Drouin
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

#include "booz_link_mcu.h"


uint8_t* link_mcu_tx_buf;
uint8_t* link_mcu_rx_buf;





#ifdef BOOZ_FILTER_MCU

volatile uint8_t link_mcu_status;
volatile uint8_t link_mcu_buf_idx;

uint32_t it_count;
uint16_t foo_debug;


/*
  wiring on IMU_V3
  P0_4   SCK0
  P0_5   MISO0
  P0_6   MOSI0
  P0_7   SSEL0
  P1_24  DRDY
*/

#include "armVIC.h"


void SPI0_ISR(void) __attribute__((naked));


/*  */
#define S0SPCR_BitEnable 0 << 2 /* enable more than 8 bits transferts */
#define S0SPCR_CPHA      1 << 3 /* data sampled on first edge         */
#define S0SPCR_CPOL      0 << 4 /* clock idles high                   */
#define S0SPCR_MSTR      1 << 5 /* master mode                        */
#define S0SPCR_LSBF      0 << 6 /* MSB first                          */
#define S0SPCR_SPIE      0 << 7 /* SPI interrupt disabled             */
#define S0SPCR_BITS      0 << 8 /* 16 bits transferts                 */

#define S0SPCR_VAL ( S0SPCR_BitEnable |		\
		     S0SPCR_CPHA |		\
		     S0SPCR_CPOL |		\
		     S0SPCR_MSTR |		\
		     S0SPCR_LSBF |		\
		     S0SPCR_SPIE |		\
		     S0SPCR_BITS )



#define SPI0_EnableInterrupt()  SetBit(S0SPCR, 7)
#define SPI0_ClearInterrupt()   SetBit(S0SPINT, SPI0IF)

void booz_link_mcu_hw_init ( void ) {

  /* SS pin is output */
  SetBit(IO0DIR, SS_PIN);
  SPI0_UnselectSlave();

  /* init SPI0 */
  /* setup pins for sck, miso, mosi */
  PINSEL0 |= 1<<8 | 1<<10 | 1<<12;
  /* configure SPI : see above */
  S0SPCR = S0SPCR_VAL;
  /* setup SPI clock rate ~ 450Khz with 15MHz VPB */
  S0SPCCR = 0x40;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI0);   // SPI0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI0);     // SPI0 interrupt enabled
  VICVectCntl3 = VIC_ENABLE | VIC_SPI0;
  VICVectAddr3 = (uint32_t)SPI0_ISR;    // address of the ISR

  /* clear all potentially pending interrupts */
  uint16_t foo1 __attribute__((unused)) = S0SPSR;
  uint16_t foo2 __attribute__((unused)) = S0SPDR;
  SPI0_ClearInterrupt();
  SPI0_EnableInterrupt();

  link_mcu_status = LINK_MCU_STATUS_IDLE;
  link_mcu_buf_idx = 0;

  link_mcu_tx_buf = (uint8_t*)&inter_mcu_state;
  link_mcu_rx_buf = (uint8_t*)&booz_link_mcu_state_unused;

}


void SPI0_ISR(void) {
  ISR_ENTRY();
  /* read status register */
  uint16_t foo __attribute__((unused)) = S0SPSR;
  foo_debug = foo;
  it_count++;

  /* read_data_register   */
  link_mcu_rx_buf[link_mcu_buf_idx] = S0SPDR;
  link_mcu_buf_idx++;
  if (link_mcu_buf_idx < LINK_MCU_BUF_LEN) {
    S0SPDR = link_mcu_tx_buf[link_mcu_buf_idx];
  }
  else {
    SPI0_UnselectSlave();
    link_mcu_status = LINK_MCU_STATUS_IDLE;
  }

  SPI0_ClearInterrupt();
  /* clear this interrupt from the VIC */
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}

#endif /* BOOZ_FILTER_MCU */














#ifdef BOOZ_CONTROLLER_MCU  

/* 
   IMU connected to SSP ( aka SPI1 ) as master
   Controller is slave
*/

volatile uint32_t it_cnt;
volatile uint32_t rx_it_cnt;
volatile uint32_t ti_it_cnt;

#include "LPC21xx.h"
#include "interrupt_hw.h" 

volatile uint8_t link_mcu_rx_buf_idx;
volatile uint8_t link_mcu_tx_buf_idx;

#define LinkMcuReceive() {	         				\
    while (bit_is_set(SSPSR, RNE)) {					\
      if (link_mcu_rx_buf_idx < LINK_MCU_BUF_LEN) {			\
	link_mcu_rx_buf[link_mcu_rx_buf_idx] = SSPDR;			\
	link_mcu_rx_buf_idx++;						\
	if (link_mcu_rx_buf_idx == LINK_MCU_BUF_LEN)			\
	  booz_link_mcu_status = BOOZ_LINK_MCU_DATA_AVAILABLE;		\
      }                                                                 \
      else {                                                            \
	uint8_t foo __attribute__ ((unused));				\
	foo = SSPDR;							\
      }                                                                 \
    }									\
  }


static void SSP_ISR(void) __attribute__((naked));

/* SSPCR0 settings */
#define SSP_DDS  0x07 << 0  /* data size         : 8 bits                    */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI                       */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : first clock transition    */
#define SSP_CPHA 0x01 << 7  /* clock phase       : SCK idles high            */
#define SSP_SCR  0x01 << 8  /* serial clock rate : divide by 2               */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE  0x00 << 1  /* SSP enable        : disabled                  */
#define SSP_MS   0x01 << 2  /* master slave mode : slave                     */
#define SSP_SOD  0x00 << 3  /* slave output disable : no                     */

#define SSPCR0_VAL (SSP_DDS |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR )
#define SSPCR1_VAL (SSP_LBM |  SSP_SSE | SSP_MS | SSP_SOD )

#define SSP_PINSEL1_SCK  (2<<2)
#define SSP_PINSEL1_MISO (2<<4)
#define SSP_PINSEL1_MOSI (2<<6)
#define SSP_PINSEL1_SSEL (2<<8)

void booz_link_mcu_hw_init ( void ) {

  /* setup pins for SSP (SCK, MISO, MOSI, SSEL) */
  PINSEL1 |= SSP_PINSEL1_SCK  | SSP_PINSEL1_MISO | 
             SSP_PINSEL1_MOSI | SSP_PINSEL1_SSEL ;

  /* setup SSP */
  SSPCR0 = SSPCR0_VAL;;
  SSPCR1 = SSPCR1_VAL;
  SSPCPSR = 0x10;
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT( VIC_SPI1 );  /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT( VIC_SPI1 );    /* enable it            */
  VICVectCntl9 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr9 = (uint32_t)SSP_ISR;     /* address of the ISR   */

  link_mcu_rx_buf  = (uint8_t*)&inter_mcu_state;
  link_mcu_tx_buf = (uint8_t*)&booz_link_mcu_state_unused;

  BoozLinkMcuHwRestart();
}

static void SSP_ISR(void) {
 ISR_ENTRY();

 it_cnt++;

 /*  Rx half full  */
 if (bit_is_set(SSPMIS, RXMIS)) { 
   rx_it_cnt++;
   /* LinkMcuTransmit(); */
   LinkMcuReceive();
   SSP_EnableRti();
 }

 /* Rx timeout     */ 
 if ( bit_is_set(SSPMIS, RTMIS)) {
   ti_it_cnt++;
   LinkMcuReceive();
   SSP_ClearRti();
   SSP_DisableRti();
   SSP_Disable();
   booz_link_mcu_status = BOOZ_LINK_MCU_DATA_AVAILABLE;
 }

 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}

#endif /* BOOZ_CONTROLLER_MCU */
