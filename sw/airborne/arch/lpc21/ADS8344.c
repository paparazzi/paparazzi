/*
 * Copyright (C) 2008- ENAC
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

#include "ADS8344.h"
#include "LPC21xx.h"
#include "armVIC.h"
#include BOARD_CONFIG
#include "led.h"
#include "mcu_periph/spi.h"

#define ADS8344_SS_IODIR IO0DIR
#define ADS8344_SS_IOSET IO0SET
#define ADS8344_SS_IOCLR IO0CLR
#define ADS8344_SS_PIN   20

#define ADS8344Select()   SetBit(ADS8344_SS_IOCLR,ADS8344_SS_PIN)
#define ADS8344Unselect() SetBit(ADS8344_SS_IOSET,ADS8344_SS_PIN)

bool_t ADS8344_available;
uint16_t ADS8344_values[NB_CHANNELS];

#define POWER_MODE (1 << 1 | 1)
#define SGL_DIF 1 // Single ended


/* set SSP input clock, PCLK / CPSDVSR = 750kHz      */
/* SSP clock, 750kHz / (SCR+1) = 750kHz / 15 = 50kHz */

#if (PCLK == 15000000)
#define CPSDVSR    20
#else

#if (PCLK == 30000000)
#define CPSDVSR    40
#else

#if (PCLK == 60000000)
#define CPSDVSR    80
#else

#error unknown PCLK frequency
#endif
#endif
#endif

/* SSPCR0 settings */
#define SSP_DSS  0x07 << 0  /* data size            : 8 bits   */
#define SSP_FRF  0x00 << 4  /* frame format         : SPI      */
#define SSP_CPOL 0x00 << 6  /* clock polarity       : idle low */
#define SSP_CPHA 0x00 << 7  /* clock phase          : 1        */
#define SSP_SCR  0x0E << 8  /* serial clock rate    : 1MHz     */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode        : disabled */
#define SSP_SSE  0x00 << 1  /* SSP enable           : disabled */
#define SSP_MS   0x00 << 2  /* master slave mode    : master   */
#define SSP_SOD  0x00 << 3  /* slave output disable : disabled */


static void SPI1_ISR(void) __attribute__((naked));
static uint8_t channel;

#warning "This driver should be updated to use the new SPI peripheral"

#ifndef SPI1_VIC_SLOT
#define SPI1_VIC_SLOT 7
#endif

void ADS8344_init(void)
{
  channel = 0;
  ADS8344_available = FALSE;

  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= 2 << 2 | 2 << 4 | 2 << 6;

  /* setup SSP */
  SSPCR0 = SSP_DSS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = CPSDVSR; /* -> 50kHz */

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);     // SPI1 interrupt enabled
  _VIC_CNTL(SPI1_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
  _VIC_CNTL(SPI1_VIC_SLOT) = (uint32_t)SPI1_ISR;    /* address of the ISR */

  /* setup slave select */
  /* configure SS pin */
  SetBit(ADS8344_SS_IODIR,  ADS8344_SS_PIN);   /* pin is output  */
  ADS8344Unselect();                           /* pin low        */
}

static inline void read_values(void)
{
  uint8_t foo __attribute__((unused)) = SSPDR;
  uint8_t msb = SSPDR;
  uint8_t lsb = SSPDR;
  uint8_t llsb = SSPDR;
  ADS8344_values[channel] = (msb << 8 | lsb) << 1 | llsb >> 7;
}

static inline void send_request(void)
{
  uint8_t control = 1 << 7 | channel << 4 | SGL_DIF << 2 | POWER_MODE;

  SSPDR = control;
  SSPDR = 0;
  SSPDR = 0;
  SSPDR = 0;
}

void ADS8344_start(void)
{
  ADS8344Select();
  SpiClearRti();
  SpiEnableRti();
  SpiEnable();
  send_request();
}

void SPI1_ISR(void)
{
  ISR_ENTRY();
  LED_TOGGLE(2);
  read_values();
  channel++;
  if (channel > 7) {
    channel = 0;
    ADS8344_available = TRUE;
  }
  send_request();
  SpiClearRti();

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}
