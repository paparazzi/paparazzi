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
#include "armVIC.h"
#include "lcd_dogm_hw.h"


static void SPI1_ISR(void) __attribute__((naked));

#define PINSEL1_SCK  (2 << 2)
#define PINSEL1_MOSI (2 << 6)
#define PINSEL1_SSEL (2 << 8)

/* SSPCR0 settings */
#define SSP_DSS  0x07 << 0  /* data size            : 8 bits    */
#define SSP_FRF  0x00 << 4  /* frame format         : SPI       */
#define SSP_CPOL 0x01 << 6  /* clock polarity       : idle high */
#define SSP_CPHA 0x01 << 7  /* clock phase          : low->high */
#define SSP_SCR  0x1F << 8  /* serial clock rate    : 29.3kHz, SSP input clock / 16 */

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

void lcd_spi_tx(uint8_t data)
{
  SpiClearRti();
  SpiEnableRti();
  SpiEnable();
  SSPDR = data;
}

void lcd_dogm_init_hw(void)
{
  /* setup pins for SSP (SCK, MOSI) */
  PINSEL1 |= PINSEL1_SCK | PINSEL1_MOSI;

  /* setup SSP */
  SSPCR0 = SSP_DSS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = SSPCPSR_VAL; /* Prescaler */

  /* SS, RS pin is output */
  SetBit(LCDDOGM_SS_IODIR, LCDDOGM_SS_PIN);
  SetBit(LCDDOGM_RS_IODIR, LCDDOGM_RS_PIN);
  /* unselected lcd */
  lcddogmUnselect();

  /* Configure interrupt vector for SPI */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT(VIC_SPI1);     /* SPI1 interrupt enabled */
  _VIC_CNTL(SPI1_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
  _VIC_CNTL(SPI1_VIC_SLOT) = (uint32_t)SPI1_ISR;    /* address of the ISR */
}

void SPI1_ISR(void)
{
  ISR_ENTRY();

  while (bit_is_set(SSPSR, RNE)) {
    uint16_t foo __attribute__((unused));
    foo = SSPDR;
  }
  SpiClearRti();                  /* clear interrupt */
  SpiDisableRti();
  SpiDisable();
  lcddogmUnselect();

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

