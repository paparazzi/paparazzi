/*
 * Copyright (C) 2010 Martin Mueller
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

/** \file max11040.c
 *  \brief Maxim MAX11040 ADC hw interface
 *
 *  SS    on P0.20 (SSEL)
 *  DRDY  on P0.16 (EINT0)
 */

#include "armVIC.h"
#include "max11040_hw.h"
#include "adcs/max11040.h"

#ifdef LOGGER
extern unsigned int getclock(void);
#endif

volatile uint8_t num_irqs = 0;

static void SSP_ISR(void) __attribute__((naked));
static void EXTINT_ISR(void) __attribute__((naked));

#warning "This driver should be updated to use the new SPI peripheral"

#ifndef SPI1_VIC_SLOT
#define SPI1_VIC_SLOT 7
#endif

static void SSP_ISR(void)
{
  int i;
  ISR_ENTRY();

  switch (max11040_status) {

    case MAX11040_RESET: {
      /* read dummy control byte reply */
      uint8_t foo __attribute__((unused));
      foo = SSPDR;
      foo = SSPDR;
      /* write configuration register */
      SSP_Send(0x60);       /* wr conf */
      SSP_Send(0x30);       /* adc0: en24bit, xtalen, no faultdis */
      for (i = 1; i < MAXM_NB_ADCS; i++) {
        SSP_Send(0x20);     /* adcx: en24bit, no xtalen, no faultdis */
      }
      max11040_status = MAX11040_CONF;
      SSP_ClearRti();
    }
    break;

    case MAX11040_CONF: {
      /* read dummy control byte reply */
      uint8_t foo __attribute__((unused));
      foo = SSPDR;
      for (i = 0; i < MAXM_NB_ADCS; i++) {
        foo = SSPDR;
      }
      /* write sampling instant register */
      SSP_Send(0x40);       /* wr instant */
      for (i = 0; i < MAXM_NB_ADCS; i++) {
        SSP_Send(0);        /* adcx: no delay */
        SSP_Send(0);
        SSP_Send(0);
        SSP_Send(0);
      }
      max11040_status = MAX11040_INSTANT;
      SSP_ClearRti();
    }
    break;

    case MAX11040_INSTANT: {
      /* read dummy control byte reply */
      uint8_t foo __attribute__((unused));
      foo = SSPDR;
      for (i = 0; i < MAXM_NB_ADCS; i++) {
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
      }
      /* write data rate control register */
      SSP_Send(0x50);    /* wr rate */
      SSP_Send(0x26);    /* adc: 250.1 sps */
      SSP_Send(0x00);
      max11040_status = MAX11040_RATE;
      SSP_ClearRti();
    }
    break;

    case MAX11040_RATE: {
      uint8_t foo __attribute__((unused));
      foo = SSPDR;
      foo = SSPDR;
      foo = SSPDR;
      /* read data register */
      SSP_Send(0xF0);       /* rd data */
      for (i = 0; i < MAXM_NB_ADCS; i++) {
        SSP_Send(0x00);     /* adcx: data */
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
      }
      max11040_status = MAX11040_DATA;
      SSP_ClearRti();
    }
    break;

    case MAX11040_DATA: {
      uint8_t foo __attribute__((unused));
      foo = SSPDR;
      for (i = 0; i < MAXM_NB_ADCS; i++) {
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
        foo = SSPDR;
      }

      /* read data */
      /* read data register */
      SSP_Send(0xF0);       /* rd data */
      for (i = 0; i < MAXM_NB_ADCS; i++) {
        SSP_Send(0x00);     /* adc0: data */
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
      }

      SSP_ClearRti();
    }
    break;

    case MAX11040_DATA2: {
      uint8_t foo __attribute__((unused));

      SSP_ClearRti();
      SSP_ClearRxi();

      if (max11040_count <= MAXM_NB_CHAN + 2) {
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
        SSP_Send(0x00);
      }

      if (max11040_count == 0) { foo = SSPDR; }

      max11040_values[max11040_buf_in][max11040_count]  = SSPDR << 16;
      max11040_values[max11040_buf_in][max11040_count] |= SSPDR << 8;
      max11040_values[max11040_buf_in][max11040_count] |= SSPDR;
      if (max11040_values[max11040_buf_in][max11040_count] & 0x800000) {
        max11040_values[max11040_buf_in][max11040_count] |= 0xFF000000;
      }

      max11040_count++;

      max11040_values[max11040_buf_in][max11040_count]  = SSPDR << 16;
      max11040_values[max11040_buf_in][max11040_count] |= SSPDR << 8;
      max11040_values[max11040_buf_in][max11040_count] |= SSPDR;
      if (max11040_values[max11040_buf_in][max11040_count] & 0x800000) {
        max11040_values[max11040_buf_in][max11040_count] |= 0xFF000000;
      }

      max11040_count++;

      if (max11040_count == MAXM_NB_CHAN) {
        MaxmUnselect();
        max11040_data = MAX11040_DATA_AVAILABLE;
        i = max11040_buf_in + 1;
        if (i >= MAX11040_BUF_SIZE) { i = 0; }
        if (i != max11040_buf_out) {
          max11040_buf_in = i;
        } else {
          //throw error;
        }
      }
    }
    break;

  }

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

void EXTINT_ISR(void)
{
  ISR_ENTRY();

  if (num_irqs++ == 5) {
    /* switch SSEL P0.20 to be used as GPIO */
    PINSEL1 &= ~(3 << 8);
    IO0DIR |= 1 << 20;
    max11040_status = MAX11040_DATA2;
  }

  if (max11040_status == MAX11040_DATA2) {

#ifdef LOGGER
    max11040_timestamp[max11040_buf_in] = getclock();
#endif

    MaxmSelect();

    /* read data */
    SSP_Send(0xF0);
    SSP_Send(0x00);
    SSP_Send(0x00);
    SSP_Send(0x00);
    SSP_Send(0x00);
    SSP_Send(0x00);
    SSP_Send(0x00);

    max11040_count = 0;
  }

  /* clear EINT */
  SetBit(EXTINT, MAXM_DRDY_EINT);

  VICVectAddr = 0x00000000;    /* clear this interrupt from the VIC */
  ISR_EXIT();
}


void max11040_hw_init(void)
{
  int i;

  /* *** configure SPI ***  */
  /* setup pins for SSP (SCK, MISO, MOSI, SSEL) */
  PINSEL1 |= SSP_PINSEL1_SCK  | SSP_PINSEL1_MISO | SSP_PINSEL1_MOSI | SSP_PINSEL1_SSEL;

  /* setup SSP */
  SSPCR0 = SSPCR0_VAL;;
  SSPCR1 = SSPCR1_VAL;
  SSPCPSR = 0x02;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);           /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT(VIC_SPI1);             /* enable it            */
  _VIC_CNTL(SPI1_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
  _VIC_ADDR(SPI1_VIC_SLOT) = (uint32_t)SSP_ISR;  /* address of the ISR   */


  /* *** configure DRDY pin***  */
  /* connected pin to EXINT */
  MAXM_DRDY_PINSEL |= MAXM_DRDY_PINSEL_VAL << MAXM_DRDY_PINSEL_BIT;
  SetBit(EXTMODE, MAXM_DRDY_EINT);     /* EINT is edge trigered */
  ClearBit(EXTPOLAR, MAXM_DRDY_EINT);  /* EINT is trigered on falling edge */
  SetBit(EXTINT, MAXM_DRDY_EINT);      /* clear pending EINT */

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(MAXM_DRDY_VIC_IT);                         /* select EINT as IRQ source */
  VICIntEnable = VIC_BIT(MAXM_DRDY_VIC_IT);                           /* enable it                 */
  _VIC_CNTL(MAX11040_DRDY_VIC_SLOT) = VIC_ENABLE | MAXM_DRDY_VIC_IT;
  _VIC_ADDR(MAX11040_DRDY_VIC_SLOT) = (uint32_t)EXTINT_ISR;           /* address of the ISR        */


  /* write configuration register */
  SSP_Send(0x60);       /* wr conf */
  for (i = 0; i < MAXM_NB_ADCS; i++) {
    SSP_Send(0x40);     /* adcx: reset */
  }
  SSP_Enable();
  SSP_ClearRti();
  SSP_EnableRti();
}

