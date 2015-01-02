/*
 * Copyright (C) 2008  Pascal Brisset, Antoine Drouin
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
 * @file arch/lpc21/mcu_periph/adc_arch.c
 * @ingroup lpc21_arch
 *
 * Handling of ADC hardware for lpc21xx.
 */

#include "mcu_periph/adc.h"

#include "LPC21xx.h"
#include "armVIC.h"
#include BOARD_CONFIG


#ifdef USE_AD0
#ifndef AD0_VIC_SLOT
#define AD0_VIC_SLOT 2
#endif
#endif

#ifdef USE_AD1
#ifndef AD1_VIC_SLOT
#define AD1_VIC_SLOT 4
#endif
#endif


/** First NB_ADC for bank 0, others for bank 2 */
static struct adc_buf *buffers[NB_ADC * 2];

volatile uint16_t adc0_val[NB_ADC] = {1,  2,  3,  4,  5,  6,  7,  8};
volatile uint16_t adc1_val[NB_ADC] = {9, 10, 11, 12, 13, 14, 15, 16};

void adcISR0(void) __attribute__((naked));
void adcISR1(void) __attribute__((naked));

void adc_buf_channel(uint8_t adc_channel, struct adc_buf *s, uint8_t av_nb_sample)
{
  buffers[adc_channel] = s;
  s->av_nb_sample = av_nb_sample;
}

/*

pin11 AD0.0  P0.27   PINSEL1 1 << 22
pin13 AD0.1  P0.28   PINSEL1 1 << 24
pin14 AD0.2  P0.29   PINSEL1 1 << 26
pin15 AD0.3  P0.30   PINSEL1 1 << 28
pin9  AD0.4  P0.25   PINSEL1 1 << 18
pin10 AD0.5  P0.26   PINSEL1 1 << 20
pin27 AD0.6  P0.4    PINSEL0 3 <<  8
pin29 AD0.7  P0.5    PINSEL0 3 << 10

pin30 AD1.0  P0.6    PINSEL0 3 << 12
pin33 AD1.1  P0.8    PINSEL0 3 << 16
pin35 AD1.2  P0.10   PINSEL0 3 << 20
pin38 AD1.3  P0.12   PINSEL0 3 << 24
pin39 AD1.4  P0.13   PINSEL0 3 << 26
pin45 AD1.5  P0.15   PINSEL0 3 << 30
pin1  AD1.6  P0.21   PINSEL1 2 << 10
pin2  AD1.7  P0.22   PINSEL1 1 << 12

*/

static const uint32_t ADC_PINSEL0_ONES = 0
#if defined USE_AD0_6
    | 3 << 8
#endif
#if defined USE_AD0_7
    | 3 << 10
#endif
#if defined USE_AD1_0
    | 3 << 12
#endif
#if defined USE_AD1_1
    | 3 << 16
#endif
#if defined USE_AD1_2
    | 3 << 20
#endif
#if defined USE_AD1_3
    | 3 << 24
#endif
#if defined USE_AD1_4
    | 3 << 26
#endif
#if defined USE_AD1_5
    | 3 << 30
#endif
    ;

static const uint32_t ADC_PINSEL1_ONES = 0
#if defined USE_AD0_0
    | 1 << 22
#endif
#if defined USE_AD0_1
    | 1 << 24
#endif
#if defined USE_AD0_2
    | 1 << 26
#endif
#if defined USE_AD0_3
    | 1 << 28
#endif
#if defined USE_AD0_4
    | 1 << 18
#endif
#if defined USE_AD0_5
    | 1 << 20
#endif
#if defined USE_AD1_6
    | 2 << 10
#endif
#if defined USE_AD1_7
    | 1 << 12
#endif
    ;

static const uint32_t ADC_AD0CR_SEL_HW_SCAN = 0
#if defined USE_AD0_0
    | 1 << 0
#endif
#if defined USE_AD0_1
    | 1 << 1
#endif
#if defined USE_AD0_2
    | 1 << 2
#endif
#if defined USE_AD0_3
    | 1 << 3
#endif
#if defined USE_AD0_4
    | 1 << 4
#endif
#if defined USE_AD0_5
    | 1 << 5
#endif
#if defined USE_AD0_6
    | 1 << 6
#endif
#if defined USE_AD0_7
    | 1 << 7
#endif
    ;

static const uint32_t ADC_AD1CR_SEL_HW_SCAN = 0
#if defined USE_AD1_0
    | 1 << 0
#endif
#if defined USE_AD1_1
    | 1 << 1
#endif
#if defined USE_AD1_2
    | 1 << 2
#endif
#if defined USE_AD1_3
    | 1 << 3
#endif
#if defined USE_AD1_4
    | 1 << 4
#endif
#if defined USE_AD1_5
    | 1 << 5
#endif
#if defined USE_AD1_6
    | 1 << 6
#endif
#if defined USE_AD1_7
    | 1 << 7
#endif
    ;

void adc_init(void)
{

  /* connect pins for selected ADCs */
  PINSEL0 |= ADC_PINSEL0_ONES;
  PINSEL1 |= ADC_PINSEL1_ONES;

#ifdef USE_AD0
  /* FIXME: this needs to be investigated, we should run just below 4.5MHz,
      but we are a lot slower (e.g. 58.6kHz with PCLK = 15MHz), see
      lpc_vor_convertions.c for right timing code */
  /* setup hw scan - PCLK/256 ( 58.6kHz/117.2kHz/234.4kHz ) - BURST ON */
  AD0CR = ADC_AD0CR_SEL_HW_SCAN | 0xFF << 8 | 1 << 16 | 0x01 << 21 ;
  /* AD0 selected as IRQ */
  VICIntSelect &= ~VIC_BIT(VIC_AD0);
  /* AD0 interrupt enabled */
  VICIntEnable = VIC_BIT(VIC_AD0);
  /* AD0 interrupt as VIC2 */
  _VIC_CNTL(AD0_VIC_SLOT) = VIC_ENABLE | VIC_AD0;
  _VIC_ADDR(AD0_VIC_SLOT) = (uint32_t)adcISR0;
#endif

#ifdef USE_AD1
  /* FIXME: this needs to be investigated, we should run just below 4.5MHz,
      but we are a lot slower (e.g. 58.6kHz with PCLK = 15MHz), see
      lpc_vor_convertions.c for right timing code */
  /* setup hw scan - PCLK/256 ( 58.6kHz/117.2kHz/234.4kHz ) - BURST ON */
  AD1CR = ADC_AD1CR_SEL_HW_SCAN | 0xFF << 8 | 1 << 16 | 0x01 << 21 ;
  /* AD1 selected as IRQ */
  VICIntSelect &= ~VIC_BIT(VIC_AD1);
  /* AD1 interrupt enabled */
  VICIntEnable = VIC_BIT(VIC_AD1);
  /* AD1 interrupt as VIC2 */
  _VIC_CNTL(AD1_VIC_SLOT) = VIC_ENABLE | VIC_AD1;
  _VIC_ADDR(AD1_VIC_SLOT) = (uint32_t)adcISR1;
#endif

}


void adcISR0(void)
{
  ISR_ENTRY();
  uint32_t tmp = AD0GDR;
  uint8_t  channel = (uint8_t)(tmp >> 24) & 0x07;
  uint16_t value = (uint16_t)(tmp >> 6) & 0x03FF;
  adc0_val[channel] = value;

  struct adc_buf *buf = buffers[channel];
  if (buf) {
    uint8_t new_head = buf->head + 1;
    if (new_head >= buf->av_nb_sample) { new_head = 0; }
    buf->sum -= buf->values[new_head];
    buf->values[new_head] = value;
    buf->sum += value;
    buf->head = new_head;
  }

  VICVectAddr = 0x00000000;                 // clear this interrupt from the VIC
  ISR_EXIT();                               // recover registers and return
}

void adcISR1(void)
{
  ISR_ENTRY();
  uint32_t tmp = AD1GDR;
  uint8_t channel = (uint8_t)(tmp >> 24) & 0x07;
  uint16_t value = (uint16_t)(tmp >> 6) & 0x03FF;
  adc1_val[channel] = value;
  struct adc_buf *buf = buffers[channel + NB_ADC];
  if (buf) {
    uint8_t new_head = buf->head + 1;
    if (new_head >= buf->av_nb_sample) { new_head = 0; }
    buf->sum -= buf->values[new_head];
    buf->values[new_head] = value;
    buf->sum += value;
    buf->head = new_head;
  }

  VICVectAddr = 0x00000000;                 // clear this interrupt from the VIC
  ISR_EXIT();                               // recover registers and return
}
