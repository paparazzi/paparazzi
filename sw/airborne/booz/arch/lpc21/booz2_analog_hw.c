/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 */

#include "booz2_analog.h"

#include "booz2_analog_baro.h"
#include "booz2_battery.h"

#ifndef USE_EXTRA_ADC
/* Default mode
 * Bust OFF
 * Only one ADC can be read on each bank
 * Baro and Bat are read on interrupt
 */

#include "armVIC.h"
#include "sys_time.h"

void ADC0_ISR ( void ) __attribute__((naked));
void ADC1_ISR ( void ) __attribute__((naked));

void booz2_analog_init_hw( void ) {

  /* start ADC0 */
  /* select P0.29 as AD0.2 for bat meas*/
  PINSEL1 |=  0x01 << 26;
  /* sample AD0.2 - PCLK/4 ( 3.75MHz) - ON */
  AD0CR = 1 << 2 | 0x03 << 8 | 1 << 21;
  /* AD0 selected as IRQ */
  VICIntSelect &= ~VIC_BIT(VIC_AD0);
  /* AD0 interrupt enabled */
  VICIntEnable = VIC_BIT(VIC_AD0);
  /* AD0 interrupt as VIC2 */
  _VIC_CNTL(ADC0_VIC_SLOT) = VIC_ENABLE | VIC_AD0;
  _VIC_ADDR(ADC0_VIC_SLOT) = (uint32_t)ADC0_ISR;
  /* start convertion on T0M1 match */
  AD0CR |= 4 << 24;


 /* clear match 1 */
  T0EMR &= ~TEMR_EM1;
  /* set high on match 1 */
  T0EMR |= TEMR_EMC1_2;
  /* first match in a while */
  T0MR1 = 1024;


  /* start ADC1 */
  /* select P0.10 as AD1.2 for baro*/
  ANALOG_BARO_PINSEL |=  ANALOG_BARO_PINSEL_VAL << ANALOG_BARO_PINSEL_BIT;
  /* sample AD1.2 - PCLK/4 ( 3.75MHz) - ON */
  AD1CR = 1 << 2 | 0x03 << 8 | 1 << 21;
  /* AD0 selected as IRQ */
  VICIntSelect &= ~VIC_BIT(VIC_AD1);
  /* AD0 interrupt enabled */
  VICIntEnable = VIC_BIT(VIC_AD1);
  /* AD0 interrupt as VIC2 */
  _VIC_CNTL(ADC1_VIC_SLOT) = VIC_ENABLE | VIC_AD1;
  _VIC_ADDR(ADC1_VIC_SLOT) = (uint32_t)ADC1_ISR;
  /* start convertion on T0M3 match */
  AD1CR |= 5 << 24;


  /* clear match 2 */
  T0EMR &= ~TEMR_EM3;
  /* set high on match 2 */
  T0EMR |= TEMR_EMC3_2;
  /* first match in a while */
  T0MR3 = 512;

  /* turn on DAC pins */
  PINSEL1 |= 2 << 18;
}


void ADC0_ISR ( void ) {
  ISR_ENTRY();
  uint32_t tmp = AD0GDR;
  uint16_t tmp2 = (uint16_t)(tmp >> 6) & 0x03FF;
  Booz2BatteryISRHandler(tmp2);
  /* trigger next convertion */
  T0MR1 += BOOZ2_ANALOG_BATTERY_PERIOD;
  /* lower clock         */
  T0EMR &= ~TEMR_EM1;
  VICVectAddr = 0x00000000;                 // clear this interrupt from the VIC
  ISR_EXIT();                               // recover registers and return
}

void ADC1_ISR ( void ) {
  ISR_ENTRY();
  uint32_t tmp = AD1GDR;
  uint16_t tmp2 = (uint16_t)(tmp >> 6) & 0x03FF;
  Booz2BaroISRHandler(tmp2);
  /* trigger next convertion */
  T0MR3 += BOOZ2_ANALOG_BARO_PERIOD;
  /* lower clock         */
  T0EMR &= ~TEMR_EM3;
  VICVectAddr = 0x00000000;                 // clear this interrupt from the VIC
  ISR_EXIT();                               // recover registers and return
}

#else // USE_EXTRA_ADC
/* Extra ADCs are read
 * Bust ON
 * Baro and Bat values are updated by hand
 * Four ADCs can be configured
 * ADC_1 is available on the cam connector
 */

#include "LPC21xx.h"
#include "sys_time.h"

uint16_t booz2_adc_1; 
uint16_t booz2_adc_2; 
uint16_t booz2_adc_3; 
uint16_t booz2_adc_4; 

void booz2_analog_init_hw( void ) {

  /* AD0 */
  /* PCLK/4 ( 3.75MHz) - BURST - ON */
  AD0CR = 0x03 << 8 | 1 << 16 | 1 << 21;
  /* disable global interrupt */
  ClearBit(AD0INTEN,8);

  /* AD1 */
  /* PCLK/4 ( 3.75MHz) - BURST - ON */
  AD1CR = 0x03 << 8 | 1 << 16 | 1 << 21;
  /* disable global interrupt */
  ClearBit(AD1INTEN,8);

  /* select P0.29 as AD0.2 for bat meas*/
  PINSEL1 |=  0x01 << 26;
  /* sample AD0.2 */
  AD0CR |= 1 << 2;


  /* select P0.10 as AD1.2 for baro*/
  ANALOG_BARO_PINSEL |=  ANALOG_BARO_PINSEL_VAL << ANALOG_BARO_PINSEL_BIT;
  /* sample AD1.2 */
  AD1CR |= 1 << 2;
  /* turn on DAC pins */
  PINSEL1 |= 2 << 18;

#ifdef USE_ADC_1
  /* select P0.13 as AD1.4 adc 1 */
  PINSEL0 |= 0x03 << 26;
  AD1CR |= 1 << 4;
#endif
#ifdef USE_ADC_2
  /* select P0.4 as AD0.6 adc 2 */
  PINSEL0 |= 0x03 << 8;
  AD0CR |= 1 << 6;
#endif
#ifdef USE_ADC_3
  /* select P0.5 as AD0.7 adc 3 */
  PINSEL0 |= 0x03 << 10;
  AD0CR |= 1 << 7;
#endif
#ifdef USE_ADC_4
  /* select P0.6 as AD1.0 adc 4 */
  PINSEL0 |= 0x03 << 12;
  AD1CR |= 1 << 0;
#endif

  booz2_adc_1 = 0; 
  booz2_adc_2 = 0; 
  booz2_adc_3 = 0; 
  booz2_adc_4 = 0; 
}

void booz2_analog_baro_read(void) {
  uint32_t tmp = AD1DR2;
  uint16_t tmp2 = (uint16_t)(tmp >> 6) & 0x03FF;
  Booz2BaroISRHandler(tmp2);
}

void booz2_analog_bat_read(void) {
  uint32_t tmp = AD0DR2;
  uint16_t tmp2 = (uint16_t)(tmp >> 6) & 0x03FF;
  Booz2BatteryISRHandler(tmp2);
}

void booz2_analog_extra_adc_read(void) {
  uint32_t tmp,tmp2;
#ifdef USE_ADC_1
  tmp = AD1DR4;
  tmp2 = (uint16_t)(tmp >> 6) & 0x03FF;
  booz2_adc_1 = tmp2;
#endif
#ifdef USE_ADC_2
  tmp = AD0DR6;
  tmp2 = (uint16_t)(tmp >> 6) & 0x03FF;
  booz2_adc_2 = tmp2;
#endif
#ifdef USE_ADC_3
  tmp = AD0DR7;
  tmp2 = (uint16_t)(tmp >> 6) & 0x03FF;
  booz2_adc_3 = tmp2;
#endif
#ifdef USE_ADC_4
  tmp = AD1DR0;
  tmp2 = (uint16_t)(tmp >> 6) & 0x03FF;
  booz2_adc_4 = tmp2;
#endif
}

#endif

