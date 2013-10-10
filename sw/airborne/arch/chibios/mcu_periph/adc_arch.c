/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
/**
 * @brief chibios arch dependant adc drivers
 * @note Empty for now, just to provide compatibilty
 * 		 Comments below from stm32/mcu_periph/adc_arch.c
 *
 */

/*
  For better understanding of timer and GPIO settings:

  Table of GPIO pins available per ADC:

  ADC1/2:                   ADC3:
  C0  -> PA0				C0  -> PA0
  C1  -> PA1				C1  -> PA1
  C2  -> PA2				C2  -> PA2
  C3  -> PA3				C3  -> PA3
  C4  -> PA4				C4  -> PF6
  C5  -> PA5				C5  -> PF7
  C6  -> PA6				C6  -> PF8
  C7  -> PA7				C7  -> PF9
  C8  -> PB0				C8  -> PF10
  C9  -> PB1
  C10 -> PC0				C10 -> PC0
  C11 -> PC1				C11 -> PC1
  C12 -> PC2				C12 -> PC2
  C13 -> PC3				C13 -> PC3
  C14 -> PC4
  C15 -> PC5

  Table of timers available per ADC (from libstm/src/stm32_adc.c):

  T1_TRGO:    Timer1 TRGO event (ADC1, ADC2 and ADC3)
  T1_CC4:     Timer1 capture compare4 (ADC1, ADC2 and ADC3)
  T2_TRGO:    Timer2 TRGO event (ADC1 and ADC2)
  T2_CC1:     Timer2 capture compare1 (ADC1 and ADC2)
  T3_CC4:     Timer3 capture compare4 (ADC1 and ADC2)
  T4_TRGO:    Timer4 TRGO event (ADC1 and ADC2)
  TIM8_CC4: External interrupt line 15 or Timer8 capture compare4 event (ADC1 and ADC2)
  T4_CC3:     Timer4 capture compare3 (ADC3 only)
  T8_CC2:     Timer8 capture compare2 (ADC3 only)
  T8_CC4:     Timer8 capture compare4 (ADC3 only)
  T5_TRGO:    Timer5 TRGO event (ADC3 only)
  T5_CC4:     Timer5 capture compare4 (ADC3 only)

  By setting ADC_ExternalTrigInjecConv_None, injected conversion
  is started by software instead of external trigger for any ADC.

  Table of APB per Timer (from libstm/src/stm32_tim.c):

  RCC_APB1: TIM2, TIM3, TIM4, TIM5, TIM7 (non-advanced timers)
  RCC_APB2: TIM1, TIM8 (advanced timers)

*/
#include "mcu_periph/adc.h"

void adc_buf_channel(uint8_t adc_channel,
                     struct adc_buf * s,
                     uint8_t av_nb_sample){
  //TODO
  (void) adc_channel;
  (void) s;
  (void) av_nb_sample;
}

void adc_init( void ) {
	//TODO
}
