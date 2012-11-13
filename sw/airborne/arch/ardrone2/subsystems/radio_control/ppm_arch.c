/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
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

#include "subsystems/radio_control.h"
#include "subsystems/radio_control/ppm.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/tim.h>
#include <stm32/misc.h>

#include "mcu_periph/sys_time.h"

/*
 *
 * This a radio control ppm driver for stm32
 * signal on PA1 TIM2/CH2 (uart1 trig on lisa/L)
 *
 */
uint8_t  ppm_cur_pulse;
uint32_t ppm_last_pulse_time;
bool_t   ppm_data_valid;
static uint32_t timer_rollover_cnt;

void tim2_irq_handler(void);

void ppm_arch_init ( void ) {

  /* TIM2 channel 2 pin (PA.01) configuration */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  /* Time Base configuration */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period        = 0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler     = 0x8;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

 /* TIM2 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM2 CH2 pin (PA.01)
     The Rising edge is used as active edge,
  ------------------------------------------------------------ */
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x00;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  /* Enable the TIM2 global Interrupt */
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM2, TIM_IT_CC2|TIM_IT_Update, ENABLE);

  ppm_last_pulse_time = 0;
  ppm_cur_pulse = RADIO_CONTROL_NB_CHANNEL;
  timer_rollover_cnt = 0;

}


void tim2_irq_handler(void) {

  if(TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

    uint32_t now = TIM_GetCapture2(TIM2) + timer_rollover_cnt;
    DecodePpmFrame(now);
  }
  else if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
    timer_rollover_cnt+=(1<<16);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }

}
