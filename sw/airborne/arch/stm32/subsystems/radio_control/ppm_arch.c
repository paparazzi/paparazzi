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

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/nvic.h>

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

void tim2_isr(void);

void ppm_arch_init ( void ) {

  /* TIM2 clock enable */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);

  /* GPIOA clock enable */
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);

  /* TIM2 channel 2 pin (PA.01) configuration */
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_FLOAT, GPIO1);

  /* Time Base configuration */
  timer_reset(TIM2);
  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
		 TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_period(TIM2, 0xFFFF);
  timer_set_prescaler(TIM2, 0x8);

 /* TIM2 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM2 CH2 pin (PA.01)
     The Rising edge is used as active edge,
  ------------------------------------------------------------ */
  timer_ic_set_polarity(TIM2, TIM_IC2, TIM_IC_RISING);
  timer_ic_set_input(TIM2, TIM_IC2, TIM_IC_IN_TI2);
  timer_ic_set_prescaler(TIM2, TIM_IC2, TIM_IC_PSC_OFF);
  timer_ic_set_filter(TIM2, TIM_IC2, TIM_IC_OFF);

  /* Enable the TIM2 global Interrupt. */
  nvic_set_priority(NVIC_TIM2_IRQ, 2);
  nvic_enable_irq(NVIC_TIM2_IRQ);

  /* Enable the CC2 and Update interrupt requests. */
  timer_enable_irq(TIM2, TIM_DIER_CC2IE | TIM_DIER_UIE);

  /* Enable capture channel. */
  timer_ic_enable(TIM2, TIM_IC2);

  /* TIM2 enable counter */
  timer_enable_counter(TIM2);

  ppm_last_pulse_time = 0;
  ppm_cur_pulse = RADIO_CONTROL_NB_CHANNEL;
  timer_rollover_cnt = 0;

}


void tim2_isr(void) {

  if((TIM2_SR & TIM_SR_CC2IF) != 0) {
    timer_clear_flag(TIM2, TIM_SR_CC2IF);

    uint32_t now = timer_get_counter(TIM2) + timer_rollover_cnt;
    DecodePpmFrame(now);
  }
  else if((TIM2_SR & TIM_SR_UIF) != 0) {
    timer_rollover_cnt+=(1<<16);
    timer_clear_flag(TIM2, TIM_SR_UIF);
  }

}
