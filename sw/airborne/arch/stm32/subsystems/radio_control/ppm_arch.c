/*
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

/**
 * @file arch/stm32/subsystems/radio_control/ppm_arch.c
 * @ingroup stm32_arch
 *
 * STM32 ppm decoder.
 *
 * Input signal either on:
 *  - PA1 TIM2/CH2 (uart1 trig on Lisa/L)  (Servo 6 on Lisa/M)
 *  - PA10 TIM1/CH3 (uart1 trig on Lisa/L) (uart1 rx on Lisa/M)
 *
 */

#include "subsystems/radio_control.h"
#include "subsystems/radio_control/ppm.h"

#include BOARD_CONFIG

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "mcu_periph/sys_time.h"
#include "mcu_periph/gpio.h"


#define ONE_MHZ_CLK 1000000

uint8_t  ppm_cur_pulse;
uint32_t ppm_last_pulse_time;
bool_t   ppm_data_valid;
static uint32_t timer_rollover_cnt;

/*
 * Timer clock frequency (before prescaling) is twice the frequency
 * of the APB domain to which the timer is connected.
 */

#ifdef STM32F1
/**
 * HCLK = 72MHz, Timer clock also 72MHz since
 * TIM1_CLK = APB2 = 72MHz
 * TIM2_CLK = 2 * APB1 = 2 * 32MHz
 */
#define PPM_TIMER_CLK       AHB_CLK
#endif

#if USE_PPM_TIM2

PRINT_CONFIG_MSG("Using TIM2 for PPM input.")

#define PPM_RCC             &RCC_APB1ENR
#define PPM_PERIPHERAL      RCC_APB1ENR_TIM2EN
#define PPM_TIMER           TIM2

#ifdef STM32F4
/* Since APB prescaler != 1 :
 * Timer clock frequency (before prescaling) is twice the frequency
 * of the APB domain to which the timer is connected.
 */
#define PPM_TIMER_CLK       (rcc_ppre1_frequency * 2)
#endif

#elif USE_PPM_TIM1

PRINT_CONFIG_MSG("Using TIM1 for PPM input.")

#define PPM_RCC             &RCC_APB2ENR
#define PPM_PERIPHERAL      RCC_APB2ENR_TIM1EN
#define PPM_TIMER           TIM1

#ifdef STM32F4
#define PPM_TIMER_CLK       (rcc_ppre2_frequency * 2)
#endif

#else
#error Unknown PPM input timer configuration.
#endif

void ppm_arch_init ( void ) {

  /* timer clock enable */
  rcc_peripheral_enable_clock(PPM_RCC, PPM_PERIPHERAL);

  /* GPIO clock enable */
  gpio_enable_clock(PPM_GPIO_PORT);

  /* timer gpio configuration */
  gpio_setup_pin_af(PPM_GPIO_PORT, PPM_GPIO_PIN, PPM_GPIO_AF, FALSE);

  /* Time Base configuration */
  timer_reset(PPM_TIMER);
  timer_set_mode(PPM_TIMER, TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_period(PPM_TIMER, 0xFFFF);
  timer_set_prescaler(PPM_TIMER, (PPM_TIMER_CLK / (RC_PPM_TICKS_PER_USEC*ONE_MHZ_CLK)) - 1);

 /* TIM configuration: Input Capture mode ---------------------
     The Rising edge is used as active edge
  ------------------------------------------------------------ */
#if defined PPM_PULSE_TYPE && PPM_PULSE_TYPE == PPM_PULSE_TYPE_POSITIVE
  timer_ic_set_polarity(PPM_TIMER, PPM_CHANNEL, TIM_IC_RISING);
#elif defined PPM_PULSE_TYPE && PPM_PULSE_TYPE == PPM_PULSE_TYPE_NEGATIVE
  timer_ic_set_polarity(PPM_TIMER, PPM_CHANNEL, TIM_IC_FALLING);
#else
#error "Unknown PPM_PULSE_TYPE"
#endif
  timer_ic_set_input(PPM_TIMER, PPM_CHANNEL, PPM_TIMER_INPUT);
  timer_ic_set_prescaler(PPM_TIMER, PPM_CHANNEL, TIM_IC_PSC_OFF);
  timer_ic_set_filter(PPM_TIMER, PPM_CHANNEL, TIM_IC_OFF);

  /* Enable timer Interrupt(s). */
  nvic_set_priority(PPM_IRQ, 2);
  nvic_enable_irq(PPM_IRQ);

#ifdef PPM_IRQ2
  nvic_set_priority(PPM_IRQ2, 2);
  nvic_enable_irq(PPM_IRQ2);
#endif

  /* Enable the Capture/Compare and Update interrupt requests. */
  timer_enable_irq(PPM_TIMER, (PPM_CC_IE | TIM_DIER_UIE));

  /* Enable capture channel. */
  timer_ic_enable(PPM_TIMER, PPM_CHANNEL);

  /* TIM enable counter */
  timer_enable_counter(PPM_TIMER);

  ppm_last_pulse_time = 0;
  ppm_cur_pulse = RADIO_CONTROL_NB_CHANNEL;
  timer_rollover_cnt = 0;

}

#if USE_PPM_TIM2

void tim2_isr(void) {

  if((TIM2_SR & PPM_CC_IF) != 0) {
    timer_clear_flag(TIM2, PPM_CC_IF);

    uint32_t now = timer_get_counter(TIM2) + timer_rollover_cnt;
    DecodePpmFrame(now);
  }
  else if((TIM2_SR & TIM_SR_UIF) != 0) {
    timer_rollover_cnt+=(1<<16);
    timer_clear_flag(TIM2, TIM_SR_UIF);
  }

}

#elif USE_PPM_TIM1

#if defined(STM32F1)
void tim1_up_isr(void) {
#elif defined(STM32F4)
void tim1_up_tim10_isr(void) {
#endif
  if((TIM1_SR & TIM_SR_UIF) != 0) {
    timer_rollover_cnt+=(1<<16);
    timer_clear_flag(TIM1, TIM_SR_UIF);
  }
}

void tim1_cc_isr(void) {
  if((TIM1_SR & PPM_CC_IF) != 0) {
    timer_clear_flag(TIM1, PPM_CC_IF);

    uint32_t now = timer_get_counter(TIM1) + timer_rollover_cnt;
    DecodePpmFrame(now);
  }
}

#endif /* USE_PPM_TIM1 */

