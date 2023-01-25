/*
 * Copyright (C) 2010-2014 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file arch/stm32/modules/radio_control/ppm_arch.c
 * @ingroup stm32_arch
 *
 * STM32 ppm decoder.
 *
 * Input signal either on:
 *  - PA1 TIM2/CH2 (uart1 trig on Lisa/L)  (Servo 6 on Lisa/M)
 *  - PA10 TIM1/CH3 (uart1 trig on Lisa/L) (uart1 rx on Lisa/M)
 *
 */

#include "modules/radio_control/radio_control.h"
#include "modules/radio_control/ppm.h"

#include BOARD_CONFIG

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "mcu_periph/gpio.h"

// for timer_get_frequency
#include "mcu_arch.h"

#define ONE_MHZ_CLK 1000000
#ifdef NVIC_TIM_IRQ_PRIO
#define PPM_IRQ_PRIO  NVIC_TIM_IRQ_PRIO
#else
#define PPM_IRQ_PRIO 2
#endif


static uint32_t timer_rollover_cnt;


#if USE_PPM_TIM1

PRINT_CONFIG_MSG("Using TIM1 for PPM input.")
#define PPM_TIMER           TIM1
#define RCC_TIM_PPM         RCC_TIM1
#define RST_TIM_PPM         RST_TIM1

#elif USE_PPM_TIM2

PRINT_CONFIG_MSG("Using TIM2 for PPM input.")
#define PPM_TIMER           TIM2
#define RCC_TIM_PPM         RCC_TIM2
#define RST_TIM_PPM         RST_TIM2

#elif USE_PPM_TIM3

PRINT_CONFIG_MSG("Using TIM3 for PPM input.")
#define PPM_TIMER           TIM3
#define RCC_TIM_PPM         RCC_TIM3
#define RST_TIM_PPM         RST_TIM3

#elif USE_PPM_TIM4

PRINT_CONFIG_MSG("Using TIM4 for PPM input.")
#define PPM_TIMER           TIM4
#define RCC_TIM_PPM         RCC_TIM4
#define RST_TIM_PPM         RST_TIM4

#elif USE_PPM_TIM5

PRINT_CONFIG_MSG("Using TIM5 for PPM input.")
#define PPM_TIMER           TIM5
#define RCC_TIM_PPM         RCC_TIM5
#define RST_TIM_PPM         RST_TIM5

#elif USE_PPM_TIM8

PRINT_CONFIG_MSG("Using TIM8 for PPM input.")
#define PPM_TIMER           TIM8
#define RCC_TIM_PPM         RCC_TIM8
#define RST_TIM_PPM         RST_TIM8

#elif USE_PPM_TIM9

PRINT_CONFIG_MSG("Using TIM9 for PPM input.")
#define PPM_TIMER           TIM9
#define RCC_TIM_PPM         RCC_TIM9
#define RST_TIM_PPM         RST_TIM9

#elif USE_PPM_TIM12

PRINT_CONFIG_MSG("Using TIM12 for PPM input.")
#define PPM_TIMER           TIM12
#define RCC_TIM_PPM         RCC_TIM12
#define RST_TIM_PPM         RST_TIM12

#else
#error Unknown PPM input timer configuration.
#endif

void ppm_arch_init(void)
{
  /* timer clock enable */
  rcc_periph_clock_enable(RCC_TIM_PPM);

  /* GPIO configuration as input capture for timer */
  gpio_setup_pin_af(PPM_GPIO_PORT, PPM_GPIO_PIN, PPM_GPIO_AF, FALSE);

  /* Time Base configuration */
  rcc_periph_reset_pulse(RST_TIM_PPM);
  timer_set_mode(PPM_TIMER, TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_period(PPM_TIMER, 0xFFFF);
  uint32_t timer_clk = timer_get_frequency(PPM_TIMER);
  timer_set_prescaler(PPM_TIMER, (timer_clk / (RC_PPM_TICKS_PER_USEC * ONE_MHZ_CLK)) - 1);

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
  nvic_set_priority(PPM_IRQ, PPM_IRQ_PRIO);
  nvic_enable_irq(PPM_IRQ);

#ifdef PPM_IRQ2
  nvic_set_priority(PPM_IRQ2, PPM_IRQ_PRIO);
  nvic_enable_irq(PPM_IRQ2);
#endif

  /* Enable the Capture/Compare and Update interrupt requests. */
  timer_enable_irq(PPM_TIMER, (PPM_CC_IE | TIM_DIER_UIE));

  /* Enable capture channel. */
  timer_ic_enable(PPM_TIMER, PPM_CHANNEL);

  /* TIM enable counter */
  timer_enable_counter(PPM_TIMER);

  timer_rollover_cnt = 0;
}

#if USE_PPM_TIM2

void tim2_isr(void)
{
  if ((TIM2_SR & PPM_CC_IF) != 0) {
    timer_clear_flag(TIM2, PPM_CC_IF);

    uint32_t now = timer_get_counter(TIM2) + timer_rollover_cnt;
    ppm_decode_frame(now);
  } else if ((TIM2_SR & TIM_SR_UIF) != 0) {
    timer_rollover_cnt += (1 << 16);
    timer_clear_flag(TIM2, TIM_SR_UIF);
  }
}


#elif USE_PPM_TIM3

void tim3_isr(void)
{
  if ((TIM3_SR & PPM_CC_IF) != 0) {
    timer_clear_flag(TIM3, PPM_CC_IF);

    uint32_t now = timer_get_counter(TIM3) + timer_rollover_cnt;
    ppm_decode_frame(now);
  } else if ((TIM3_SR & TIM_SR_UIF) != 0) {
    timer_rollover_cnt += (1 << 16);
    timer_clear_flag(TIM3, TIM_SR_UIF);
  }
}

#elif USE_PPM_TIM4

void tim4_isr(void)
{
  if ((TIM4_SR & PPM_CC_IF) != 0) {
    timer_clear_flag(TIM4, PPM_CC_IF);

    uint32_t now = timer_get_counter(TIM4) + timer_rollover_cnt;
    ppm_decode_frame(now);
  } else if ((TIM4_SR & TIM_SR_UIF) != 0) {
    timer_rollover_cnt += (1 << 16);
    timer_clear_flag(TIM4, TIM_SR_UIF);
  }
}

#elif USE_PPM_TIM5

void tim5_isr(void)
{
  if ((TIM5_SR & PPM_CC_IF) != 0) {
    timer_clear_flag(TIM5, PPM_CC_IF);

    uint32_t now = timer_get_counter(TIM5) + timer_rollover_cnt;
    ppm_decode_frame(now);
  } else if ((TIM5_SR & TIM_SR_UIF) != 0) {
    timer_rollover_cnt += (1 << 16);
    timer_clear_flag(TIM5, TIM_SR_UIF);
  }
}


#elif USE_PPM_TIM1

#if defined(STM32F1)
void tim1_up_isr(void)
{
#elif defined(STM32F4)
void tim1_up_tim10_isr(void) {
#endif
  if ((TIM1_SR & TIM_SR_UIF) != 0) {
    timer_rollover_cnt += (1 << 16);
    timer_clear_flag(TIM1, TIM_SR_UIF);
  }
}

void tim1_cc_isr(void) {
  if ((TIM1_SR & PPM_CC_IF) != 0) {
    timer_clear_flag(TIM1, PPM_CC_IF);

    uint32_t now = timer_get_counter(TIM1) + timer_rollover_cnt;
    ppm_decode_frame(now);
  }
}

#elif USE_PPM_TIM8

#if defined(STM32F1)
void tim8_up_isr(void) {
#elif defined(STM32F4)
void tim8_up_tim13_isr(void) {
#endif
  if ((TIM8_SR & TIM_SR_UIF) != 0) {
    timer_rollover_cnt += (1 << 16);
    timer_clear_flag(TIM8, TIM_SR_UIF);
  }
}

void tim8_cc_isr(void) {
  if ((TIM8_SR & PPM_CC_IF) != 0) {
    timer_clear_flag(TIM8, PPM_CC_IF);

    uint32_t now = timer_get_counter(TIM8) + timer_rollover_cnt;
    ppm_decode_frame(now);
  }
}

#elif USE_PPM_TIM9 && defined(STM32F4)

void tim1_brk_tim9_isr(void)
{
  if ((TIM9_SR & PPM_CC_IF) != 0) {
    timer_clear_flag(TIM9, PPM_CC_IF);

    uint32_t now = timer_get_counter(TIM9) + timer_rollover_cnt;
    ppm_decode_frame(now);

  } else if ((TIM9_SR & TIM_SR_UIF) != 0) {
    timer_rollover_cnt += (1 << 16);
    timer_clear_flag(TIM9, TIM_SR_UIF);
  }
}

#elif USE_PPM_TIM12 && defined(STM32F4)

void tim8_brk_tim12_isr(void)
{
  if ((TIM12_SR & PPM_CC_IF) != 0) {
    timer_clear_flag(TIM12, PPM_CC_IF);

    uint32_t now = timer_get_counter(TIM12) + timer_rollover_cnt;
    ppm_decode_frame(now);

  } else if ((TIM12_SR & TIM_SR_UIF) != 0) {
    timer_rollover_cnt += (1 << 16);
    timer_clear_flag(TIM12, TIM_SR_UIF);
  }
}



#endif /* USE_PPM_TIM1 */

