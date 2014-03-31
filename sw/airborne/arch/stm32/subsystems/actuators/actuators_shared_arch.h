/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file arch/stm32/subsystems/actuators/actuators_shared_arch.h
 *  STM32 PWM and dualPWM servos shared functions.
 */

#ifndef ACTUATORS_PWM_SHARED_ARCH_H
#define ACTUATORS_PWM_SHARED_ARCH_H

#include "std.h"

#include BOARD_CONFIG

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>



#if defined(STM32F1)
//#define PCLK 72000000
#define PCLK AHB_CLK
#elif defined(STM32F4)
//#define PCLK 84000000
#define PCLK AHB_CLK/2
#endif

#define ONE_MHZ_CLK 1000000

#ifdef STM32F1
/**
 * HCLK = 72MHz, Timer clock also 72MHz since
 * TIM1_CLK = APB2 = 72MHz
 * TIM2_CLK = 2 * APB1 = 2 * 32MHz
 */
#define TIMER_APB1_CLK AHB_CLK
#define TIMER_APB2_CLK AHB_CLK
#endif

#ifdef STM32F4
/* Since APB prescaler != 1 :
 * Timer clock frequency (before prescaling) is twice the frequency
 * of the APB domain to which the timer is connected.
 */
#define TIMER_APB1_CLK (rcc_ppre1_frequency * 2)
#define TIMER_APB2_CLK (rcc_ppre2_frequency * 2)
#endif

/** Default servo update rate in Hz */
#ifndef SERVO_HZ
#define SERVO_HZ 40
#endif

// Update rate can be adapted for each timer
#ifndef TIM1_SERVO_HZ
#define TIM1_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM2_SERVO_HZ
#define TIM2_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM3_SERVO_HZ
#define TIM3_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM4_SERVO_HZ
#define TIM4_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM5_SERVO_HZ
#define TIM5_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM9_SERVO_HZ
#define TIM9_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM12_SERVO_HZ
#define TIM12_SERVO_HZ SERVO_HZ
#endif


/** @todo: these should go into libopencm3 */
#define TIM9				TIM9_BASE
#define TIM12				TIM12_BASE


#if defined(STM32F4)
extern void set_servo_gpio(uint32_t gpioport, uint16_t pin, uint8_t af_num, enum rcc_periph_clken clken);
#elif defined(STM32F1)
extern void set_servo_gpio(uint32_t gpioport, uint16_t pin, uint8_t none __attribute__((unused)), enum rcc_periph_clken clken);
#endif

extern void actuators_pwm_arch_channel_init(uint32_t timer_peripheral, enum tim_oc_id oc_id);

extern void set_servo_timer(uint32_t timer, uint32_t period, uint8_t channels_mask);

#endif /* ACTUATORS_PWM_SHARED_ARCH_H */
