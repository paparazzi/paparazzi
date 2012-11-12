/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef LED_HW_H
#define LED_HW_H

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>

#include BOARD_CONFIG

#include "std.h"


/*
 *
 *  Regular GPIO driven LEDs
 *
 */
#ifndef LED_STP08

#define _LED_GPIO_CLK(i)  i
#define _LED_GPIO(i)  i
#define _LED_GPIO_PIN(i) i
#define _LED_GPIO_ON(i) i
#define _LED_GPIO_OFF(i) i
#define _LED_AFIO_REMAP(i) i

#define LED_GPIO_CLK(i) _LED_GPIO_CLK(LED_ ## i ## _GPIO_CLK)
#define LED_GPIO(i) _LED_GPIO(LED_ ## i ## _GPIO)
#define LED_GPIO_PIN(i) _LED_GPIO_PIN(LED_ ## i ## _GPIO_PIN)
#define LED_GPIO_ON(i) _LED_GPIO_ON(LED_ ## i ## _GPIO_ON)
#define LED_GPIO_OFF(i) _LED_GPIO_OFF(LED_ ## i ## _GPIO_OFF)
#define LED_AFIO_REMAP(i) _LED_AFIO_REMAP(LED_ ## i ## _AFIO_REMAP)

/* set pin as output */
#define LED_INIT(i) {                               \
    rcc_peripheral_enable_clock(&RCC_APB2ENR,       \
                                LED_GPIO_CLK(i));	\
    gpio_set_mode(LED_GPIO(i),                      \
                  GPIO_MODE_OUTPUT_50_MHZ,          \
                  GPIO_CNF_OUTPUT_PUSHPULL,         \
                  LED_GPIO_PIN(i));                 \
    LED_AFIO_REMAP(i);                              \
  }

#define LED_ON(i) { LED_GPIO_ON(i)(LED_GPIO(i)) = LED_GPIO_PIN(i);}
#define LED_OFF(i) { LED_GPIO_OFF(i)(LED_GPIO(i)) = LED_GPIO_PIN(i);}
#define LED_TOGGLE(i) {	GPIO_ODR(LED_GPIO(i)) ^= LED_GPIO_PIN(i);}

#define LED_PERIODIC() {}


/*
 *
 * Shift register driven LEDs
 *
 */
#else  /* LED_STP08 */
#define NB_LED 8
extern uint8_t led_status[NB_LED];
/* Lisa/L uses a shift register for driving LEDs */
/* PA8  led_clk  */
/* PC15 led_data */

#define LED_INIT(_i) {                                  \
    rcc_peripheral_enable_clock(&RCC_APB2ENR,           \
                                RCC_APB2ENR_IOPAEN |    \
                                RCC_APB2ENR_IOPCEN);    \
    gpio_set_mode(GPIOA,                                \
                  GPIO_MODE_OUTPUT_50_MHZ,              \
                  GPIO_CNF_OUTPUT_PUSHPULL,             \
                  GPIO8);                               \
    gpio_set_mode(GPIOC,                                \
                  GPIO_MODE_OUTPUT_50_MHZ,              \
                  GPIO_CNF_OUTPUT_PUSHPULL,             \
                  GPIO15);                              \
    for(uint8_t _cnt=0; _cnt<NB_LED; _cnt++)            \
      led_status[_cnt] = FALSE;                         \
  }

#define LED_ON(i)  { led_status[i] = TRUE;  }
#define LED_OFF(i) { led_status[i] = FALSE; }
#define LED_TOGGLE(i) {led_status[i] = !led_status[i];}

#define LED_PERIODIC() {                                    \
    for (uint8_t _cnt = 0; _cnt < NB_LED; _cnt++) {         \
      if (led_status[_cnt])                                 \
        GPIO_BSRR(GPIOC) = GPIO15;                          \
      else                                                  \
        GPIO_BRR(GPIOC) = GPIO15;                           \
      GPIO_BSRR(GPIOA) = GPIO8; /* clock rising edge */     \
      GPIO_BRR(GPIOA) = GPIO8;  /* clock falling edge */    \
    }                                                       \
  }

#endif /* LED_STP08 */

#endif /* LED_HW_H */
