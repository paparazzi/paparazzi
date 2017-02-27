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

#include "mcu_periph/gpio.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include BOARD_CONFIG

#include "std.h"

/*
 *
 *  Regular GPIO driven LEDs
 *
 */
#ifndef LED_STP08

#define _LED_EVAL(i) i

#define LED_GPIO(i) _LED_EVAL(LED_ ## i ## _GPIO)
#define LED_GPIO_PIN(i) _LED_EVAL(LED_ ## i ## _GPIO_PIN)
#define LED_GPIO_ON(i) _LED_EVAL(LED_ ## i ## _GPIO_ON)
#define LED_GPIO_OFF(i) _LED_EVAL(LED_ ## i ## _GPIO_OFF)
#define LED_AFIO_REMAP(i) _LED_EVAL(LED_ ## i ## _AFIO_REMAP)


#define LED_INIT(i) {                                  \
    gpio_setup_output(LED_GPIO(i), LED_GPIO_PIN(i));   \
    LED_AFIO_REMAP(i);                                 \
  }

#define LED_ON(i) LED_GPIO_ON(i)(LED_GPIO(i), LED_GPIO_PIN(i))
#define LED_OFF(i) LED_GPIO_OFF(i)(LED_GPIO(i), LED_GPIO_PIN(i))
#define LED_TOGGLE(i) gpio_toggle(LED_GPIO(i), LED_GPIO_PIN(i))

#define LED_DISABLE(i) gpio_setup_input(LED_GPIO(i), LED_GPIO_PIN(i))

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
    rcc_periph_clock_enable(RCC_GPIOA);                 \
    rcc_periph_clock_enable(RCC_GPIOC);                 \
    gpio_set_mode(GPIOA,                                \
                  GPIO_MODE_OUTPUT_50_MHZ,              \
                  GPIO_CNF_OUTPUT_PUSHPULL,             \
                  GPIO8);                               \
    gpio_set_mode(GPIOC,                                \
                  GPIO_MODE_OUTPUT_50_MHZ,              \
                  GPIO_CNF_OUTPUT_PUSHPULL,             \
                  GPIO15);                              \
    for(uint8_t _cnt=0; _cnt<NB_LED; _cnt++)            \
      led_status[_cnt] = false;                         \
  }

#define LED_ON(i)  { led_status[i] = true;  }
#define LED_OFF(i) { led_status[i] = false; }
#define LED_TOGGLE(i) {led_status[i] = !led_status[i];}
#define LED_DISABLE(i) LED_OFF(i)

#define LED_PERIODIC() {                                    \
    for (uint8_t _cnt = 0; _cnt < NB_LED; _cnt++) {         \
      if (led_status[_cnt])                                 \
        gpio_set(GPIOC, GPIO15);                            \
      else                                                  \
        gpio_clear(GPIOC, GPIO15);                          \
      gpio_set(GPIOA, GPIO8); /* clock rising edge */       \
      gpio_clear(GPIOA, GPIO8);  /* clock falling edge */   \
    }                                                       \
  }

#endif /* LED_STP08 */

#endif /* LED_HW_H */
