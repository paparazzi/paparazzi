/*
 * Copyright (C) 2014 Tobias Muench
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
 * @file modules/gpio_timer/gpio_timer.c
 *
 * Configuration
 * @code{.xml}
 * <define name="SWITCH_GPIO" value="GPIOB,GPIO1"/>
 * @endcode
 */

#include "gpio_timer.h"
#include "generated/airframe.h"
#include "mcu_periph/gpio.h"

#ifndef GPIO_TIME_ON
#define GPIO_TIME_ON 8 /* 4Hz -> 2s */
#endif

#ifndef GPIO_TIME_OFF
#define GPIO_TIME_OFF 4 /* 4Hz -> 1s */
#endif

#ifndef POWER_ON
#define POWER_ON gpio_set
#endif

#ifndef POWER_OFF
#define POWER_OFF gpio_clear
#endif


#ifndef SWITCH_GPIO
#error GPIO_TIMER: Please specify SWITCH_GPIO (e.g. <define name="SWITCH_GPIO" value="GPIOB,GPIO1"/>)
#endif

uint32_t timer_1;
uint32_t timer_2;
bool_t status;

void gpio_timer_init(void)
{
 timer_1 = GPIO_TIME_OFF;
 timer_2 = GPIO_TIME_ON;
 gpio_setup_output(SWITCH_GPIO);
 POWER_ON(SWITCH_GPIO);
 status = TRUE;
}

void gpio_timer_periodic( void )
{
  if (timer_2==0 && status==TRUE) {
    POWER_OFF(SWITCH_GPIO);
    status = FALSE;
    timer_1 = GPIO_TIME_OFF;
  } else {
    timer_2--;
  }
  if (timer_1==0 && status==FALSE) {
    POWER_ON(SWITCH_GPIO);
    status = TRUE;
    timer_2 = GPIO_TIME_ON;
  } else {
    timer_1--;
  }
}
