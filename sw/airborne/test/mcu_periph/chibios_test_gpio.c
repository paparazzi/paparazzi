/*
 * Copyright (C) 2015 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
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

/* ChibiOS includes */
#include "ch.h"

/* paparazzi includes */
#include "mcu.h"
#include "led.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/gpio.h"

static inline void main_periodic(void);
static inline void main_periodic_2(void);

/*
 * Thread Area Definitions
 */
#define CH_CFG_THREAD_AREA_MAIN_PERIODIC 128

/*
 * Thread Area Initialization
 */
static THD_WORKING_AREA(wa_thd_main_periodic, CH_CFG_THREAD_AREA_MAIN_PERIODIC);

/*
 * Static Thread Definitions
 */
static __attribute__((noreturn)) void thd_main_periodic(void *arg);

/*
 * Test Thread
 */
static __attribute__((noreturn)) void thd_main_periodic(void *arg)
{
  chRegSetThreadName("thd_main_periodic");
  (void) arg;
  systime_t time = chVTGetSystemTime();
  while (TRUE)
  {
    time += TIME_MS2I(500);
    main_periodic();
    chThdSleepUntil(time);
  }
}

/*
 * Called from the systime interrupt handler
 */
static inline void main_periodic(void)
{
  gpio_toggle(GPIOA,8);
}


static inline void main_periodic_2(void)
{
  gpio_toggle(GPIOB,4);
}


int main(void)
{
  mcu_arch_init();

  /*
   * Init threads
   */
  chThdCreateStatic(wa_thd_main_periodic, sizeof(wa_thd_main_periodic), NORMALPRIO, thd_main_periodic, NULL);


  while (TRUE) {
	  sys_time_ssleep(1);
	  main_periodic_2();
  }

  return 0;
}
