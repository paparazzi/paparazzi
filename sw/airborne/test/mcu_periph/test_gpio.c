/*
 * Copyright (C) 2014 Felix Ruess <felix.ruess@gmail.com>
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

#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/gpio.h"

#ifndef TEST_GPIO1
#define TEST_GPIO1 GPIOB,GPIO17
#endif

#ifndef TEST_GPIO2
#define TEST_GPIO2 GPIOB,GPIO23
#endif


/*
 * Called from the systime interrupt handler
 */
static inline void main_periodic(uint8_t id __attribute__((unused)))
{
  gpio_toggle(TEST_GPIO1);
}


static inline void main_periodic_2(void)
{
  gpio_toggle(TEST_GPIO2);
}


int main(void)
{

  // not calling mcu_init with PERIPHERALS_AUTO_INIT
  // rather explicitly init only sys_time
  mcu_arch_init();
  sys_time_init();

  gpio_setup_output(TEST_GPIO1);
  gpio_setup_output(TEST_GPIO2);

  unsigned int tmr_2 = sys_time_register_timer(2, NULL);
  sys_time_register_timer(1, main_periodic);


  while (1) {
    if (sys_time_check_and_ack_timer(tmr_2)) {
      main_periodic_2();
    }
  }

  return 0;
}
