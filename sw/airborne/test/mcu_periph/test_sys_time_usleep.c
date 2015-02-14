/*
 * Copyright (C) 2012 The Paparazzi Team
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
#include "led.h"
#include "mcu_periph/sys_time.h"

static inline void main_periodic_1(void);
static inline void main_periodic_15(void);
static inline void main_periodic_05(uint8_t id);

int main(void)
{

  mcu_init();
  sys_time_register_timer(0.5, main_periodic_05);

  mcu_int_enable();

  while (1) {
    /* sleep for 1s */
    sys_time_usleep(1000000);
    main_periodic_1();

    /* sleep for 0.5s */
    sys_time_usleep(500000);
    main_periodic_15();
  }

  return 0;
}

/*
 * Called from main loop polling
 */
static inline void main_periodic_1(void)
{
#ifdef LED_GREEN
  LED_TOGGLE(LED_GREEN);
#endif
}

static inline void main_periodic_15(void)
{
#ifdef LED_BLUE
  LED_TOGGLE(LED_BLUE);
#endif
}

/*
 * Called from the systime interrupt handler
 */
static inline void main_periodic_05(uint8_t id __attribute__((unused)))
{
#ifdef LED_RED
  LED_TOGGLE(LED_RED);
#endif
}
