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

static inline void main_periodic_02(void);
static inline void main_periodic_03(void);
static inline void main_periodic_05(uint8_t id);
static inline void main_event(void);

int main(void)
{

  mcu_init();
  tid_t tmr_02 = sys_time_register_timer(0.2, NULL);
  tid_t tmr_03 = sys_time_register_timer(0.3, NULL);
  sys_time_register_timer(0.5, main_periodic_05);


  while (1) {
    if (sys_time_check_and_ack_timer(tmr_02)) {
      main_periodic_02();
    }
    if (sys_time_check_and_ack_timer(tmr_03)) {
      main_periodic_03();
    }
    main_event();
  }

  return 0;
}

/*
   Called from main loop polling
*/
static inline void main_periodic_02(void)
{
#ifdef LED_GREEN
  LED_TOGGLE(LED_GREEN);
#endif
}

static inline void main_periodic_03(void)
{
#ifdef LED_BLUE
  LED_TOGGLE(LED_BLUE);
#endif
}

/*
   Called from the systime interrupt handler
*/
static inline void main_periodic_05(__attribute__((unused)) uint8_t id)
{
#ifdef LED_RED
  LED_TOGGLE(LED_RED);
#endif
}


static inline void main_event(void)
{
}
