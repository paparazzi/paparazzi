/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include <inttypes.h>

#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#define DATALINK_C
#include "subsystems/datalink/downlink.h"

#include "mcu_periph/i2c.h"
#include "peripherals/ami601.h"
#include "math/pprz_algebra_int.h"

#include "std.h"

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);

static inline void on_mag(void);

int main(void)
{
  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
    main_event_task();
  }
  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);

  LED_ON(4);
  ami601_init();
  downlink_init();
  mcu_int_enable();
}

static inline void main_periodic_task(void)
{
  //  RunOnceEvery(100, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);});

  RunOnceEvery(10, { ami601_read();});
}

static inline void main_event_task(void)
{
  mcu_event();

  AMI601Event(on_mag);
}

static inline void on_mag(void)
{
  LED_TOGGLE(4);
  ami601_status = AMI601_IDLE;
  struct Int32Vect3 bla = {ami601_values[0], ami601_values[1], ami601_values[2]};
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &bla.x, &bla.y, &bla.z);
}
