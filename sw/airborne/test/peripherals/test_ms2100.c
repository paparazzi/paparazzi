/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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


#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#define DATALINK_C
#include "subsystems/datalink/downlink.h"
#include "modules/datalink/pprz_dl.h"
#include "peripherals/ms2100.h"
#include "led.h"

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);

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
  sys_time_register_timer((1. / 50), NULL);

  ms2100_init(&ms2100, &(MS2100_SPI_DEV), MS2100_SLAVE_IDX);
  downlink_init();
  pprz_dl_init();
}

static inline void main_periodic_task(void)
{
  RunOnceEvery(10, {
    uint16_t foo = sys_time.nb_sec;
    DOWNLINK_SEND_TAKEOFF(DefaultChannel, DefaultDevice, &foo);
    LED_TOGGLE(2);
    LED_PERIODIC();
  });

  ms2100_periodic(&ms2100);

}

static inline void main_event_task(void)
{
  mcu_event();

  ms2100_event(&ms2100);
  if (ms2100.status == MS2100_DATA_AVAILABLE) {
    RunOnceEvery(10, {
      int32_t mag_x = ms2100.data.vect.x;
      int32_t mag_y = ms2100.data.vect.y;
      int32_t mag_z = ms2100.data.vect.z;
      DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice,
      &mag_x, &mag_y, &mag_z);
    });
    ms2100.status = MS2100_IDLE;
  }
}

