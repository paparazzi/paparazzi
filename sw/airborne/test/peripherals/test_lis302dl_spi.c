/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file test/peripherals/test_lis302dl_spi.c
 *
 * Test for LIS302DL 3-axis accelerometer from ST using SPI.
 */

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#define DATALINK_C
#include "subsystems/datalink/downlink.h"
#include "led.h"

#include "peripherals/lis302dl_spi.h"

#ifndef LIS302DL_SPI_DEV
#define LIS302DL_SPI_DEV spi2
#endif
PRINT_CONFIG_VAR(LIS302DL_SPI_DEV)

#ifndef LIS302DL_SPI_SLAVE_IDX
#define LIS302DL_SPI_SLAVE_IDX SPI_SLAVE2
#endif
PRINT_CONFIG_VAR(LIS302DL_SPI_SLAVE_IDX)

struct Lis302dl_Spi lis302;

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
  downlink_init();
  lis302dl_spi_init(&lis302, &(LIS302DL_SPI_DEV), LIS302DL_SPI_SLAVE_IDX);
}

static inline void main_periodic_task(void)
{
  RunOnceEvery(100, DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM));

  if (sys_time.nb_sec > 1) {
    lis302dl_spi_periodic(&lis302);
#if USE_LED_5
    RunOnceEvery(10, LED_TOGGLE(5););
#endif
  }
}

static inline void main_event_task(void)
{
  mcu_event();

  if (sys_time.nb_sec > 1) {
    lis302dl_spi_event(&lis302);
  }

  if (lis302.data_available) {
    struct Int32Vect3 accel;
    VECT3_COPY(accel, lis302.data.vect);
    lis302.data_available = false;

    RunOnceEvery(10, {
      DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice,
      &accel.x, &accel.y, &accel.z);
#if USE_LED_6
      LED_TOGGLE(6);
#endif
    });
  }
}
