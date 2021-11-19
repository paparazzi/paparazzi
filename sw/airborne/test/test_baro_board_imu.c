/*
 * Copyright (C) 2015 Braiins Systems <jan.capek@braiins.cz>
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

/*
 *
 * IMU initialization functions for boards where barometer is an I2C
 * slave of IMU chip.
 *
 */

#include BOARD_CONFIG

#include "mcu_periph/sys_time.h"

#include "modules/imu/imu.h"
#include "test_baro_board_imu.h"

void test_baro_board_imu_init(void)
{
  imu_init();
}


void test_baro_board_imu_periodic_task(void)
{
  if (sys_time.nb_sec > 1) {
    imu_periodic();
  }
}


void test_baro_board_imu_event_task(void)
{
  ImuEvent();
}
