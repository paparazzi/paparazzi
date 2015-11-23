/*
 * Copyright (C) 2015 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file boards/ardrone/board.c
 *
 * ARDrone2 specific board initialization function.
 *
 */

#include <stdlib.h>
#include "mcu.h"

/* check if the bat_voltage_ardrone2 module is loaded */
#include "generated/modules.h"
#ifndef BAT_VOLTAGE_ARDRONE2_PERIODIC_FREQ
#warning No battery voltage measurement available! Please add <load name="bat_voltage_ardrone2.xml"/> to your modules.
#endif

#include "peripherals/video_device.h"

struct video_config_t front_camera = {
  .w = 1280,
  .h = 720,
  .dev_name = "/dev/video1",
  .subdev_name = NULL,
  .format = V4L2_PIX_FMT_UYVY,
  .buf_cnt = 10,
  .filters = 0
};

struct video_config_t bottom_camera = {
  .w = 320,
  .h = 240,
  .dev_name = "/dev/video2",
  .subdev_name = NULL,
  .format = V4L2_PIX_FMT_UYVY,
  .buf_cnt = 10,
  .filters = 0
};


void board_init(void)
{
  // First we try to kill the program.elf and its respawner if it is running
  int ret = system("killall -9 program.elf.respawner.sh; killall -9 program.elf");
  (void) ret;
}
