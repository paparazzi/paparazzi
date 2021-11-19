/*
 * Copyright (C) 2014 Eduardo Lavratti <agressiva@hotmail.com>
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

/** @file modules/digital_cam/dc_shoot_rc.c
 * Digital Camera remote shoot using radio channel.
 *
 * Use radio channel to take a picture.
 * Only works with fixedwing firmware.
 */

#include "dc_shoot_rc.h"
#include "modules/intermcu/inter_mcu.h"
#include "dc.h"

#ifndef DC_RADIO_SHOOT
#error "You need to define DC_RADIO_SHOOT to a RADIO_xxx channel to use this module"
#endif

#define DC_RADIO_SHOOT_THRESHOLD 3000

void dc_shoot_rc_periodic(void)
{
  static uint8_t rd_shoot = 0;
  static uint8_t rd_num = 0;

  if ((rd_shoot == 0) && (imcu_get_radio(DC_RADIO_SHOOT) > DC_RADIO_SHOOT_THRESHOLD)) {
    dc_send_command(DC_SHOOT);
    rd_shoot = 1;
  }
  if ((rd_shoot == 1) && (rd_num < 4)) {
    rd_num = rd_num + 1;
  } else {
    rd_num = 0;
    rd_shoot = 0;
  }
}
