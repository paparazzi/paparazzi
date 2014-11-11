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
 * Use radio channel to take a picture
 * Only work with Fixedwing
 */

#include "dc_shoot_rc.h"
#include "inter_mcu.h"
#include "dc.h"

#ifndef DC_RADIO_SHOOT
PRINT_CONFIG_MSG("You need to define DC_RADIO_SHOT to use this module");
#endif

void dc_shoot_rc_periodic(void)
{
  static uint8_t rd_shoot = 0;
  static uint8_t rd_num = 0;

  if ((rd_shoot == 0) && (fbw_state->channels[DC_RADIO_SHOOT] > 3000)) {
    dc_send_command(DC_SHOOT);
    rd_shoot = 1;
  }
  if ((rd_shoot == 1) && (rd_num < 4))  //FIX-IT using timer
  {rd_num = rd_num + 1;}
  else {
    rd_num = 0;
    rd_shoot = 0;
  }
}