/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * @file "modules/joystick/joystick.c"
 * @author Gautier Hattenberger
 * Handle JOYSTICK_RAW messages
 */

#include "modules/joystick/joystick.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/abi.h"
#include "pprzlink/dl_protocol.h"
#include "generated/airframe.h"

struct Joystick joystick;

void joystick_init(void)
{
  joystick.roll = 0;
  joystick.pitch = 0;
  joystick.yaw = 0;
  joystick.throttle = 0;
}

void joystick_parse(void)
{
  if (DL_JOYSTICK_RAW_ac_id(dl_buffer) == AC_ID) {
    joystick.roll = DL_JOYSTICK_RAW_roll(dl_buffer);
    joystick.pitch = DL_JOYSTICK_RAW_pitch(dl_buffer);
    joystick.yaw = DL_JOYSTICK_RAW_yaw(dl_buffer);
    joystick.throttle = DL_JOYSTICK_RAW_throttle(dl_buffer);
    AbiSendMsgJOYSTICK(JOYSTICK_ID, joystick.roll, joystick.pitch, joystick.yaw, joystick.throttle);
  }
}

