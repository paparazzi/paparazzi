/*
 * $Id$
 *
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

#include "subsystems/ahrs.h"
#include "firmwares/rotorcraft/commands.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "camera_mount.h"


void camera_mount_init(void) {
  commands[COMMAND_CAMERA] = MIN_PPRZ;
}


void camera_mount_run(void) {
  int32_t pitch = ahrs.ltp_to_body_euler.theta;
  if (pitch < 0) pitch = 0;
  commands[COMMAND_CAMERA] = pitch * 3 - 7200;
  Bound(commands[COMMAND_CAMERA], MIN_PPRZ, MAX_PPRZ);

  switch (guidance_h_mode) {
    case GUIDANCE_H_MODE_TOYTRONICS_FORWARD:
    case GUIDANCE_H_MODE_TOYTRONICS_AEROBATIC:
      commands[COMMAND_CAMERA] = -9600;
      break;

    default:
      break;
      // other modes do nothing, leave previous command in place
  }
}
