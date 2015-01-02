/*
 * Copyright (C) 2012-2013 Freek van Tienen
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
 * @file boards/ardrone/actuators_at.c
 * ardrone2-sdk actuators are driven by external software controller by AT-commands
 */

#include "subsystems/ahrs/ahrs_ardrone2.h"
#include "actuators_at.h"
#include "generated/airframe.h"
#include "boards/ardrone/at_com.h"

void actuators_init(void)
{
  init_at_com();
}

void actuators_set(pprz_t commands[])
{
  //Calculate the thrus, roll, pitch and yaw from the PPRZ commands
  float thrust = ((float)(commands[COMMAND_THRUST] - MAX_PPRZ / 2) / (float)MAX_PPRZ) * 2.0f;
  float roll = ((float)commands[COMMAND_ROLL] / (float)MAX_PPRZ);
  float pitch = ((float)commands[COMMAND_PITCH] / (float)MAX_PPRZ);
  float yaw = ((float)commands[COMMAND_YAW] / (float)MAX_PPRZ);

  //Starting engine
  if(thrust > 0 && (ahrs_ardrone2.control_state == CTRL_DEFAULT || ahrs_ardrone2.control_state == CTRL_INIT
                    || ahrs_ardrone2.control_state == CTRL_LANDED)) {
    at_com_send_ref(REF_TAKEOFF);
  }

  //Check emergency or stop engine
  if ((ahrs_ardrone2.state & ARDRONE_EMERGENCY_MASK) != 0) {
    at_com_send_ref(REF_EMERGENCY);
  } else if(thrust < -0.9 && !(ahrs_ardrone2.control_state == CTRL_DEFAULT ||
            ahrs_ardrone2.control_state == CTRL_INIT || ahrs_ardrone2.control_state == CTRL_LANDED)) {
    at_com_send_ref(0);
  }

  //Calibration
  if ((ahrs_ardrone2.state & ARDRONE_MAGNETO_NEEDS_CALIB) != 0 &&
      (ahrs_ardrone2.control_state == CTRL_FLYING || ahrs_ardrone2.control_state == CTRL_HOVERING)) {
    at_com_send_calib(0);
  }

  //Moving
  if ((ahrs_ardrone2.state & ARDRONE_MAGNETO_NEEDS_CALIB) == 0 &&
      (ahrs_ardrone2.control_state == CTRL_FLYING || ahrs_ardrone2.control_state == CTRL_HOVERING)) {
    at_com_send_pcmd(1, thrust, roll, pitch, yaw);
  }

  //Keep alive (FIXME)
  at_com_send_config("general:navdata_demo", "FALSE");
}
