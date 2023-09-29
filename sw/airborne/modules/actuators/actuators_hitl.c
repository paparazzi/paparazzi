/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/actuators/actuators_hitl.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Sends commands or actuators for Hardware In The Loop simulation
 */

#include "modules/actuators/actuators_hitl.h"
#include "modules/datalink/telemetry.h"
#include "modules/core/commands.h"
#include "modules/actuators/actuators.h"
#include "generated/airframe.h"
#if ROTORCRAFT_FIRMWARE && NPS_NO_MOTOR_MIXING
#include "modules/actuators/motor_mixing.h"
#endif


#ifndef HITL_DEVICE 
#error "HITL_DEVICE must be defined"
#endif

// for sending only
static struct pprz_transport actuators_hitl_tp;

void actuators_hitl_init(void)
{
  pprz_transport_init(&actuators_hitl_tp);
}

void actuators_hitl_periodic(void)
{
#if FIXEDWING_FIRMWARE
  pprz_msg_send_COMMANDS(&actuators_hitl_tp.trans_tx, &(HITL_DEVICE).device, AC_ID,
      COMMANDS_NB, commands);
#endif
#if ROTORCRAFT_FIRMWARE
#if NPS_NO_MOTOR_MIXING
  pprz_msg_send_ACTUATORS(&actuators_hitl_tp.trans_tx, &(HITL_DEVICE).device, AC_ID,
      ACTUATORS_NB, actuators_pprz);
#else // use motor mixing
  int16_t motors[MOTOR_MIXING_NB_MOTOR];
  for (uint8_t i = 0; i < MOTOR_MIXING_NB_MOTOR; i++)
  {
    motors[i] = (int16_t)motor_mixing.commands[i];
  }
  pprz_msg_send_MOTOR_MIXING(&actuators_hitl_tp.trans_tx, &(HITL_DEVICE).device, AC_ID,
      MOTOR_MIXING_NB_MOTOR, motors);
#endif
#endif
}


