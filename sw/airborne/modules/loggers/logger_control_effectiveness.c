/*
 * Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 *
 */

/** @file modules/loggers/logger_control_effectiveness.c
 *  @brief Log data required to compute control effectiveness
 */

#include "modules/loggers/logger_control_effectiveness.h"
#include "modules/loggers/sdlog_chibios.h"
#include "mcu_periph/sys_time.h"
#include "state.h"

// define parameters logged by default

#ifndef LOGGER_CONTROL_EFFECTIVENESS_COMMANDS
#define LOGGER_CONTROL_EFFECTIVENESS_COMMANDS TRUE
#endif

#ifndef LOGGER_CONTROL_EFFECTIVENESS_ACTUATORS
#define LOGGER_CONTROL_EFFECTIVENESS_ACTUATORS FALSE
#endif

#ifndef LOGGER_CONTROL_EFFECTIVENESS_POS
#define LOGGER_CONTROL_EFFECTIVENESS_POS FALSE
#endif

#ifndef LOGGER_CONTROL_EFFECTIVENESS_SPEED
#define LOGGER_CONTROL_EFFECTIVENESS_SPEED FALSE
#endif

#ifndef LOGGER_CONTROL_EFFECTIVENESS_AIRSPEED
#define LOGGER_CONTROL_EFFECTIVENESS_AIRSPEED FALSE
#endif

// extra includes

#if LOGGER_CONTROL_EFFECTIVENESS_COMMANDS
#ifdef ROTORCRAFT_FIRMWARE
#include "firmwares/rotorcraft/stabilization.h"
#endif
#ifdef FIXEDWING_FIRMWARE
#include "modules/core/commands.h"
#endif
#endif

#if LOGGER_CONTROL_EFFECTIVENESS_ACTUATORS
#include "modules/actuators/actuators.h"
#endif

/** Write the log header line according to the enabled parts */
void logger_control_effectiveness_start(void)
{
  if (pprzLogFile != -1) {
    sdLogWriteLog(pprzLogFile, "time,gyro_p,gyro_q,gyro_r,ax,ay,az");
#if LOGGER_CONTROL_EFFECTIVENESS_COMMANDS
    for (unsigned int i = 0; i < COMMANDS_NB; i++) {
      sdLogWriteLog(pprzLogFile, ",cmd_%d", i);
    }
#endif
#if LOGGER_CONTROL_EFFECTIVENESS_ACTUATORS
    for (unsigned int i = 0; i < ACTUATORS_NB; i++) {
      sdLogWriteLog(pprzLogFile, ",act_%d", i);
    }
#endif
#if LOGGER_CONTROL_EFFECTIVENESS_POS
    sdLogWriteLog(pprzLogFile, ",pos_x,pos_y,pos_z");
#endif
#if LOGGER_CONTROL_EFFECTIVENESS_SPEED
    sdLogWriteLog(pprzLogFile, ",speed_x,speed_y,speed_z");
#endif
#if LOGGER_CONTROL_EFFECTIVENESS_AIRSPEED
    sdLogWriteLog(pprzLogFile, ",eas");
#endif
    sdLogWriteLog(pprzLogFile,"\n");
  }
}


/** Log the values to file */
void logger_control_effectiveness_periodic(void)
{
  if (pprzLogFile == -1) {
    return;
  }

  struct Int32Rates *rates = stateGetBodyRates_i();
  struct Int32Vect3 *accel_body = stateGetAccelBody_i();

  // log time, rate and accel
  sdLogWriteLog(pprzLogFile, "%.5f,%ld,%ld,%ld,%ld,%ld,%ld",
      get_sys_time_float(),
      rates->p,
      rates->q,
      rates->r,
      accel_body->x,
      accel_body->y,
      accel_body->z);

  // log commands
#if LOGGER_CONTROL_EFFECTIVENESS_COMMANDS
  for (unsigned int i = 0; i < COMMANDS_NB; i++) {
#ifdef ROTORCRAFT_FIRMWARE
    sdLogWriteLog(pprzLogFile, ",%ld", stabilization_cmd[i]);
#endif
#ifdef FIXEDWING_FIRMWARE
    sdLogWriteLog(pprzLogFile, ",%d", commands[i]);
#endif
  }
#endif

  // log actuators
#if LOGGER_CONTROL_EFFECTIVENESS_ACTUATORS
  for (unsigned int i = 0; i < ACTUATORS_NB; i++) {
    sdLogWriteLog(pprzLogFile, ",%d", actuators[i]);
  }
#endif

  // log position
#if LOGGER_CONTROL_EFFECTIVENESS_POS
  struct EnuCoor_i *pos = stateGetPositionEnu_i();
  sdLogWriteLog(pprzLogFile, ",%ld,%ld,%ld", pos->x, pos->y, pos->z);
#endif

  // log speed
#if LOGGER_CONTROL_EFFECTIVENESS_SPEED
  struct EnuCoor_i *speed = stateGetSpeedEnu_i();
  sdLogWriteLog(pprzLogFile, ",%ld,%ld,%ld", speed->x, speed->y, speed->z);
#endif

  // log airspeed
#if LOGGER_CONTROL_EFFECTIVENESS_AIRSPEED
  sdLogWriteLog(pprzLogFile, ",%.2f", stateGetAirspeed_f());
#endif

  // end line
  sdLogWriteLog(pprzLogFile,"\n");
}

