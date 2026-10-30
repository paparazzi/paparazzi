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
#include "modules/loggers/logger_utils.h"
#include "mcu_periph/sys_time.h"
#include "state.h"

#if USE_CHIBIOS_RTOS
#define LogFormatHeader "%.5f,%ld,%ld,%ld,%ld,%ld,%ld"
#define LogFormatVect3 ",%ld,%ld,%ld"
#define LogFormatI32 ",%ld"

#else // assume Linux based OS
#define LogFormatHeader "%.5f,%d,%d,%d,%d,%d,%d"
#define LogFormatVect3 ",%d,%d,%d"
#define LogFormatI32 ",%d"
static FILE* pprzLogFile = NULL;

/* Set the default log path to bebop storage */
#ifndef LOGGER_CONTROL_EFFECTIVENESS_FILE_PATH
#define LOGGER_CONTROL_EFFECTIVENESS_FILE_PATH /data/ftp/internal_000/control_eff
#endif

#endif

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

#ifndef LOGGER_CONTROL_EFFECTIVENESS_RPM
#define LOGGER_CONTROL_EFFECTIVENESS_RPM FALSE
#endif

// extra includes

#if LOGGER_CONTROL_EFFECTIVENESS_COMMANDS
#include "modules/core/commands.h"
#endif

#if LOGGER_CONTROL_EFFECTIVENESS_ACTUATORS
#include "modules/actuators/actuators.h"
#endif

#if LOGGER_CONTROL_EFFECTIVENESS_RPM
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"
static abi_event rpm_feedback_ev;
static int32_t act_rpm[ACTUATORS_NB];

static void rpm_feedback_cb(uint8_t sender_id UNUSED, struct act_feedback_t *feedback, uint8_t num_act)
{
  for (int i = 0; i < num_act; i++) {
    if (feedback[i].set.rpm) {
      act_rpm[feedback[i].idx] = feedback[i].rpm;
    }
  }
}
#endif

/** Init module */
void logger_control_effectiveness_init(void)
{
#if LOGGER_CONTROL_EFFECTIVENESS_RPM
  AbiBindMsgACT_FEEDBACK(ABI_BROADCAST, &rpm_feedback_ev, rpm_feedback_cb);
#endif
}

/** Write the log header line according to the enabled parts */
void logger_control_effectiveness_start(void)
{
  LogOpen(pprzLogFile, STRINGIFY(LOGGER_CONTROL_EFFECTIVENESS_FILE_PATH), NULL);

  if (LogFileIsOpen(pprzLogFile)) {
    LogWrite(pprzLogFile, "time,gyro_p,gyro_q,gyro_r,ax,ay,az");
#if LOGGER_CONTROL_EFFECTIVENESS_COMMANDS
    for (unsigned int i = 0; i < COMMANDS_NB; i++) {
      LogWrite(pprzLogFile, ",cmd_%d", i);
    }
#endif
#if LOGGER_CONTROL_EFFECTIVENESS_ACTUATORS
    for (unsigned int i = 0; i < ACTUATORS_NB; i++) {
      LogWrite(pprzLogFile, ",act_%d", i);
    }
#endif
#if LOGGER_CONTROL_EFFECTIVENESS_POS
    LogWrite(pprzLogFile, ",pos_x,pos_y,pos_z");
#endif
#if LOGGER_CONTROL_EFFECTIVENESS_SPEED
    LogWrite(pprzLogFile, ",speed_x,speed_y,speed_z");
#endif
#if LOGGER_CONTROL_EFFECTIVENESS_AIRSPEED
    LogWrite(pprzLogFile, ",eas");
#endif
#if LOGGER_CONTROL_EFFECTIVENESS_RPM
    for (unsigned int i = 0; i < ACTUATORS_NB; i++) {
      LogWrite(pprzLogFile, ",rpm_%d", i);
    }
#endif
    LogWrite(pprzLogFile,"\n");
  }
}

void logger_control_effectiveness_stop(void)
{
  if (LogFileIsOpen(pprzLogFile)) {
    LogClose(pprzLogFile);
  }
}

/** Log the values to file */
void logger_control_effectiveness_periodic(void)
{
  if (!LogFileIsOpen(pprzLogFile)) {
    return;
  }

  struct Int32Rates *rates = stateGetBodyRates_i();
  struct Int32Vect3 *accel_body = stateGetAccelBody_i();

  // log time, rate and accel
  LogWrite(pprzLogFile, LogFormatHeader,
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
    LogWrite(pprzLogFile, ",%d", commands[i]);
  }
#endif

  // log actuators
#if LOGGER_CONTROL_EFFECTIVENESS_ACTUATORS
  for (unsigned int i = 0; i < ACTUATORS_NB; i++) {
    LogWrite(pprzLogFile, ",%d", actuators[i].pprz_val);
  }
#endif

  // log position
#if LOGGER_CONTROL_EFFECTIVENESS_POS
  struct EnuCoor_i *pos = stateGetPositionEnu_i();
  LogWrite(pprzLogFile, LogFormatVect3, pos->x, pos->y, pos->z);
#endif

  // log speed
#if LOGGER_CONTROL_EFFECTIVENESS_SPEED
  struct EnuCoor_i *speed = stateGetSpeedEnu_i();
  LogWrite(pprzLogFile, LogFormatVect3, speed->x, speed->y, speed->z);
#endif

  // log airspeed
#if LOGGER_CONTROL_EFFECTIVENESS_AIRSPEED
  LogWrite(pprzLogFile, ",%.2f", stateGetAirspeed_f());
#endif

  // log rpm
#if LOGGER_CONTROL_EFFECTIVENESS_RPM
  for (unsigned int i = 0; i < ACTUATORS_NB; i++) {
    LogWrite(pprzLogFile, LogFormatI32, act_rpm[i]);
  }
#endif
  // end line
  LogWrite(pprzLogFile,"\n");
}

