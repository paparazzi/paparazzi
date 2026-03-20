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
#include "mcu_periph/sys_time.h"
#include "state.h"

#if USE_CHIBIOS_RTOS
#include "modules/loggers/sdlog_chibios.h"
#define LogWrite sdLogWriteLog
#define LogFileIsOpen() (pprzLogFile != -1)
#define LogOpen(_file) {}
#define LogClose(_file) {}
#define LogFormatHeader "%.5f,%ld,%ld,%ld,%ld,%ld,%ld"
#define LogFormatVect3 ",%ld,%ld,%ld"

#else // assume Linux based OS
#include <stdio.h>
#include <time.h>
#include <unistd.h>
static void open_log(FILE* file);
#define LogWrite fprintf
#define LogFileIsOpen() (pprzLogFile != NULL)
#define LogOpen(_file) open_log(_file)
#define LogClose(_file) { \
    fclose(_file); \
    _file = NULL; \
}
#define LogFormatHeader "%.5f,%d,%d,%d,%d,%d,%d"
#define LogFormatVect3 ",%d,%d,%d"
static FILE* pprzLogFile = NULL;

/* Set the default log path to bebop storage */
#ifndef LOGGER_CONTROL_EFFECTIVENESS_FILE_PATH
#define LOGGER_CONTROL_EFFECTIVENESS_FILE_PATH /data/ftp/internal_000
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

// extra includes

#if LOGGER_CONTROL_EFFECTIVENESS_COMMANDS
#include "modules/core/commands.h"
#endif

#if LOGGER_CONTROL_EFFECTIVENESS_ACTUATORS
#include "modules/actuators/actuators.h"
#endif

/** Write the log header line according to the enabled parts */
void logger_control_effectiveness_start(void)
{
  LogOpen(pprzLogFile);

  if (LogFileIsOpen()) {
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
    LogWrite(pprzLogFile,"\n");
  }
}

void logger_control_effectiveness_stop(void)
{
  if (LogFileIsOpen()) {
    LogClose(pprzLogFile);
  }
}

/** Log the values to file */
void logger_control_effectiveness_periodic(void)
{
  if (LogFileIsOpen()) {
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

  // end line
  LogWrite(pprzLogFile,"\n");
}

#if !USE_CHIBIOS_RTOS
static void open_log(FILE *file)
{
  // Create output folder if necessary
  if (access(STRINGIFY(LOGGER_CONTROL_EFFECTIVENESS_FILE_PATH), F_OK)) {
    char save_dir_cmd[256];
    sprintf(save_dir_cmd, "mkdir -p %s", STRINGIFY(LOGGER_CONTROL_EFFECTIVENESS_FILE_PATH));
    if (system(save_dir_cmd) != 0) {
      printf("[logger] Could not create log file directory %s.\n", STRINGIFY(LOGGER_CONTROL_EFFECTIVENESS_FILE_PATH));
      return;
    }
  }

  // Get current date/time for filename
  char date_time[80];
  time_t now = time(0);
  struct tm  tstruct;
  tstruct = *localtime(&now);
  strftime(date_time, sizeof(date_time), "%Y%m%d-%H%M%S", &tstruct);

  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(LOGGER_CONTROL_EFFECTIVENESS_FILE_PATH), date_time);
  while ((file = fopen(filename, "r"))) {
    fclose(file);

    sprintf(filename, "%s/%s_%05d.csv", STRINGIFY(LOGGER_FILE_PATH), date_time, counter);
    counter++;
  }

  file = fopen(filename, "w");
  if(!file) {
    printf("[logger] ERROR opening log file %s!\n", filename);
    return;
  }

  printf("[logger] Start logging to %s...\n", filename);
}
#endif

