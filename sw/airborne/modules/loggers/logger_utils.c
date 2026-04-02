/*
 * Copyright (C) 2026 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file modules/loggers/logger_utils.c
 *  @brief Generic definitions and tools for logging on ChibiOS or Linux
 */

#include "modules/loggers/logger_utils.h"

#if !USE_CHIBIOS_RTOS

#include <time.h>
#include <stdlib.h>
#include <stdint.h>

FILE *open_log(char* path, char* name)
{
  // Create output folder if necessary
  if (access(path, F_OK)) {
    char save_dir_cmd[256];
    sprintf(save_dir_cmd, "mkdir -p %s", path);
    if (system(save_dir_cmd) != 0) {
      printf("[logger] Could not create log file directory %s.\n", path);
      return NULL;
    }
  }

  uint32_t counter = 0;
  char filename[512];
  char date_time[80];

  if (name == NULL) {
    // Get current date/time for filename
    time_t now = time(0);
    struct tm  tstruct;
    tstruct = *localtime(&now);
    strftime(date_time, sizeof(date_time), "%Y%m%d-%H%M%S", &tstruct);
    sprintf(filename, "%s/%s.csv", path, date_time);
  } else {
    // use specified name
    sprintf(filename, "%s/%s.csv", path, name);
  }

  // Check for available files
  FILE *file;
  while ((file = fopen(filename, "r"))) {
    fclose(file);

    if (name == NULL) {
      sprintf(filename, "%s/%s_%05d.csv", path, date_time, counter);
    } else {
      sprintf(filename, "%s/%s_%05d.csv", path, name, counter);
    }
    counter++;
  }

  file = fopen(filename, "w");
  if(!file) {
    printf("[logger] ERROR opening log file %s!\n", filename);
    return NULL;
  }

  printf("[logger] Start logging to %s...\n", filename);
  return file;
}

#endif

