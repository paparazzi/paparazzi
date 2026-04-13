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

/** @file modules/loggers/logger_utils.h
 *  @brief Generic definitions and tools for logging on ChibiOS or Linux
 */

#ifndef LOGGER_UTILS_H
#define LOGGER_UTILS_H

#if USE_CHIBIOS_RTOS

#include "modules/loggers/sdlog_chibios.h"
#define LogWrite sdLogWriteLog
#define LogFileIsOpen(_file) (_file != -1)
#define LogOpen(_file, _path, _name) {}
#define LogClose(_file) {}
typedef FileDes LogFile_t;

#else // assume Linux based OS

#include <stdio.h>
#include <unistd.h>
#define LogWrite fprintf
#define LogFileIsOpen(_file) (_file != NULL)
#define LogOpen(_file, _path, _name) { _file = open_log(_path, _name); }
#define LogClose(_file) { \
    fclose(_file); \
    _file = NULL; \
}
typedef FILE* LogFile_t;

/** Open a log file.
 *  If the log name already exists, a counter is the file name is automatically incremented.
 * @param path log directory
 * @param name use current time if NULL, specified name otherwise
 * @return file pointer
 */
extern FILE *open_log(char* path, char* name);

#endif

#endif /* LOGGER_UTILS_H */

