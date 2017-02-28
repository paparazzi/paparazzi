/*
 * Copyright (C) 2013-2015 Gautier Hattenberger, Alexandre Bustico
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

/*
 * @file modules/loggers/sdlog_chibios.c
 * @brief sdlog process with battery monitoring
 *
 */

#ifndef SDLOG_CHIBIOS_H
#define SDLOG_CHIBIOS_H

#include "std.h"
#include "modules/loggers/sdlog_chibios/sdLog.h"
#include "pprzlink/pprzlink_device.h"

/*
 what to be done  :
 * having an api to register new log
 * keep internally a list of open file
 * when power failure event occurs, close all logs
 */

extern FileDes pprzLogFile;

#if FLIGHTRECORDER_SDLOG
// if activated, will log specific telemetry process
extern FileDes flightRecorderLogFile;
#endif

extern void sdlog_chibios_init(void);
extern void sdlog_chibios_finish(bool flush);

/** chibios_sdlog structure
 */
struct chibios_sdlog {
  FileDes *file;
  /** Generic device interface */
  struct link_device device;
};

extern struct chibios_sdlog chibios_sdlog;

/** init chibios_sdlog structure
 */
extern void chibios_sdlog_init(struct chibios_sdlog *sdlog, FileDes *file);

#endif
