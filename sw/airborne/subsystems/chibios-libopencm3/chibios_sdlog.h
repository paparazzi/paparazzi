/*
 * Copyright (C) 2013 Gautier Hattenberger, Alexandre Bustico
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
 * @file subsystems/chibios-libopencm3/chibios_sdlog.h
 * @brief sdlog process with battery monitoring
 *
 */

#ifndef CHIBIOS_SDLOG_H
#define CHIBIOS_SDLOG_H

#include "ff.h"

/*
 what to be done  :
 * having an api to register new log
 * keep internally a list of open file
 * when power failure event occurs, close all logs
 */

extern FIL pprzLogFile;

#if LOG_PROCESS_STATE
// if activated, will log all process states
extern FIL processLogFile;
#endif

extern bool_t chibios_logInit(const bool_t binaryFile);
extern void chibios_logFinish(void);

#endif
