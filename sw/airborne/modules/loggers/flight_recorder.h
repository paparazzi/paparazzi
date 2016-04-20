/*
 * Copyright (C) 2015 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
/**
 * @file "modules/loggers/flight_recorder.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Record flight data according to your telemetry file
 */

#ifndef FLIGHT_RECORDER_H
#define FLIGHT_RECORDER_H

#if FLIGHTRECORDER_SDLOG
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
extern struct chibios_sdlog flightrecorder_sdlog;
#endif

/** Init function
 */
extern void flight_recorder_init(void);

/** Periodic function
 *
 * should be called at TELEMETRY_FREQUENCY
 */
extern void flight_recorder_periodic(void);

#endif

