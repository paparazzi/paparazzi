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
 * @file "modules/loggers/flight_recorder.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Record flight data according to your telemetry file
 */

#define PERIODIC_C_FLIGHTRECORDER

#include "modules/loggers/flight_recorder.h"

#include "subsystems/datalink/telemetry.h"
#include "subsystems/datalink/pprzlog_transport.h"

#if FLIGHTRECORDER_SDLOG
#include "sdLog.h"
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
static struct chibios_sdlog flightrecorder_sdlog;
#ifndef FLIGHTRECORDER_DEVICE
#define FLIGHTRECORDER_DEVICE flightrecorder_sdlog
#else
#warning "SD log is activated, but FLIGHTRECORDER_DEVICE is alreay set (should not be defined)"
#endif

#else
// include downlink for other devices
#include "subsystems/datalink/downlink.h"
#endif

#ifndef TELEMETRY_PROCESS_FlightRecorder
#error "You need to use a telemetry xml file with FlightRecorder process!"
#endif

void flight_recorder_init()
{
#if FLIGHTRECORDER_SDLOG
  chibios_sdlog_init(&flightrecorder_sdlog, &flightRecorderLogFile);
#endif
}

void flight_recorder_periodic()
{
#if FLIGHTRECORDER_SDLOG
  // test if sd log is ready
  if (flightRecorderLogFile == -1) return;
#endif

#if PERIODIC_TELEMETRY
  periodic_telemetry_send_FlightRecorder(DefaultPeriodic, &pprzlog_tp.trans_tx, &(FLIGHTRECORDER_DEVICE).device);
#endif
}


