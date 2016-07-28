/*
 * Copyright (C) 2016 Michal Podhradsky <http://github.com/podhrmic>
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
 * @file "modules/loggers/flight_logger.c"
 * @author Michal Podhradsky <http://github.com/podhrmic>
 * Send telemetry messages over a serial port to external logger.
 *
 *
 * Not needed unless we want to define our own messages - in such case you would define the two
 * structs below and then define individual messages and registered them in the same way as
 * periodic telemetry.
 * struct telemetry_cb_slots telemetry_cbs_logger[TELEMETRY_PPRZ_NB_MSG] = TELEMETRY_PPRZ_CBS;
 * struct periodic_telemetry logger_telemetry = { TELEMETRY_PPRZ_NB_MSG, telemetry_cbs_logger };
 *
 * The registration should be done in the init function:
 * register_periodic_telemetry(&logger_telemetry, PPRZ_MSG_ID_xxx, send_xxx_message);
 *
 * Two extra notes for the periodic function:
 * 1) this sends registered messages from FlightRecorder process (as mentioned above) over dedicated port do:
 *    periodic_telemetry_send_FlightRecorder(&logger_telemetry, &pprz_tp_logger.trans_tx, &(FLIGHT_LOGGER_PORT).device);
 * 2) to send FlightRecorder telemetry messages over default channel just change to:
 *    periodic_telemetry_send_FlightRecorder(DefaultPeriodic, &(DefaultChannel).trans_tx, &(DefaultDevice).device);
 */

#define PERIODIC_C_FLIGHTRECORDER

#include "modules/loggers/flight_logger.h"
#include "subsystems/datalink/telemetry.h"

// transport struct
struct pprz_transport pprz_tp_logger;

void flight_logger_init(void)
{
  pprz_transport_init(&pprz_tp_logger);
}

void flight_logger_periodic(void)
{
#if PERIODIC_TELEMETRY
  // send periodic messages as defined in the FlightRecorder process, we are using DefaultPeriodic so we can send standard messages
  periodic_telemetry_send_FlightRecorder(DefaultPeriodic, &pprz_tp_logger.trans_tx, &(FLIGHTLOGGER_PORT).device);
#endif
}


