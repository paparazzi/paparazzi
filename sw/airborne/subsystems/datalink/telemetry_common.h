/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 *
 */

/** @file subsystems/datalink/telemetry_common.h
 *
 *  Common tools for periodic telemetry interface
 *  Allows subsystem to register callback functions
 */

#ifndef TELEMETRY_COMMON_H
#define TELEMETRY_COMMON_H

#include <inttypes.h>
#include "std.h"
#include "mcu_periph/link_device.h"
#include "subsystems/datalink/transport.h"

/** Set default periodic telemetry to NULL
 */
#ifndef DefaultPeriodic
#define DefaultPeriodic NULL
#endif

/** Telemetry callback definition
 */
typedef void (*telemetry_cb)(struct transport_tx *trans, struct link_device *dev);

/** Telemetry header
 */
struct telemetry_msg {
  char msg[64];     ///< name in telemetry xml file
  telemetry_cb cb;  ///< callback funtion
};

/** Telemetry structure.
 *  Contains the total number of messages (from generated telemetry file)
 *  and the list of registered callbacks
 */
struct pprz_telemetry {
  uint8_t nb;                 ///< number of messages
  struct telemetry_msg* msgs; ///< the list of (msg name, callbacks)
};

/** Register a telemetry callback function.
 * empty implementation is provided if PERIODIC_TELEMETRY is not set or set to FALSE
 * @param _pt periodic telemetry structure to register
 * @param _msg message name (string) as defined in telemetry xml file
 * @param _cb callback function, called according to telemetry mode and specified period
 * @return TRUE if message registered with success, FALSE otherwise
 */
#if PERIODIC_TELEMETRY
extern bool_t register_periodic_telemetry(struct pprz_telemetry * _pt, const char * _msg, telemetry_cb _cb);
#else
static inline bool_t register_periodic_telemetry(struct pprz_telemetry * _pt __attribute__((unused)), const char * _msg __attribute__((unused)), telemetry_cb _cb __attribute__((unused))) { return FALSE; }
#endif

#if USE_PERIODIC_TELEMETRY_REPORT
/** Send an error report when trying to send message that as not been register
 * @param _process telemetry process id
 * @param _mode telemetry mode
 * @param _id id of the message in telemetry system (see var/<AC>/generated/periodic_telemetry.h)
 */
extern void periodic_telemetry_err_report(uint8_t _process, uint8_t _mode, uint8_t _id);
#endif

#endif /* TELEMETRY_COMMON_H */

