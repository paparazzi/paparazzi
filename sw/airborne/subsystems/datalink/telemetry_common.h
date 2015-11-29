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
#include "messages.h"

/** Telemetry callback definition
 */
typedef void (*telemetry_cb)(struct transport_tx *trans, struct link_device *dev);

/** periodic telemetry msg name definition
 */
typedef const char telemetry_msg[64];

/** number of callbacks that can be registered per msg */
#define TELEMETRY_NB_CBS 4

struct telemetry_cb_slots {
  uint8_t id;  ///< id of telemetry message
  telemetry_cb slots[TELEMETRY_NB_CBS];
};

/** Periodic telemetry structure.
 *  Contains the total number of messages (from generated telemetry file)
 *  and the list of registered callbacks
 */
struct periodic_telemetry {
  uint8_t nb;                     ///< number of messages
  struct telemetry_cb_slots *cbs; ///< array of callbacks defined through TELEMETRY_MSG
};

/** Register a telemetry callback function.
 * empty implementation is provided if PERIODIC_TELEMETRY is not set or set to FALSE
 * @param _pt periodic telemetry structure to register
 * @param _id message ID (use PPRZ_MSG_ID_<message_name> define)
 * @param _cb callback function, called according to telemetry mode and specified period
 * @return -1 on failure to register, index of callback otherwise
 */
#if PERIODIC_TELEMETRY
extern int8_t register_periodic_telemetry(struct periodic_telemetry *_pt, uint8_t _id, telemetry_cb _cb);
#else
static inline int8_t register_periodic_telemetry(struct periodic_telemetry *_pt __attribute__((unused)),
    uint8_t _id __attribute__((unused)), telemetry_cb _cb __attribute__((unused))) { return -1; }
#endif

#if USE_PERIODIC_TELEMETRY_REPORT
/** Send an error report when trying to send message that as not been register
 * @param _process telemetry process id
 * @param _mode telemetry mode
 * @param _id id of the message
 */
extern void periodic_telemetry_err_report(uint8_t _process, uint8_t _mode, uint8_t _id);
#endif

#endif /* TELEMETRY_COMMON_H */

