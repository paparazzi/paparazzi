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

/**
 * @file subsystems/datalink/telemetry.c
 *
 * Periodic telemetry system utility function.
 *
 */

#include "subsystems/datalink/telemetry_common.h"
#include "generated/periodic_telemetry.h"

/* Implement global structures from generated header.
 * Can register up to #TELEMETRY_NB_CBS callbacks per periodic message.
 */
struct telemetry_cb_slots telemetry_cbs[TELEMETRY_NB_MSG] = TELEMETRY_CBS_NULL;
struct periodic_telemetry pprz_telemetry = { TELEMETRY_NB_MSG, telemetry_cbs };


/** Register a telemetry callback function.
 * @param _pt periodic telemetry structure to register
 * @param _msgn message name as defined in telemetry xml file with prepended TELEMETRY_MSG_
 * @param _cb callback function, called according to telemetry mode and specified period
 * @return -1 on failure to register, index of callback otherwise
 */
int8_t register_periodic_telemetry(struct periodic_telemetry *_pt, int8_t _msgn, telemetry_cb _cb)
{
  uint8_t i;
  // return FALSE if NULL is passed as periodic_telemetry
  if (_pt == NULL) { return -1; }
  // return FALSE if message number is not existing
  if (_msgn > _pt->nb || _msgn == -1) { return -1; }
  // register another callback if not all TELEMETRY_NB_CBS slots taken
  for (i = 0; i < TELEMETRY_NB_CBS; i++) {
    if (_pt->cbs[_msgn].slots[i] == NULL) {
      _pt->cbs[_msgn].slots[i] = _cb;
      return i;
    }
  }
  // message matched but no more empty slots available
  return -1;
}

#if USE_PERIODIC_TELEMETRY_REPORT

#include "subsystems/datalink/downlink.h"

/** Send an error report when trying to send message that as not been register
 * @param _process telemetry process id
 * @param _mode telemetry mode
 * @param _id id of the message in telemetry system (see var/<AC>/generated/periodic_telemetry.h)
 */
void periodic_telemetry_err_report(uint8_t _process, uint8_t _mode, uint8_t _id)
{
  uint8_t process = _process;
  uint8_t mode = _mode;
  uint8_t id = _id;
  DOWNLINK_SEND_PERIODIC_TELEMETRY_ERR(DefaultChannel, DefaultDevice, &process, &mode, &id);
}

#endif
