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
 * @file modules/datalink/telemetry.c
 *
 * Periodic telemetry system utility function.
 *
 */

#include "modules/datalink/telemetry_common.h"
#include "modules/datalink/telemetry.h"
#include "generated/periodic_telemetry.h"
#if FIXEDWING_FIRMWARE || ROTORCRAFT_FIRMWARE || ROVER_FIRMWARE
#include "autopilot.h"
#endif


/* Implement global structures from generated header.
 * Can register up to #TELEMETRY_NB_CBS callbacks per periodic message.
 */
struct telemetry_cb_slots telemetry_cbs[TELEMETRY_PPRZ_NB_MSG] = TELEMETRY_PPRZ_CBS;
struct periodic_telemetry pprz_telemetry = { TELEMETRY_PPRZ_NB_MSG, telemetry_cbs };


/** Register a telemetry callback function.
 * @param _pt periodic telemetry structure to register
 * @param _id message ID (use PPRZ_MSG_ID_<message_name> define)
 * @param _cb callback function, called according to telemetry mode and specified period
 * @return -1 on failure to register, index of callback otherwise
 */
int8_t register_periodic_telemetry(struct periodic_telemetry *_pt, uint8_t _id, telemetry_cb _cb)
{
  uint8_t i, j;
  // return if NULL is passed as periodic_telemetry
  if (_pt == NULL) { return -1; }
  // check if message with id _msgn has a periodic entery in telemetry file
  for (i = 0; i < _pt->nb; i++) {
    if (_pt->cbs[i].id == _id) {
      // msg found, register another callback if not all TELEMETRY_NB_CBS slots taken
      for (j = 0; j < TELEMETRY_NB_CBS; j++) {
        if (_pt->cbs[i].slots[j] == NULL) {
          _pt->cbs[i].slots[j] = _cb;
          return j;
        }
      }
      // message matched but no more empty slots available
      return -1;
    }
  }
  // message is not in telemetry file
  return -1;
}

/** Peridioc task
 * Send a series of initialisation messages followed by a stream of periodic ones.
 */
void telemetry_reporting_task(void)
{
#if FIXEDWING_FIRMWARE || ROTORCRAFT_FIRMWARE || ROVER_FIRMWARE
  static uint8_t boot = true;

  /* initialisation phase during boot */
  if (boot) {
#if DOWNLINK && !(defined INTERMCU_FBW)
    autopilot_send_version();
#endif
    boot = false;
  }
  /* then report periodicly */
  else {
#if FIXEDWING_FIRMWARE
#if AP
    periodic_telemetry_send_Ap(DefaultPeriodic, &(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif // AP
#if FBW || !TELEMETRY_INTERMCU // FIXME for now send both AP and FBW process if needed
    periodic_telemetry_send_Fbw(DefaultPeriodic, &(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif //FBW
#else // ROTORCRAFT && ROVER
    periodic_telemetry_send_Main(DefaultPeriodic, &(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif
  }
#endif
}


#if USE_PERIODIC_TELEMETRY_REPORT

#include "modules/datalink/downlink.h"

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
