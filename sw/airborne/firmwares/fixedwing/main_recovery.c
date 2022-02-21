/*
 * Copyright (C) 2022 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file firmwares/fixedwing/main_recovery.c
 *
 * Recovery mode: run manual mode in case of hardfault
 * Based on legacy FBW
 *
 */

#include "firmwares/fixedwing/main_recovery.h"
#include "generated/airframe.h"
#include "generated/modules.h"
#include "modules/core/abi.h"
#include "modules/radio_control/radio_control.h"
#include "modules/core/commands.h"
#include "modules/energy/electrical.h"
#include "autopilot_utils.h"
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

uint8_t recovery_mode;

tid_t periodic_tid;     ///< id for periodic task timer
tid_t electrical_tid;   ///< id for electrical_periodic() timer
#if PERIODIC_TELEMETRY
tid_t telemetry_tid;    ///< id for periodic telemetry
#endif

#ifndef RECOVERY_RC_ID
#define RECOVERY_RC_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(RECOVERY_RC_ID)
static abi_event rc_ev;

#define RECOVERY_MODE_OF_PPRZ(mode) ((mode) < THRESHOLD_MANUAL_PPRZ ? RECOVERY_MODE_MANUAL : RECOVERY_MODE_FAILSAFE)

static void rc_cb(uint8_t __attribute__((unused)) sender_id,
                  struct RadioControl *rc)
{
  recovery_mode = RECOVERY_MODE_OF_PPRZ(rc->values[RADIO_MODE]);
  if (recovery_mode == RECOVERY_MODE_MANUAL && rc->status == RC_OK) {
    // on RC callback in manual, set commands from RC
    SetCommandsFromRC(commands, rc->values);
  }
}

/********** PERIODIC MESSAGES ************************************************/
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_fbw_status(struct transport_tx *trans, struct link_device *dev)
{
#ifdef RADIO_CONTROL
  uint8_t rc_status = radio_control.status;
  uint8_t rc_rate = radio_control.frame_rate;
#else
  uint8_t rc_status = 0;
  uint8_t rc_rate = 0;
#endif
  pprz_msg_send_FBW_STATUS(trans, dev, AC_ID,
                           &rc_status, &rc_rate, &recovery_mode,
                           &electrical.vsupply, &electrical.current);
}

#endif /* PERIODIC_TELEMETRY */

/********** INIT *************************************************************/
void main_recovery_init(void)
{
  // mcu init done in main

  // don't call all core modules, only a subset
  electrical_init();
  commands_init();

  modules_radio_control_init();
  modules_actuators_init();
  modules_datalink_init();

  recovery_mode = RECOVERY_MODE_FAILSAFE;

  /**** start timers for periodic functions *****/
  periodic_tid = sys_time_register_timer((1. / 60.), NULL);
  electrical_tid = sys_time_register_timer(0.1, NULL);

  // actuators periodic will update the actuators
  // but the autopilot.mode is passed to SetActuatorsFromCommands
  // it is set to manual if possible, otherwise behavior is undefined
#ifdef AP_MODE_MANUAL
  autopilot.mode = AP_MODE_MANUAL;
#endif

  // Bind to RC event
  AbiBindMsgRADIO_CONTROL(RECOVERY_RC_ID, &rc_ev, rc_cb);

#if PERIODIC_TELEMETRY
  telemetry_tid = sys_time_register_timer((1. / TELEMETRY_FREQUENCY), NULL);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FBW_STATUS, send_fbw_status);
#endif /* PERIODIC_TELEMETRY */

}

/********** EVENT ************************************************************/

void main_recovery_event(void)
{
  modules_mcu_event_task();
  modules_radio_control_event_task();
  modules_actuators_event_task();
  modules_datalink_event_task();
}

/************* PERIODIC ******************************************************/

void main_recovery_periodic(void)
{
  if (sys_time_check_and_ack_timer(periodic_tid)) {
    modules_mcu_periodic_task();
    modules_radio_control_periodic_task();
    // if RC is lost or not in manual, set failsafe commands
    if (radio_control.status == RC_REALLY_LOST ||
        recovery_mode == RECOVERY_MODE_FAILSAFE) {
      recovery_mode = RECOVERY_MODE_FAILSAFE;
      SetCommands(commands_failsafe);
    }
    modules_actuators_periodic_task();
  }

#if !(RECOVERY_DISABLE_ELECTRICAL)
  if (sys_time_check_and_ack_timer(electrical_tid)) {
    electrical_periodic();
  }
#endif

#if PERIODIC_TELEMETRY
  if (sys_time_check_and_ack_timer(telemetry_tid)) {
    // only fbw part during recovery
    periodic_telemetry_send_Fbw(DefaultPeriodic, &(DefaultChannel).trans_tx, &(DefaultDevice).device);
  }
#endif

}

