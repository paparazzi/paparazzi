/*
 * Copyright (C) 2015 Felix Ruess <felix.ruess@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file firmwares/demo/demo_ahrs_actuators.c
 *  Demo prog with ahrs and simple roll/pitch commands to actuators.
 */

#include <inttypes.h>

/* PERIODIC_C_MAIN is defined before generated/periodic_telemetry.h
 * in order to implement telemetry_mode_Main_*
 */
#define PERIODIC_C_MAIN
#define ABI_C

#include "subsystems/datalink/telemetry.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/abi.h"

#include "generated/airframe.h"
#include "generated/settings.h"

#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "state.h"
#include "subsystems/imu.h"
#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_aligner.h"

#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#include "subsystems/settings.h"

#include "pprz_version.h"

#ifndef DEMO_MAX_ROLL
#define DEMO_MAX_ROLL RadOfDeg(65)
#endif

#ifndef DEMO_MAX_PITCH
#define DEMO_MAX_PITCH RadOfDeg(65)
#endif

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);

static void send_alive(struct transport_tx *trans, struct link_device *dev);
static void send_autopilot_version(struct transport_tx *trans, struct link_device *dev);
static void send_actuators(struct transport_tx *trans, struct link_device *dev);
static void send_commands(struct transport_tx *trans, struct link_device *dev);

int main(void)
{
  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
    main_event_task();
  }
  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);

  stateInit();
  actuators_init();

  imu_init();
#if USE_AHRS_ALIGNER
  ahrs_aligner_init();
#endif
  ahrs_init();

  settings_init();

  mcu_int_enable();

  downlink_init();

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AUTOPILOT_VERSION, send_autopilot_version);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ALIVE, send_alive);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_COMMANDS, send_commands);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACTUATORS, send_actuators);

  // send body_to_imu from here for now
  AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
}

static inline void main_periodic_task(void)
{
  /* Simply set current roll/pitch as commands.
   * Scale DEMO_MAX_ROLL/PITCH to MAX_PPRZ (the max commands)
   */
  commands[COMMAND_ROLL] = stateGetNedToBodyEulers_f()->phi * MAX_PPRZ / DEMO_MAX_ROLL;
  commands[COMMAND_PITCH] = stateGetNedToBodyEulers_f()->theta * MAX_PPRZ / DEMO_MAX_ROLL;

  /* generated macro from airframe file, seconds AP_MODE param not used */
  SetActuatorsFromCommands(commands, 0);

  if (sys_time.nb_sec > 1) {
    imu_periodic();
  }
  RunOnceEvery(10, { LED_PERIODIC();});
  RunOnceEvery(PERIODIC_FREQUENCY, { datalink_time++; });
  periodic_telemetry_send_Main(DefaultPeriodic, &(DefaultChannel).trans_tx, &(DefaultDevice).device);
}

static inline void main_event_task(void)
{
  mcu_event();
  ImuEvent();
  DatalinkEvent();
}

static void send_alive(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ALIVE(trans, dev, AC_ID, 16, MD5SUM);
}

void send_autopilot_version(struct transport_tx *trans, struct link_device *dev)
{
  static uint32_t ap_version = PPRZ_VERSION_INT;
  static char *ver_desc = PPRZ_VERSION_DESC;
  pprz_msg_send_AUTOPILOT_VERSION(trans, dev, AC_ID, &ap_version, strlen(ver_desc), ver_desc);
}

static void send_actuators(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ACTUATORS(trans, dev, AC_ID , ACTUATORS_NB, actuators);
}

static void send_commands(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_COMMANDS(trans, dev, AC_ID, COMMANDS_NB, commands);
}
