/*
 * Copyright (C) 2015 The Paparazzi Team
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file firmwares/rotorcraft/main_fbw.c
 *
 * Rotorcraft FBW main loop.
 */

#include <inttypes.h>
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#if USE_MOTOR_MIXING
#include "subsystems/actuators/motor_mixing.h"
#endif


#include "subsystems/electrical.h"

#include "subsystems/radio_control.h"

#include "subsystems/intermcu/intermcu_fbw.h"

#include "firmwares/rotorcraft/main_fbw.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"

//#include "generated/modules.h"

/** Fly by wire modes */
typedef enum {FBW_MODE_MANUAL = 0, FBW_MODE_AUTO = 1, FBW_MODE_FAILSAFE = 2} fbw_mode_enum;
fbw_mode_enum fbw_mode;

/* MODULES_FREQUENCY is defined in generated/modules.h
 * according to main_freq parameter set for modules in airframe file
 */
//PRINT_CONFIG_VAR(MODULES_FREQUENCY)

tid_t main_periodic_tid; ///< id for main_periodic() timer
//tid_t modules_tid;     ///< id for modules_periodic_task() timer
tid_t radio_control_tid; ///< id for radio_control_periodic_task() timer
tid_t electrical_tid;    ///< id for electrical_periodic() timer
tid_t telemetry_tid;     ///< id for telemetry_periodic() timer

#ifndef SITL
int main(void)
{
  main_init();

  while (1) {
    handle_periodic_tasks();
    main_event();
  }

  return 0;
}
#endif /* SITL */

STATIC_INLINE void main_init(void)
{
  // fbw_init
  fbw_mode = FBW_MODE_FAILSAFE;

  mcu_init();

  electrical_init();

  actuators_init();
#if USE_MOTOR_MIXING
  motor_mixing_init();
#endif

  radio_control_init();

  // TODO
  //modules_init();

  mcu_int_enable();

  intermcu_init();

  // register the timers for the periodic functions
  main_periodic_tid = sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
//  modules_tid = sys_time_register_timer(1. / MODULES_FREQUENCY, NULL);
  radio_control_tid = sys_time_register_timer((1. / 60.), NULL);
  electrical_tid = sys_time_register_timer(0.1, NULL);
  telemetry_tid = sys_time_register_timer((1. / 20.), NULL);
}


//////////////////////////
// PERIODIC

STATIC_INLINE void handle_periodic_tasks(void)
{
  if (sys_time_check_and_ack_timer(main_periodic_tid)) {
    main_periodic();
  }
  //if (sys_time_check_and_ack_timer(modules_tid)) {
  // TODO
  //modules_periodic_task();
  //}
  if (sys_time_check_and_ack_timer(radio_control_tid)) {
    radio_control_periodic_task();
  }
  if (sys_time_check_and_ack_timer(electrical_tid)) {
    electrical_periodic();
  }
  if (sys_time_check_and_ack_timer(telemetry_tid)) {
    telemetry_periodic();
  }
}

STATIC_INLINE void telemetry_periodic(void)  // 60Hz
{
  /* Send status to AP */
  intermcu_send_status(fbw_mode);

  /* Handle Modems */
  // TODO
  // Read Telemetry
}

STATIC_INLINE void main_periodic(void)
{
  /* Inter-MCU watchdog */
  intermcu_periodic();

  /* Safety logic */
  bool_t ap_lost = (inter_mcu.status == INTERMCU_LOST);
  bool_t rc_lost = (radio_control.status == RC_REALLY_LOST);
  if (rc_lost) {
    if (ap_lost) {
      // Both are lost
      fbw_mode = FBW_MODE_FAILSAFE;
    } else {
      if (fbw_mode == FBW_MODE_MANUAL) {
        fbw_mode = RC_LOST_FBW_MODE;
      } else {
        if (fbw_mode == FBW_MODE_FAILSAFE) {
          // No change: failsafe stays failsafe
        } else {
          // Lost RC while in working Auto mode
          fbw_mode = RC_LOST_IN_AUTO_FBW_MODE;
        }
      }
    }
  } else { // rc_is_good
    if (fbw_mode == FBW_MODE_AUTO) {
      if (ap_lost) {
        fbw_mode = AP_LOST_FBW_MODE;
      }
    }
  }

  /* set failsafe commands     */
  if (fbw_mode == FBW_MODE_FAILSAFE) {
    SetCommands(commands_failsafe);
  }

  /* set actuators     */
  SetActuatorsFromCommands(commands, autopilot_mode);

  /* blink     */
  RunOnceEvery(10, LED_PERIODIC());
}


///////////////////////
// Event

static void autopilot_on_rc_frame(void)
{
  /* get autopilot fbw mode as set by RADIO_MODE 3-way switch */
  if (radio_control.values[RADIO_FBW_MODE] < (MIN_PPRZ / 2)) {
    fbw_mode = FBW_MODE_MANUAL;
  } else {
    fbw_mode = FBW_MODE_AUTO;
  }

  /* Trying to switch to auto when AP is lost */
  if ((inter_mcu.status == INTERMCU_LOST) &&
      (fbw_mode == FBW_MODE_AUTO)) {
    fbw_mode = AP_LOST_FBW_MODE;
  }

  /* if manual */
  if (fbw_mode == FBW_MODE_MANUAL) {
#ifdef SetCommandsFromRC
    SetCommandsFromRC(commands, radio_control.values);
#else
#warning "FBW: needs commands from RC in order to be useful."
#endif
  }

  /* Forward radiocontrol to AP */
  intermcu_on_rc_frame();
}

static void autopilot_on_ap_command(void)
{
  if (fbw_mode != FBW_MODE_MANUAL) {
    SetCommands(intermcu_commands);
  }
}

STATIC_INLINE void main_event(void)
{
  /* event functions for mcu peripherals: i2c, usb_serial.. */
  mcu_event();

  // Handle RC
  RadioControlEvent(autopilot_on_rc_frame);

  // InterMCU
  InterMcuEvent(autopilot_on_ap_command);

  // TODO Modules
  //modules_event_task();
}
