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

#define MODULES_C
#include "generated/modules.h"



/** Fly by wire modes */
typedef enum {FBW_MODE_MANUAL = 0, FBW_MODE_AUTO = 1, FBW_MODE_FAILSAFE = 2} fbw_mode_enum;
fbw_mode_enum fbw_mode;

/* MODULES_FREQUENCY is defined in generated/modules.h
 * according to main_freq parameter set for modules in airframe file
 */
//PRINT_CONFIG_VAR(MODULES_FREQUENCY)

tid_t main_periodic_tid; ///< id for main_periodic() timer
tid_t modules_tid;     ///< id for modules_periodic_task() timer
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

  actuators_init();

  electrical_init();

#if USE_MOTOR_MIXING
  motor_mixing_init();
#endif

  radio_control_init();

  modules_init();

  mcu_int_enable();

  intermcu_init();

  // register the timers for the periodic functions
  main_periodic_tid = sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  modules_tid = sys_time_register_timer(1. / MODULES_FREQUENCY, NULL);
  radio_control_tid = sys_time_register_timer((1. / 60.), NULL);
  electrical_tid = sys_time_register_timer(0.1, NULL);
  telemetry_tid = sys_time_register_timer((1. / 10.), NULL);
}


//////////////////////////
// PERIODIC

STATIC_INLINE void handle_periodic_tasks(void)
{


  if (sys_time_check_and_ack_timer(main_periodic_tid)) {
    main_periodic();
  }
  if (sys_time_check_and_ack_timer(modules_tid)) {

    modules_periodic_task();
  }
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
  bool ap_lost = (intermcu.status == INTERMCU_LOST);
  bool rc_lost = (radio_control.status == RC_REALLY_LOST);
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

#ifdef BOARD_PX4IO
  //due to a baud rate issue on PX4, for a few seconds the baud is 1500000 however this may result in package loss, causing the motors to spin at random
  //to prevent this situation:
  if (intermcu.stable_px4_baud != PPRZ_BAUD) {
    fbw_mode = FBW_MODE_FAILSAFE;
    autopilot_motors_on = false;
    //signal to user whether fbw can be flashed:
#ifdef FBW_MODE_LED
    LED_OFF(FBW_MODE_LED); // causes really fast blinking
#endif
  }
#endif

  // TODO make module out of led blink?
#ifdef FBW_MODE_LED
  static uint16_t dv = 0;
#endif
  /* set failsafe commands     */
  if (fbw_mode == FBW_MODE_FAILSAFE) {
    autopilot_motors_on = false;
    SetCommands(commands_failsafe);

#ifdef FBW_MODE_LED
    if (!(dv++ % (PERIODIC_FREQUENCY / 20))) { LED_TOGGLE(FBW_MODE_LED);}
  } else if (fbw_mode == FBW_MODE_MANUAL) {
    if (!(dv++ % (PERIODIC_FREQUENCY))) { LED_TOGGLE(FBW_MODE_LED);}
  } else if (fbw_mode == FBW_MODE_AUTO) {
    intermcu_blink_fbw_led(dv++);
#endif // FWB_MODE_LED
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
    //TODO, check whether the aircraft can actually be flown in manual mode
    //most rotory aircraft can't, at least not without additional IMU aid
    //for now, just turn set to failsafe instead of manual mode.
    fbw_mode = FBW_MODE_FAILSAFE;
  } else {
    fbw_mode = FBW_MODE_AUTO;
  }

  /* Trying to switch to auto when AP is lost */
  if ((intermcu.status == INTERMCU_LOST) &&
      (fbw_mode == FBW_MODE_AUTO)) {
    fbw_mode = AP_LOST_FBW_MODE;
  }

  /* if manual */
  if (fbw_mode == FBW_MODE_MANUAL) {
    autopilot_motors_on = true;
    SetCommands(commands_failsafe);
#ifdef SetCommandsFromRC
    SetCommandsFromRC(commands, radio_control.values);
#else
#warning "FBW: needs commands from RC in order to be useful."
#endif
  }

  /* Forward radiocontrol to AP */
  intermcu_on_rc_frame(fbw_mode);
}

static void autopilot_on_ap_command(void)
{
  if (fbw_mode != FBW_MODE_MANUAL) {
    SetCommands(intermcu_commands);
  } else if (fbw_mode == FBW_MODE_AUTO) {
    autopilot_motors_on = true;
  } else {
    autopilot_motors_on = false;
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

  //Modules
  modules_event_task();
}
