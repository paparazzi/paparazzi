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
#include "led.h"
#include "mcu_periph/sys_time.h"

#include "modules/core/commands.h"
#include "subsystems/actuators.h"
#if USE_MOTOR_MIXING
#include "subsystems/actuators/motor_mixing.h"
#endif

#include "modules/energy/electrical.h"
#include "modules/radio_control/radio_control.h"
#include "subsystems/intermcu/intermcu_fbw.h"
#include "firmwares/rotorcraft/main_fbw.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"

#define MODULES_C
#include "generated/modules.h"

/* So one can use these in command_laws section */
#define And(x, y) ((x) && (y))
#define Or(x, y) ((x) || (y))
#define Min(x,y) (x < y ? x : y)
#define Max(x,y) (x > y ? x : y)
#define LessThan(_x, _y) ((_x) < (_y))
#define MoreThan(_x, _y) ((_x) > (_y))


/** Fly by wire modes */
fbw_mode_enum fbw_mode;
bool fbw_motors_on = false;

/* MODULES_FREQUENCY is defined in generated/modules.h
 * according to main_freq parameter set for modules in airframe file
 */
PRINT_CONFIG_VAR(MODULES_FREQUENCY)

tid_t main_periodic_tid; ///< id for main_periodic() timer
tid_t modules_tid;     ///< id for modules_periodic_task() timer
tid_t radio_control_tid; ///< id for radio_control_periodic_task() timer
tid_t electrical_tid;    ///< id for electrical_periodic() timer
tid_t telemetry_tid;     ///< id for telemetry_periodic() timer


/** Main initialization */
void main_init(void)
{
  // Set startup mode to Failsafe
  fbw_mode = FBW_MODE_FAILSAFE;

  mcu_init();

  actuators_init();

  electrical_init();

#if USE_MOTOR_MIXING
  motor_mixing_init();
#endif

  radio_control_init();

  modules_init();


  intermcu_init();

  // Register the timers for the periodic functions
  main_periodic_tid = sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  modules_tid = sys_time_register_timer(1. / MODULES_FREQUENCY, NULL);
  radio_control_tid = sys_time_register_timer((1. / 60.), NULL);
  electrical_tid = sys_time_register_timer(0.1, NULL);
  telemetry_tid = sys_time_register_timer((1. / 10.), NULL);
}


//////////////////////////
// PERIODIC

void handle_periodic_tasks(void)
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

void telemetry_periodic(void)
{
  /* Send status to AP */
  intermcu_send_status(fbw_mode);

  /* Handle Modems */
  // TODO
}

/* Checks the different safety cases and sets the correct FBW mode */
static void fbw_safety_check(void)
{
  /* Safety logic */
  bool ap_lost = (intermcu.status == INTERMCU_LOST);
  bool rc_lost = (radio_control.status == RC_REALLY_LOST);

  // Both the RC and AP are lost
  if (rc_lost && ap_lost) {
      fbw_mode = FBW_MODE_FAILSAFE;
  }
  // RC is valid but lost AP
  else if (!rc_lost && ap_lost) {
    // Only crucial when AP was in control
    if (fbw_mode == FBW_MODE_AUTO) {
      fbw_mode = AP_LOST_FBW_MODE;
    }
  }
  // RC is lost but AP is valid
  else if (rc_lost && !ap_lost) {

    // Lost RC while flying in manual trough FBW
    if (fbw_mode == FBW_MODE_MANUAL) {
      fbw_mode = RC_LOST_FBW_MODE;
    }
    // Allways keep failsafe when RC is lost
    else if (fbw_mode == FBW_MODE_FAILSAFE) {
        // No change: failsafe stays failsafe
    }
    // Lost RC while in working Auto mode
    else {
      fbw_mode = RC_LOST_IN_AUTO_FBW_MODE;
    }
  }
}

/* Sets the actual actuator commands */
void main_periodic(void)
{
  /* Inter-MCU watchdog */
  intermcu_periodic();

  /* Safety check and set FBW mode */
  fbw_safety_check();

#ifdef BOARD_PX4IO
  //due to a baud rate issue on PX4, for a few seconds the baud is 1500000 however this may result in package loss, causing the motors to spin at random
  //to prevent this situation:
  if (intermcu.stable_px4_baud != PPRZ_BAUD) {
    fbw_mode = FBW_MODE_FAILSAFE;
    fbw_motors_on = false;
    //signal to user whether fbw can be flashed:
#ifdef FBW_MODE_LED
    LED_OFF(FBW_MODE_LED); // causes really fast blinking
#endif
  }
#endif

  // TODO make module out of led blink?
#ifdef FBW_MODE_LED
  static uint16_t dv = 0;
  if (fbw_mode == FBW_MODE_FAILSAFE) {
    if (!(dv++ % (PERIODIC_FREQUENCY / 20))) { LED_TOGGLE(FBW_MODE_LED);}
  } else if (fbw_mode == FBW_MODE_MANUAL) {
    if (!(dv++ % (PERIODIC_FREQUENCY))) { LED_TOGGLE(FBW_MODE_LED);}
  } else if (fbw_mode == FBW_MODE_AUTO) {
    LED_ON(FBW_MODE_LED);
  }
#endif // FWB_MODE_LED

  /* Set failsafe commands */
  if (fbw_mode == FBW_MODE_FAILSAFE) {
    fbw_motors_on = false;
    SetCommands(commands_failsafe);
  }

  /* If in auto copy autopilot motors on */
  if (fbw_mode == FBW_MODE_AUTO) {
    fbw_motors_on = autopilot_motors_on;
  }

  /* Set actuators */
  SetActuatorsFromCommands(commands, autopilot_get_mode());

  /* Periodic blinking */
  RunOnceEvery(10, LED_PERIODIC());
}


///////////////////////
// Event

/** Callback when we received an RC frame */
static void fbw_on_rc_frame(void)
{
  /* get autopilot fbw mode as set by RADIO_MODE 3-way switch */
  if (radio_control.values[RADIO_FBW_MODE] < (MIN_PPRZ / 2) && !FBW_MODE_AUTO_ONLY) {

#ifdef RADIO_KILL_SWITCH
    static bool  kill_state_init = false; // require a kill == off before enabling engines with kill == on
    if (radio_control.values[RADIO_KILL_SWITCH] < (MIN_PPRZ / 2)) {
      fbw_mode = FBW_MODE_FAILSAFE;
      kill_state_init = true;
    } else {
      if (kill_state_init)
        fbw_mode = FBW_MODE_MANUAL;
      else
        fbw_mode = FBW_MODE_FAILSAFE;
    }
#else
      fbw_mode = FBW_MODE_MANUAL;
#endif

  } else {
    fbw_mode = FBW_MODE_AUTO;
  }

  /* Failsafe check if intermcu is lost while AP was in control */
  if ((intermcu.status == INTERMCU_LOST) &&
      (fbw_mode == FBW_MODE_AUTO)) {
    fbw_mode = AP_LOST_FBW_MODE;
  }

  /* If the FBW is in control */
  if (fbw_mode == FBW_MODE_MANUAL) {
    fbw_motors_on = true;
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

/** Callback when receive commands from the AP */
static void fbw_on_ap_command(void)
{
  // Only set the command from AP when we are in AUTO mode
  if (fbw_mode == FBW_MODE_AUTO) {
    SetCommands(intermcu_commands);
  }
}

void main_event(void)
{
  /* Event functions for mcu peripherals: i2c, usb_serial.. */
  mcu_event();

  /* Handle RC */
  RadioControlEvent(fbw_on_rc_frame);

  /* InterMCU (gives autopilot commands as output) */
  InterMcuEvent(fbw_on_ap_command);

  /* FBW modules */
  modules_event_task();
}
