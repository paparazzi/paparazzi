/*
 * Copyright (C) 2015 The Paparazzi Team
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
 * @file main_fbw.c
 *
 * FBW main loop.
 */

#define MODULES_C

#define ABI_C

#include "main_fbw.h"
#include "generated/airframe.h"
#include "generated/modules.h"
#include "modules/core/abi.h"
#include "modules/datalink/datalink.h"
#include "modules/datalink/telemetry.h"

/** Fly by wire modes */
uint8_t fbw_mode;
bool fbw_motors_on = false;

/* if PRINT_CONFIG is defined, print some config options */
PRINT_CONFIG_VAR(PERIODIC_FREQUENCY)
/* SYS_TIME_FREQUENCY/PERIODIC_FREQUENCY should be an integer, otherwise the timer will not be correct */
#if !(SYS_TIME_FREQUENCY/PERIODIC_FREQUENCY*PERIODIC_FREQUENCY == SYS_TIME_FREQUENCY)
#warning "The SYS_TIME_FREQUENCY can not be divided by PERIODIC_FREQUENCY. Make sure this is the case for correct timing."
#endif

/* TELEMETRY_FREQUENCY is defined in generated/periodic_telemetry.h
 * defaults to 60Hz or set by TELEMETRY_FREQUENCY configure option in airframe file
 */
PRINT_CONFIG_VAR(TELEMETRY_FREQUENCY)

/**
 * IDs for timers
 */
tid_t periodic_tid;       ///< id for general periodic task timer
tid_t radio_control_tid;  ///< id for radio_control_periodic() timer
tid_t electrical_tid;     ///< id for electrical_periodic() timer
tid_t telemetry_tid;      ///< id for telemetry_periodic() timer

/**
 * ABI RC binding
 */
#ifndef MAIN_FBW_RC_ID
#define MAIN_FBW_RC_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(MAIN_FBW_RC_ID)
static abi_event rc_ev;
static void rc_cb(uint8_t sender_id, struct RadioControl *rc);


/**
 * Main initialization
 */
void main_fbw_init(void)
{
  // mcu init done in main

  modules_core_init();
  modules_radio_control_init();
  modules_actuators_init();
  modules_datalink_init();

  // Set startup mode to Failsafe
  fbw_mode = FBW_MODE_FAILSAFE;

  // Bind to RC event
  AbiBindMsgRADIO_CONTROL(MAIN_FBW_RC_ID, &rc_ev, rc_cb);

  // Register the timers for the periodic functions
  periodic_tid = sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  radio_control_tid = sys_time_register_timer((1. / 60.), NULL);
  electrical_tid = sys_time_register_timer(0.1, NULL);
  telemetry_tid = sys_time_register_timer((1. / TELEMETRY_FREQUENCY), NULL);
}


/**
 * Periodic tasks
 */

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
static void main_task_periodic(void)
{
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

  /* If in auto copy autopilot motors on and commands from intermcu */
  if (fbw_mode == FBW_MODE_AUTO) {
    fbw_motors_on = intermcu_ap_motors_on;
    SetCommands(intermcu_commands);
  }

  /* in MANUAL, commands are updated in RC callback */
}

void main_fbw_periodic(void)
{
  if (sys_time_check_and_ack_timer(radio_control_tid)) {
    modules_radio_control_periodic_task();
  }

  if (sys_time_check_and_ack_timer(periodic_tid)) {
    main_task_periodic();
    modules_actuators_periodic_task();
    modules_mcu_periodic_task();
    modules_core_periodic_task();
  }

  if (sys_time_check_and_ack_timer(electrical_tid)) {
    electrical_periodic();
  }

  if (sys_time_check_and_ack_timer(telemetry_tid)) {
    modules_datalink_periodic_task();
  }
}


///////////////////////
// Event

/** Callback when we received an RC frame */
static void rc_cb(uint8_t sender_id __attribute__((unused)), struct RadioControl *rc __attribute__((unused)))
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
}

void main_fbw_parse_EMERGENCY_CMD(uint8_t *buf)
{
  if (DL_EMERGENCY_CMD_ac_id(buf) == AC_ID && DL_EMERGENCY_CMD_cmd(buf) == 0) {
    fbw_mode = FBW_MODE_FAILSAFE;
  }
}

void main_fbw_event(void)
{
  modules_mcu_event_task();
  intermcu_event();
  modules_radio_control_event_task();
  modules_actuators_event_task();
  modules_datalink_event_task();
}
