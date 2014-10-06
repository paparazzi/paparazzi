/*
 * Copyright (C) 2003-2010 The Paparazzi Team
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
 * @file firmwares/fixedwing/main_fbw.c
 *
 * FBW ( FlyByWire ) process
 *
 * This process is responsible for decoding radio control, generating actuators
 * signals either from the radio control or from the commands provided by the
 * AP (autopilot) process. It also performs a telemetry task and a low level monitoring
 * ( for parameters like the supply )
 */

#include "generated/airframe.h"

#include "firmwares/fixedwing/main_fbw.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#include "subsystems/electrical.h"
#include "subsystems/radio_control.h"
#include "firmwares/fixedwing/autopilot.h"
#include "paparazzi.h"
#include "mcu_periph/i2c.h"

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

#ifdef MCU_SPI_LINK
#include "link_mcu_spi.h"
#endif

#ifdef MCU_UART_LINK
#include "link_mcu_usart.h"
#endif

#ifdef FBW_DATALINK
#include "firmwares/fixedwing/fbw_datalink.h"
#endif

uint8_t fbw_mode;

#include "inter_mcu.h"

#ifdef USE_NPS
#include "nps_autopilot.h"
#endif

/** Trim commands for roll, pitch and yaw.
 * These are updated from the trim commands in ap_state via inter_mcu
 */
pprz_t command_roll_trim;
pprz_t command_pitch_trim;
pprz_t command_yaw_trim;


volatile uint8_t fbw_new_actuators = 0;

tid_t fbw_periodic_tid; ///< id for periodic_task_fbw() timer
tid_t electrical_tid;   ///< id for electrical_periodic() timer

/********** PERIODIC MESSAGES ************************************************/
#if PERIODIC_TELEMETRY
static void send_commands(void) {
  DOWNLINK_SEND_COMMANDS(DefaultChannel, DefaultDevice, COMMANDS_NB, commands);
}

#ifdef RADIO_CONTROL
static void send_fbw_status(void) {
  DOWNLINK_SEND_FBW_STATUS(DefaultChannel, DefaultDevice,
      &(radio_control.status), &(radio_control.frame_rate), &fbw_mode, &electrical.vsupply, &electrical.current);
}

static void send_rc(void) {
  DOWNLINK_SEND_RC(DefaultChannel, DefaultDevice, RADIO_CONTROL_NB_CHANNEL, radio_control.values);
}

#else
static void send_fbw_status(void) {
  uint8_t dummy = 0;
  DOWNLINK_SEND_FBW_STATUS(DefaultChannel, DefaultDevice,
      &dummy, &dummy, &fbw_mode, &electrical.vsupply, &electrical.current);
}
#endif

#ifdef ACTUATORS
static void send_actuators(void) {
  DOWNLINK_SEND_ACTUATORS(DefaultChannel, DefaultDevice , ACTUATORS_NB, actuators);
}
#endif

#endif

/********** INIT *************************************************************/
void init_fbw( void ) {

  mcu_init();

#if !(DISABLE_ELECTRICAL)
  electrical_init();
#endif

#ifdef ACTUATORS
  actuators_init();
  /* Load the failsafe defaults */
  SetCommands(commands_failsafe);
  fbw_new_actuators = 1;
#endif
#ifdef RADIO_CONTROL
  radio_control_init();
#endif
#ifdef INTER_MCU
  inter_mcu_init();
#endif
#ifdef MCU_SPI_LINK
  link_mcu_init();
  link_mcu_restart();
#endif

  fbw_mode = FBW_MODE_FAILSAFE;

  /**** start timers for periodic functions *****/
  fbw_periodic_tid = sys_time_register_timer((1./60.), NULL);
  electrical_tid = sys_time_register_timer(0.1, NULL);

#ifndef SINGLE_MCU
  mcu_int_enable();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(&telemetry_Fbw, "FBW_STATUS", send_fbw_status);
  register_periodic_telemetry(&telemetry_Fbw, "COMMANDS", send_commands);
#ifdef ACTUATORS
  register_periodic_telemetry(&telemetry_Fbw, "ACTUATORS", send_actuators);
#endif
#ifdef RADIO_CONTROL
  register_periodic_telemetry(&telemetry_Fbw, "RC", send_rc);
#endif
#endif

}


static inline void set_failsafe_mode( void ) {
  fbw_mode = FBW_MODE_FAILSAFE;
  SetCommands(commands_failsafe);
  fbw_new_actuators = 1;
}


#ifdef RADIO_CONTROL
static inline void handle_rc_frame( void ) {
  fbw_mode = FBW_MODE_OF_PPRZ(radio_control.values[RADIO_MODE]);
  if (fbw_mode == FBW_MODE_MANUAL)
  {
    SetCommandsFromRC(commands, radio_control.values);
    fbw_new_actuators = 1;
  }
}
#endif

uint8_t ap_has_been_ok = FALSE;
/********** EVENT ************************************************************/

void event_task_fbw( void) {
#ifdef RADIO_CONTROL
  RadioControlEvent(handle_rc_frame);
#endif

  i2c_event();

#ifdef INTER_MCU
#if defined MCU_SPI_LINK | defined MCU_UART_LINK
  link_mcu_event_task();
#endif /* MCU_SPI_LINK */


#if OUTBACK_CHALLENGE_VERY_DANGEROUS_RULE_NO_AP_MUST_FAILSAFE
#warning OUTBACK_CHALLENGE_VERY_DANGEROUS_RULE_NO_AP_MUST_FAILSAFE loose ap is forced crash
  if (ap_ok) {
    ap_has_been_ok = TRUE;
  }

  if ((ap_has_been_ok) && (!ap_ok)) {
    commands[COMMAND_FORCECRASH] = 9600;
  }
#endif

  if (inter_mcu_received_ap) {
    inter_mcu_received_ap = FALSE;
    inter_mcu_event_task();
    command_roll_trim = ap_state->command_roll_trim;
    command_pitch_trim = ap_state->command_pitch_trim;
    command_yaw_trim = ap_state->command_yaw_trim;
#if OUTBACK_CHALLENGE_DANGEROUS_RULE_RC_LOST_NO_AP
    // LOST-RC: do NOT go to autonomous
    // auto = stay in auto
    // manual = stay in manual
#else
    if (ap_ok && fbw_mode == FBW_MODE_FAILSAFE) {
      fbw_mode = FBW_MODE_AUTO;
    }
#endif
    if (fbw_mode == FBW_MODE_AUTO) {
      SetCommands(ap_state->commands);
    }
#ifdef SetApOnlyCommands
    else
    {
      SetApOnlyCommands(ap_state->commands);
    }
#endif
    fbw_new_actuators = 1;

#ifdef SINGLE_MCU
    inter_mcu_fill_fbw_state();
#endif /**Else the buffer is filled even if the last receive was not correct */
  }

#if OUTBACK_CHALLENGE_VERY_DANGEROUS_RULE_AP_CAN_FORCE_FAILSAFE
#warning DANGER DANGER DANGER DANGER: Outback Challenge Rule FORCE-CRASH-RULE: DANGER DANGER: AP is now capable to FORCE your FBW in failsafe mode EVEN IF RC IS NOT LOST: Consider the consequences.
  // OUTBACK: JURY REQUEST FLIGHT TERMINATION
  int crash = 0;
  if (commands[COMMAND_FORCECRASH] >= 8000)
  {
    set_failsafe_mode();
    crash = 1;
  }

#endif
#ifdef ACTUATORS
  if (fbw_new_actuators > 0)
  {
    pprz_t trimmed_commands[COMMANDS_NB];
    int i;
    for(i = 0; i < COMMANDS_NB; i++) trimmed_commands[i] = commands[i];

    #ifdef COMMAND_ROLL
    trimmed_commands[COMMAND_ROLL] += ChopAbs(command_roll_trim, MAX_PPRZ/10);
    #endif
    #ifdef COMMAND_PITCH
    trimmed_commands[COMMAND_PITCH] += ChopAbs(command_pitch_trim, MAX_PPRZ/10);
    #endif
    #ifdef COMMAND_YAW
    trimmed_commands[COMMAND_YAW] += ChopAbs(command_yaw_trim, MAX_PPRZ);
    #endif

    SetActuatorsFromCommands(trimmed_commands, autopilot_mode);
    fbw_new_actuators = 0;
    #if OUTBACK_CHALLENGE_VERY_DANGEROUS_RULE_AP_CAN_FORCE_FAILSAFE
    if (crash == 1)
    {
      for (;;) {
#if FBW_DATALINK
        fbw_datalink_event();
#endif
      }
    }
    #endif

  }
#endif


#ifdef MCU_SPI_LINK
  if (link_mcu_received) {
    link_mcu_received = FALSE;
    inter_mcu_fill_fbw_state(); /** Prepares the next message for AP */
    link_mcu_restart(); /** Prepares the next SPI communication */
  }
#endif /* MCU_SPI_LINK */
#endif /* INTER_MCU */

#ifdef FBW_DATALINK
  fbw_datalink_event();
#endif
}


/************* PERIODIC ******************************************************/
void periodic_task_fbw( void ) {

#ifdef FBW_DATALINK
  fbw_datalink_periodic();
#endif

#ifdef RADIO_CONTROL
  radio_control_periodic_task();
  if (fbw_mode == FBW_MODE_MANUAL && radio_control.status == RC_REALLY_LOST) {
#if OUTBACK_CHALLENGE_DANGEROUS_RULE_RC_LOST_NO_AP
#warning WARNING DANGER: OUTBACK_CHALLENGE RULE RC_LOST_NO_AP defined. If you loose RC you will NOT go to automatically go to AUTO2 Anymore!!
    set_failsafe_mode();
    commands[COMMAND_FORCECRASH] = 9600;
#else
    fbw_mode = FBW_MODE_AUTO;
#endif
  }
#endif

#ifdef INTER_MCU
  inter_mcu_periodic_task();
  if (fbw_mode == FBW_MODE_AUTO && !ap_ok)
  {
    set_failsafe_mode();
  }
#endif

#ifdef MCU_UART_LINK
  inter_mcu_fill_fbw_state();
  link_mcu_periodic_task();
#endif

#if PERIODIC_TELEMETRY
  periodic_telemetry_send_Fbw();
#endif

}

void handle_periodic_tasks_fbw(void) {

  if (sys_time_check_and_ack_timer(fbw_periodic_tid))
    periodic_task_fbw();

#if !(DISABLE_ELECTRICAL)
  if (sys_time_check_and_ack_timer(electrical_tid))
    electrical_periodic();
#endif

}
