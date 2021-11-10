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
#include "firmwares/fixedwing/main_fbw.h"

#include "generated/airframe.h"

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "modules/core/commands.h"
#include "subsystems/actuators.h"
#include "modules/energy/electrical.h"
#include "subsystems/radio_control.h"
#include "autopilot.h"
#include "paparazzi.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/uart.h"

#ifdef USE_NPS
#include "nps_autopilot.h"
#endif

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

#ifdef FBW_DATALINK
#include "firmwares/fixedwing/fbw_datalink.h"
#endif

#include "inter_mcu.h"
#include "link_mcu.h"

uint8_t fbw_mode;

/** Trim commands for roll, pitch and yaw.
 * These are updated from the trim commands in ap_state via inter_mcu
 */
pprz_t command_roll_trim;
pprz_t command_pitch_trim;
pprz_t command_yaw_trim;

volatile uint8_t fbw_new_actuators = 0;

uint8_t ap_has_been_ok = false;

tid_t fbw_periodic_tid;  ///< id for periodic_task_fbw() timer
tid_t electrical_tid;   ///< id for electrical_periodic() timer

/********** RADIO CONTROL DEFINES ************************************************/
#ifdef RADIO_CONTROL
/**
 * Defines behavior when the RC is lost, default goes to AUTO
 */
void radio_lost_mode(void);

#if !OUTBACK_CHALLENGE_DANGEROUS_RULE_RC_LOST_NO_AP && !OUTBACK_CHALLENGE_DANGEROUS_RULE_RC_LOST_NO_AP_IRREVERSIBLE
// default case
void radio_lost_mode(void)
{
  fbw_mode = FBW_MODE_AUTO;
}
#endif /* default */

#if OUTBACK_CHALLENGE_DANGEROUS_RULE_RC_LOST_NO_AP
#warning WARNING DANGER: OUTBACK_CHALLENGE RULE RC_LOST_NO_AP defined. If you loose RC you will NOT go to automatically go to AUTO2 Anymore!!
static inline void set_failsafe_mode(void);
void radio_lost_mode(void)
{
  set_failsafe_mode();
}
#endif /* OUTBACK_CHALLENGE_DANGEROUS_RULE_RC_LOST_NO_AP */

#if OUTBACK_CHALLENGE_DANGEROUS_RULE_RC_LOST_NO_AP_IRREVERSIBLE
#warning WARNING DANGER: OUTBACK_CHALLENGE_DANGEROUS_RULE_RC_LOST_NO_AP_IRREVERSIBLE defined. If you ever temporarly lost RC while in manual, you will failsafe forever even if RC is restored
void radio_lost_mode(void)
{
  commands[COMMAND_FORCECRASH] = 9600;
}
#endif /* OUTBACK_CHALLENGE_DANGEROUS_RULE_RC_LOST_NO_AP_IRREVERSIBLE */


static inline void handle_rc_frame(void)
{
  fbw_mode = FBW_MODE_OF_PPRZ(radio_control.values[RADIO_MODE]);
  if (fbw_mode == FBW_MODE_MANUAL) {
    SetCommandsFromRC(commands, radio_control.values);
    fbw_new_actuators = 1;
  }
}

void radio_control_event(void)
{
  RadioControlEvent(handle_rc_frame);
}

void radio_control_periodic_handle(void)
{
  radio_control_periodic_task();
  if (fbw_mode == FBW_MODE_MANUAL && radio_control.status == RC_REALLY_LOST) {
    radio_lost_mode();
  }
}
#else /* no RADIO_CONTROL */
void radio_control_event(void) {}
void radio_control_periodic_handle(void) {}
#endif /* RADIO_CONTROL */

/********** FBW_DATALINK defines ************************************************/
#ifdef FBW_DATALINK
void fbw_datalink_periodic_handle(void)
{
  fbw_datalink_periodic();
}
void fbw_datalink_event_handle(void)
{
  fbw_datalink_event();
}
#else /* no FBW_DATALINK */
void fbw_datalink_periodic_handle(void) {}
void fbw_datalink_event_handle(void) {}
#endif /* FBW_DATALINK */

/********** ACTUATORS defines ************************************************/
void update_actuators(void);
#if defined ACTUATORS && defined INTER_MCU
void update_actuators(void)
{
  if (fbw_new_actuators > 0) {
    pprz_t trimmed_commands[COMMANDS_NB];
    int i;
    for (i = 0; i < COMMANDS_NB; i++) {trimmed_commands[i] = commands[i];}

#ifdef COMMAND_ROLL
    trimmed_commands[COMMAND_ROLL] += ClipAbs(command_roll_trim, MAX_PPRZ / 10);
#endif /* COMMAND_ROLL */

#ifdef COMMAND_PITCH
    trimmed_commands[COMMAND_PITCH] += ClipAbs(command_pitch_trim, MAX_PPRZ / 10);
#endif /* COMMAND_PITCH */

#ifdef COMMAND_YAW
    trimmed_commands[COMMAND_YAW] += ClipAbs(command_yaw_trim, MAX_PPRZ);
#endif /* COMMAND_YAW */

    SetActuatorsFromCommands(trimmed_commands, autopilot_get_mode());
    fbw_new_actuators = 0;
  }
}
#else
void update_actuators(void) {};
#endif /* ACTUATORS && INTER_MCU */


/********** INTER_MCU defines ************************************************/
#ifdef INTER_MCU
// pre-and post functions
void pre_inter_mcu_received_ap(void);
void post_inter_mcu_received_ap(void);

/**
 * FBW_MODE_INTER_MCU_FAILSAFE defines what happens when inter MCU fails
 * Defaults goes to AUTO
 */
#define FBW_MODE_INTER_MCU_FAILSAFE FBW_MODE_AUTO

#if !OUTBACK_CHALLENGE_VERY_DANGEROUS_RULE_NO_AP_MUST_FAILSAFE && !OUTBACK_CHALLENGE_VERY_DANGEROUS_RULE_AP_CAN_FORCE_FAILSAFE
void pre_inter_mcu_received_ap(void) {};
void post_inter_mcu_received_ap(void) {};
#endif /* DEFAULT */


#if OUTBACK_CHALLENGE_DANGEROUS_RULE_RC_LOST_NO_AP
#undef   FBW_MODE_INTER_MCU_FAILSAFE
#define FBW_MODE_INTER_MCU_FAILSAFE fbw_mode
#endif /* OUTBACK_CHALLENGE_DANGEROUS_RULE_RC_LOST_NO_AP */


#if OUTBACK_CHALLENGE_VERY_DANGEROUS_RULE_NO_AP_MUST_FAILSAFE
#warning OUTBACK_CHALLENGE_VERY_DANGEROUS_RULE_NO_AP_MUST_FAILSAFE loose ap is forced crash
void pre_inter_mcu_received_ap(void)
{
  if (ap_ok) {
    ap_has_been_ok = true;
  }
  if ((ap_has_been_ok) && (!ap_ok)) {
    commands[COMMAND_FORCECRASH] = 9600;
  }
}
void post_inter_mcu_received_ap(void) {};
#endif /* OUTBACK_CHALLENGE_VERY_DANGEROUS_RULE_NO_AP_MUST_FAILSAFE */


#if OUTBACK_CHALLENGE_VERY_DANGEROUS_RULE_AP_CAN_FORCE_FAILSAFE
#warning DANGER DANGER DANGER DANGER: Outback Challenge Rule FORCE-CRASH-RULE: DANGER DANGER:
#warning AP is now capable to FORCE your FBW in failsafe mode EVEN IF RC IS NOT LOST: Consider the consequences.
// OUTBACK: JURY REQUEST FLIGHT TERMINATION
void pre_inter_mcu_received_ap(void) {};
void post_inter_mcu_received_ap(void)
{
  if (commands[COMMAND_FORCECRASH] >= 8000) {
    set_failsafe_mode();
  }
}
#endif /* OUTBACK_CHALLENGE_VERY_DANGEROUS_RULE_AP_CAN_FORCE_FAILSAFE */


/**
 * Failsafe function
 */
static inline void set_failsafe_mode(void)
{
  fbw_mode = FBW_MODE_FAILSAFE;
  SetCommands(commands_failsafe);
  fbw_new_actuators = 1;
}

/**
 * Handles inter_mcu periodic checks
 */
void inter_mcu_periodic_handle(void)
{
  inter_mcu_periodic_task();
  if (fbw_mode == FBW_MODE_AUTO && !ap_ok) {
    set_failsafe_mode();
  }

#if defined MCU_UART_LINK || defined MCU_CAN_LINK
  inter_mcu_fill_fbw_state();
  link_mcu_periodic_task();
#endif /* defined MCU_UART_LINK || defined MCU_CAN_LINK */
}

/**
 * Handles inter_mcu event
 */
void inter_mcu_event_handle(void)
{
#if defined MCU_SPI_LINK | defined MCU_UART_LINK
  link_mcu_event_task();
#endif /* MCU_SPI_LINK */

  pre_inter_mcu_received_ap();

  if (inter_mcu_received_ap) {
    inter_mcu_received_ap = false;
    inter_mcu_event_task();

    PPRZ_MUTEX_LOCK(ap_state_mtx);
    command_roll_trim = ap_state->command_roll_trim;
    command_pitch_trim = ap_state->command_pitch_trim;
    command_yaw_trim = ap_state->command_yaw_trim;
    if (ap_ok && fbw_mode == FBW_MODE_FAILSAFE) {
      fbw_mode = FBW_MODE_INTER_MCU_FAILSAFE;
    }
    if (fbw_mode == FBW_MODE_AUTO) {
      SetCommands(ap_state->commands);
    } else {
#if SET_AP_ONLY_COMMANDS
      SetApOnlyCommands(ap_state->commands);
#endif /* SET_AP_ONLY_COMMANDS */
    }
    fbw_new_actuators = 1;
    PPRZ_MUTEX_UNLOCK(ap_state_mtx);

#ifdef SINGLE_MCU
    inter_mcu_fill_fbw_state();
#endif /* SINGLE_MCU - The buffer is filled even if the last receive was not correct */
  }

  post_inter_mcu_received_ap();

  update_actuators();

#ifdef MCU_SPI_LINK
  if (link_mcu_received) {
    link_mcu_received = false;
    inter_mcu_fill_fbw_state(); /** Prepares the next message for AP */
    link_mcu_restart(); /** Prepares the next SPI communication */
  }
#endif /* MCU_SPI_LINK */
}
#else /* no INTER_MCU */
void inter_mcu_periodic_handle(void) {}
void inter_mcu_event_handle(void) {}
#endif /* INTER_MCU */

/********** PERIODIC MESSAGES ************************************************/
#if PERIODIC_TELEMETRY
static void send_commands(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_COMMANDS(trans, dev, AC_ID, COMMANDS_NB, commands);
}

#ifdef RADIO_CONTROL
static void send_fbw_status(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_FBW_STATUS(trans, dev, AC_ID,
                           &(radio_control.status), &(radio_control.frame_rate), &fbw_mode, &electrical.vsupply, &electrical.current);
}

static void send_rc(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_RC(trans, dev, AC_ID, RADIO_CONTROL_NB_CHANNEL, radio_control.values);
}

#else /* no RADIO_CONTROL */
static void send_fbw_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t dummy = 0;
  pprz_msg_send_FBW_STATUS(trans, dev, AC_ID,
                           &dummy, &dummy, &fbw_mode, &electrical.vsupply, &electrical.current);
}
#endif /* RADIO_CONTROL */

#ifdef ACTUATORS
static void send_actuators(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ACTUATORS(trans, dev, AC_ID , ACTUATORS_NB, actuators);
}
#endif /* ACTUATORS */

void periodic_telemetry_handle(void)
{
  periodic_telemetry_send_Fbw(DefaultPeriodic, &(DefaultChannel).trans_tx, &(DefaultDevice).device);
}

#else
void periodic_telemetry_handle(void) {}
#endif /* PERIODIC_TELEMETRY */

/********** INIT *************************************************************/
void init_fbw(void)
{
  mcu_init();

#if !(DISABLE_ELECTRICAL)
  electrical_init();
#endif

#ifdef ACTUATORS
  actuators_init();
  /* Load the failsafe defaults */
  SetCommands(commands_failsafe);
  fbw_new_actuators = 1;
#endif /* ACTUATORS */

#ifdef RADIO_CONTROL
  radio_control_init();
#endif /* RADIO_CONTROL */

#ifdef INTER_MCU
  inter_mcu_init();
#endif /* INTER_MCU */

#if defined MCU_SPI_LINK || defined MCU_CAN_LINK
  link_mcu_init();
#endif /* MCU_SPI_LINK || MCU_CAN_LINK */

#ifdef MCU_SPI_LINK
  link_mcu_restart();
#endif /* MCU_SPI_LINK */

  fbw_mode = FBW_MODE_FAILSAFE;

  /**** start timers for periodic functions *****/
  fbw_periodic_tid = sys_time_register_timer((1. / 60.), NULL);
  electrical_tid = sys_time_register_timer(0.1, NULL);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FBW_STATUS, send_fbw_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_COMMANDS, send_commands);

#ifdef ACTUATORS
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACTUATORS, send_actuators);
#endif /* ACTUATORS */

#ifdef RADIO_CONTROL
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RC, send_rc);
#endif /* RADIO_CONTROL */

#endif /* PERIODIC_TELEMETRY */

}

/********** EVENT ************************************************************/

void event_task_fbw(void)
{
  radio_control_event();

  /* event functions for mcu peripherals: i2c, usb_serial.. */
  mcu_event();

  inter_mcu_event_handle();

  fbw_datalink_event_handle();
}

/************* PERIODIC ******************************************************/
void periodic_task_fbw(void)
{
  fbw_datalink_periodic_handle();

  radio_control_periodic_handle();

  inter_mcu_periodic_handle();

  periodic_telemetry_handle();
}

void handle_periodic_tasks_fbw(void)
{
  if (sys_time_check_and_ack_timer(fbw_periodic_tid)) {
    periodic_task_fbw();
  }

#if !(DISABLE_ELECTRICAL)
  if (sys_time_check_and_ack_timer(electrical_tid)) {
    electrical_periodic();
  }
#endif
}
