/*
 * Paparazzi $Id$
 *
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

/** \file main_fbw.c
 *  \brief FBW ( FlyByWire ) process
 *
 *   This process is responsible for decoding radio control, generating actuators
 * signals either from the radio control or from the commands provided by the
 * AP (autopilot) process. It also performs a telemetry task and a low level monitoring
 * ( for parameters like the supply )
 */

#include "generated/airframe.h"

#include "firmwares/fixedwing/main_fbw.h"
#include "mcu.h"
#include "sys_time.h"
#include "commands.h"
#include "firmwares/fixedwing/actuators.h"
#include "subsystems/electrical.h"
#include "subsystems/radio_control.h"
#include "firmwares/fixedwing/autopilot.h"
#include "fbw_downlink.h"
#include "paparazzi.h"

#ifdef MCU_SPI_LINK
#include "link_mcu.h"
#endif

#ifdef MILLIAMP_PER_PERCENT
#error "deprecated MILLIAMP_PER_PERCENT --> Please use MILLIAMP_AT_FULL_THROTTLE"
#endif


uint8_t fbw_mode;

#include "inter_mcu.h"


volatile uint8_t fbw_new_actuators = 0;


/********** INIT *************************************************************/
void init_fbw( void ) {

  mcu_init();
  sys_time_init();
  electrical_init();

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
  link_mcu_restart();
#endif

  fbw_mode = FBW_MODE_FAILSAFE;

#ifndef SINGLE_MCU
  mcu_int_enable();
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


/********** EVENT ************************************************************/

void event_task_fbw( void) {
#ifdef RADIO_CONTROL
  RadioControlEvent(handle_rc_frame);
#endif


#ifdef INTER_MCU
#ifdef MCU_SPI_LINK
    link_mcu_event_task();
#endif /* MCU_SPI_LINK */


  if (inter_mcu_received_ap) {
    inter_mcu_received_ap = FALSE;
    inter_mcu_event_task();
    if (ap_ok && fbw_mode == FBW_MODE_FAILSAFE) {
      fbw_mode = FBW_MODE_AUTO;
    }
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

#ifdef ACTUATORS
  if (fbw_new_actuators > 0)
  {
    SetActuatorsFromCommands(commands);
    fbw_new_actuators = 0;
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

}


/************* PERIODIC ******************************************************/
void periodic_task_fbw( void ) {
  static uint8_t _10Hz; /* FIXME : sys_time should provide it */
  _10Hz++;
  if (_10Hz >= 6) _10Hz = 0;

#ifdef RADIO_CONTROL
  radio_control_periodic_task();
  if (fbw_mode == FBW_MODE_MANUAL && radio_control.status == RC_REALLY_LOST) {
    fbw_mode = FBW_MODE_AUTO;
  }
#endif

#ifdef INTER_MCU
  inter_mcu_periodic_task();
  if (fbw_mode == FBW_MODE_AUTO && !ap_ok)
  {
    set_failsafe_mode();
  }
#endif

#ifdef DOWNLINK
  fbw_downlink_periodic_task();
#endif

  if (!_10Hz) {
    electrical_periodic();
  }

}
