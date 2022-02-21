/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/**
 * @file firmwares/fixedwing/autopilot_generated.c
 *
 * Generated autopilot implementation.
 *
 */
#define AUTOPILOT_CORE_AP_C

#include "firmwares/fixedwing/autopilot_generated.h"
#include "autopilot.h"

#include "modules/radio_control/radio_control.h"
#include "modules/core/commands.h"
#include "modules/actuators/actuators.h"
#include "modules/core/settings.h"
#include "modules/datalink/telemetry.h"

#include "generated/settings.h"


void autopilot_generated_init(void)
{
  // call generated init
  autopilot_core_ap_init();
  // copy generated mode to public mode (set at startup)
  autopilot.mode = autopilot_mode_ap;
}


void autopilot_generated_periodic(void)
{

  autopilot_core_ap_periodic_task();

  // copy generated mode to public mode (may be changed on internal exceptions)
  autopilot.mode = autopilot_mode_ap;

#if defined MCU_SPI_LINK || defined MCU_UART_LINK || defined MCU_CAN_LINK
  link_mcu_send();
#elif defined INTER_MCU && defined SINGLE_MCU
  /**Directly set the flag indicating to FBW that shared buffer is available*/
  inter_mcu_received_ap = true;
#endif

}

/** AP mode setting handler
 *
 * FIXME generated something else for this?
 */
void autopilot_generated_SetModeHandler(float mode)
{
  autopilot_generated_set_mode(mode);
}


void autopilot_generated_set_mode(uint8_t new_autopilot_mode)
{
  autopilot_core_ap_set_mode(new_autopilot_mode);
  // copy generated mode to public mode
  autopilot.mode = autopilot_mode_ap;
}


void autopilot_generated_set_motors_on(bool motors_on)
{
  // only needed for consistency with other firmwares
  autopilot.motors_on = motors_on;
}


void autopilot_generated_on_rc_frame(void)
{
#ifdef SetAutoCommandsFromRC
  SetAutoCommandsFromRC(commands, radio_control.values);
#elif defined RADIO_YAW && defined COMMAND_YAW
  command_set(COMMAND_YAW, radio_control_get(RADIO_YAW));
#endif
}

