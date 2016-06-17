/*
 * Copyright (C) 2014 Felix Ruess <felix.ruess@gmail.com>
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
 * @file test_manual.c
 *
 * Test the manual control via RC including command_laws.
 */


#define DATALINK_C

#include "generated/airframe.h"
#include "generated/settings.h"

#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#if USE_MOTOR_MIXING
#include "subsystems/actuators/motor_mixing.h"
#endif

#include "subsystems/radio_control.h"

static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);

static void on_rc_frame(void);

tid_t main_periodic_tid; ///< id for main_periodic() timer
tid_t radio_control_tid; ///< id for radio_control_periodic_task() timer

bool autopilot_motors_on;

int main(void)
{

  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(main_periodic_tid)) {
      main_periodic();
    }
    if (sys_time_check_and_ack_timer(radio_control_tid)) {
      radio_control_periodic_task();
    }
    main_event();
  };
  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  downlink_init();

  actuators_init();
#if USE_MOTOR_MIXING
  motor_mixing_init();
#endif

  radio_control_init();

  mcu_int_enable();
  main_periodic_tid = sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  radio_control_tid = sys_time_register_timer((1. / 60.), NULL);

  // just to make it usable in a standard rotorcraft airframe file
  // with <call fun="motor_mixing_run(autopilot_motors_on,FALSE,values)"/>
  // in the command_laws section
  autopilot_motors_on = true;
}

static inline void main_periodic(void)
{

  SetActuatorsFromCommands(commands, 0);

  LED_PERIODIC();
  RunOnceEvery(512, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice,  16, MD5SUM);});
  RunOnceEvery(100, {DOWNLINK_SEND_RC(DefaultChannel, DefaultDevice, RADIO_CONTROL_NB_CHANNEL, radio_control.values);});
  RunOnceEvery(101, {DOWNLINK_SEND_COMMANDS(DefaultChannel, DefaultDevice, COMMANDS_NB, commands);});
  RunOnceEvery(102, {DOWNLINK_SEND_ACTUATORS(DefaultChannel, DefaultDevice, ACTUATORS_NB, actuators);});
}

static inline void main_event(void)
{
  mcu_event();
  DatalinkEvent();
  RadioControlEvent(on_rc_frame);
}

#define IdOfMsg(x) (x[1])

void dl_parse_msg(void)
{
  uint8_t msg_id = IdOfMsg(dl_buffer);
  switch (msg_id) {
    case  DL_PING: {
      DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
    }
    break;

    case DL_SETTING: {
      if (DL_SETTING_ac_id(dl_buffer) != AC_ID) { break; }
      uint8_t i = DL_SETTING_index(dl_buffer);
      float var = DL_SETTING_value(dl_buffer);
      DlSetting(i, var);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &var);
    }
    break;

    case DL_GET_SETTING : {
      if (DL_GET_SETTING_ac_id(dl_buffer) != AC_ID) { break; }
      uint8_t i = DL_GET_SETTING_index(dl_buffer);
      float val = settings_get_value(i);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
    }
    break;
#ifdef RADIO_CONTROL_TYPE_DATALINK
    case DL_RC_3CH :
#ifdef RADIO_CONTROL_DATALINK_LED
      LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
#endif
      parse_rc_3ch_datalink(
        DL_RC_3CH_throttle_mode(dl_buffer),
        DL_RC_3CH_roll(dl_buffer),
        DL_RC_3CH_pitch(dl_buffer));
      break;
    case DL_RC_4CH :
#ifdef RADIO_CONTROL_DATALINK_LED
      LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
#endif
      parse_rc_4ch_datalink(
        DL_RC_4CH_mode(dl_buffer),
        DL_RC_4CH_throttle(dl_buffer),
        DL_RC_4CH_roll(dl_buffer),
        DL_RC_4CH_pitch(dl_buffer),
        DL_RC_4CH_yaw(dl_buffer));
      break;
#endif // RADIO_CONTROL_TYPE_DATALINK

    default:
      break;
  }
}


static void on_rc_frame(void)
{

  /* if there are some commands that should always be set from RC, do it */
#ifdef SetAutoCommandsFromRC
  SetAutoCommandsFromRC(commands, radio_control.values);
#endif

#ifdef SetCommandsFromRC
  SetCommandsFromRC(commands, radio_control.values);
#else
  // no rc_commands, assume we have rotorcraft with ROLL, PITCH, YAW, THRUST
  commands[COMMAND_ROLL] = radio_control.values[RADIO_ROLL];
  commands[COMMAND_PITCH] = radio_control.values[RADIO_PITCH];
  commands[COMMAND_YAW] = radio_control.values[RADIO_YAW];
  commands[COMMAND_THRUST] = radio_control.values[RADIO_THROTTLE];
#endif

}
