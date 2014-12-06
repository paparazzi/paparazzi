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
 * @file test_actuators_pwm.c
 *
 * Simple test prog for PWM actuators.
 * Directly control actuators_pwm_values via settings.
 */


#define DATALINK_C

#include "generated/airframe.h"
#include "generated/settings.h"

#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "subsystems/actuators/actuators_pwm.h"


static inline void main_init( void );
static inline void main_periodic( void );
static inline void main_event(void);

int main(void) {

  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic();
    main_event();
  };
  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  ActuatorsPwmInit();
}

static inline void main_periodic( void ) {
  ActuatorsPwmCommit();

  LED_PERIODIC();
  RunOnceEvery(100, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice,  16, MD5SUM);});
}

static inline void main_event(void) {
  DatalinkEvent();
}



#define IdOfMsg(x) (x[1])

void dl_parse_msg( void ) {
  uint8_t msg_id = IdOfMsg(dl_buffer);
  switch (msg_id) {
    case  DL_PING:
      {
        DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
      }
      break;

    case DL_SET_ACTUATOR:
      {
        uint8_t servo_no = DL_SET_ACTUATOR_no(dl_buffer);
        uint16_t servo_value = DL_SET_ACTUATOR_value(dl_buffer);
        LED_TOGGLE(2);
        if (servo_no < ACTUATORS_PWM_NB) {
          ActuatorPwmSet(servo_no, servo_value);
        }
      }
      break;

    case DL_SETTING:
      {
        if (DL_SETTING_ac_id(dl_buffer) != AC_ID) break;
        uint8_t i = DL_SETTING_index(dl_buffer);
        float var = DL_SETTING_value(dl_buffer);
        DlSetting(i, var);
        DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &var);
      }
      break;

    case DL_GET_SETTING :
      {
        if (DL_GET_SETTING_ac_id(dl_buffer) != AC_ID) break;
        uint8_t i = DL_GET_SETTING_index(dl_buffer);
        float val = settings_get_value(i);
        DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
      }
      break;

    default:
      break;
  }
}
