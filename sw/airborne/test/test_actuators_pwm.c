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

/* PERIODIC_C_MAIN is defined before generated/periodic_telemetry.h
 * in order to implement telemetry_mode_Main_*
 */
#define PERIODIC_C_MAIN

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "generated/periodic_telemetry.h"
#pragma GCC diagnostic pop

#include "generated/airframe.h"
#include "generated/settings.h"

#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"
#include "modules/datalink/pprz_dl.h"

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "modules/actuators/actuators_pwm.h"


static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);

int main(void)
{

  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic();
    }
    main_event();
  };
  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  downlink_init();
  pprz_dl_init();
  ActuatorsPwmInit();
}

static inline void main_periodic(void)
{
  ActuatorsPwmCommit();

  LED_PERIODIC();
  RunOnceEvery(100, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice,  16, MD5SUM);});
}

static inline void main_event(void)
{
  pprz_dl_event();
}



#define IdOfMsg(x) (x[1])

void dl_parse_msg(struct link_device *dev __attribute__((unused)), struct transport_tx *trans __attribute__((unused)), uint8_t *buf)
{
  uint8_t msg_id = IdOfMsg(buf);
  switch (msg_id) {
    case  DL_PING: {
      DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
    }
    break;

    case DL_SET_ACTUATOR: {
      uint8_t servo_no = DL_SET_ACTUATOR_no(buf);
      uint16_t servo_value = DL_SET_ACTUATOR_value(buf);
#ifdef LED_2
      LED_TOGGLE(2);
#endif
      if (servo_no < ACTUATORS_PWM_NB) {
        ActuatorPwmSet(servo_no, servo_value);
      }
    }
    break;

    case DL_SETTING: {
      if (DL_SETTING_ac_id(buf) != AC_ID) { break; }
      uint8_t i = DL_SETTING_index(buf);
      float var = DL_SETTING_value(buf);
      DlSetting(i, var);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &var);
    }
    break;

    case DL_GET_SETTING : {
      if (DL_GET_SETTING_ac_id(buf) != AC_ID) { break; }
      uint8_t i = DL_GET_SETTING_index(buf);
      float val = settings_get_value(i);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
    }
    break;

    default:
      break;
  }
}
