/*
 * Copyright (C) 2006-2013 The Paparazzi Team
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
 * @file setup_actuators.c
 *
 */

#define DATALINK_C

#include "generated/airframe.h"
#include "generated/settings.h"

#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"


#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "subsystems/actuators.h"


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

  downlink_init();

  actuators_init();
  uint8_t i;
  for (i = 0; i < ACTUATORS_NB; i++) {
    //SetServo(i, 1500);
  }

  mcu_int_enable();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
}


static inline void main_periodic(void)
{

  // generated macro from airframe file
  AllActuatorsCommit();

  LED_PERIODIC();
  RunOnceEvery(100, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice,  16, MD5SUM);});
  RunOnceEvery(300, DOWNLINK_SEND_ACTUATORS(DefaultChannel, DefaultDevice, ACTUATORS_NB, actuators));
}

static inline void main_event(void)
{
  mcu_event();

  DatalinkEvent();
}


#define IdOfMsg(x) (x[1])

void dl_parse_msg(void)
{
  uint8_t msg_id = IdOfMsg(dl_buffer);
  if (msg_id == DL_SET_ACTUATOR) {
    uint8_t actuator_no = DL_SET_ACTUATOR_no(dl_buffer);
    uint16_t actuator_value = DL_SET_ACTUATOR_value(dl_buffer);
    LED_TOGGLE(2);

    /* bad hack:
     * first arg to ActuatorSet needs to be the servo _name_ as given in the airframe file
     * here we rely on the servos having number as names in the setup.xml airframe file
     */
    switch (actuator_no) {
#ifdef SERVO_0
      case 0: ActuatorSet(0, actuator_value); break;
#endif
#ifdef SERVO_1
      case 1: ActuatorSet(1, actuator_value); break;
#endif
#ifdef SERVO_2
      case 2: ActuatorSet(2, actuator_value); break;
#endif
#ifdef SERVO_3
      case 3: ActuatorSet(3, actuator_value); break;
#endif
#ifdef SERVO_4
      case 4: ActuatorSet(4, actuator_value); break;
#endif
#ifdef SERVO_5
      case 5: ActuatorSet(5, actuator_value); break;
#endif
#ifdef SERVO_6
      case 6: ActuatorSet(6, actuator_value); break;
#endif
#ifdef SERVO_7
      case 7: ActuatorSet(7, actuator_value); break;
#endif
#ifdef SERVO_8
      case 8: ActuatorSet(8, actuator_value); break;
#endif
      default: break;
    }

    //if (actuator_no < ACTUATORS_NB) {
    //  actuators[actuator_no] = actuator_value;
    //}
  }
#ifdef DlSetting
  else if (msg_id == DL_SETTING && DL_SETTING_ac_id(dl_buffer) == AC_ID) {
    uint8_t i = DL_SETTING_index(dl_buffer);
    float val = DL_SETTING_value(dl_buffer);
    DlSetting(i, val);
    LED_TOGGLE(2);

#ifdef SERVO_0
    ActuatorSet(0, actuators[SERVO_0_IDX]);
#endif
#ifdef SERVO_1
    ActuatorSet(1, actuators[SERVO_1_IDX]);
#endif
#ifdef SERVO_2
    ActuatorSet(2, actuators[SERVO_2_IDX]);
#endif
#ifdef SERVO_3
    ActuatorSet(3, actuators[SERVO_3_IDX]);
#endif
#ifdef SERVO_4
    ActuatorSet(4, actuators[SERVO_4_IDX]);
#endif
#ifdef SERVO_5
    ActuatorSet(5, actuators[SERVO_5_IDX]);
#endif
#ifdef SERVO_6
    ActuatorSet(6, actuators[SERVO_6_IDX]);
#endif
#ifdef SERVO_7
    ActuatorSet(7, actuators[SERVO_7_IDX]);
#endif
#ifdef SERVO_8
    ActuatorSet(8, actuators[SERVO_8_IDX]);
#endif

    DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
  } else if (msg_id == DL_GET_SETTING && DL_GET_SETTING_ac_id(dl_buffer) == AC_ID) {
    uint8_t i = DL_GET_SETTING_index(dl_buffer);
    float val = settings_get_value(i);
    DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
  }
#endif
}
