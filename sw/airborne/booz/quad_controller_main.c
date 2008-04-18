/*
 * $Id$
 *  
 * Copyright (C) 2008 Antoine Drouin
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

#include "booz_controller_main.h"

#include "std.h"
#include "init_hw.h"
#include "interrupt_hw.h"
#include "sys_time.h"
#include "led.h"

#include "booz_energy.h"

#include "commands.h"
#include "i2c.h"
#include "actuators.h"
#include "radio_control.h"

#include "adc.h"
#include "quad_ins.h"

#include "booz_estimator.h"
#include "booz_control.h"
#include "booz_nav.h"
#include "booz_autopilot.h"

#include "uart.h"
#include "messages.h"
#include "downlink.h"
#include "quad_controller_telemetry.h"
#include "datalink.h"



int16_t trim_p = 0;
int16_t trim_q = 0;
int16_t trim_r = 0;
uint8_t vbat = 0;

#ifndef SITL
int main( void ) {
  booz_controller_main_init();
  while(1) {
    if (sys_time_periodic())
      booz_controller_main_periodic_task();
    booz_controller_main_event_task();
  }
  return 0;
}
#endif

STATIC_INLINE void booz_controller_main_init( void ) {

  hw_init();
  led_init();
  sys_time_init();

  adc_init();
  booz_energy_init();

  i2c_init();
  actuators_init();
  SetCommands(commands_failsafe);

  ppm_init();
  radio_control_init();

  quad_ins_init();
  
  booz_estimator_init();
  booz_control_init();
  booz_nav_init();
  booz_autopilot_init();

  //FIXME
#ifndef SITL
  uart1_init_tx();
#endif

  int_enable();

  DOWNLINK_SEND_BOOT(&cpu_time_sec);
}


#define PeriodicPrescaleBy5( _code_0, _code_1, _code_2, _code_3, _code_4) { \
    static uint8_t _50hz = 0;						\
    _50hz++;								\
    if (_50hz >= 5) _50hz = 0;						\
    switch (_50hz) {							\
    case 0:								\
      _code_0;								\
      break;								\
    case 1:								\
      _code_1;								\
      break;								\
    case 2:								\
      _code_2;								\
      break;								\
    case 3:								\
      _code_3;								\
      break;								\
    case 4:								\
      _code_4;								\
      break;								\
    }									\
  }

STATIC_INLINE void booz_controller_main_periodic_task( void ) {
  
  quad_ins_periodic_task();
  /* run control loops */
  booz_autopilot_periodic_task();

  //  commands[COMMAND_P] = 0;
  //  commands[COMMAND_Q] = 0;
  //  commands[COMMAND_R] = 0;
  //  commands[COMMAND_THROTTLE] = MAX_PPRZ/3;

  SetActuatorsFromCommands(commands);

  PeriodicPrescaleBy5(							\
      {							                \
      radio_control_periodic_task();					\
      if (rc_status != RC_OK)						\
      booz_autopilot_mode = BOOZ_AP_MODE_FAILSAFE;			\
      },									\
      {									\
      booz_controller_telemetry_periodic_task();			\
      },									\
      {									\
      booz_energy_periodic();					\
      },									\
      {},									\
      {}									\
      );									\
}

STATIC_INLINE void booz_controller_main_event_task( void ) {
  
  // FIXME
#ifndef SITL
  DlEventCheckAndHandle();
#endif

  QuadInsEventCheckAndHandle(booz_estimator_read_inter_mcu_state);

  RadioControlEventCheckAndHandle(booz_autopilot_on_rc_event);
 
}
