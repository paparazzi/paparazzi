/*
 * Copyright (C) 2011 The Paparazzi Team
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
 */
#define DATALINK_C

#include BOARD_CONFIG

#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/settings.h"

#include "mcu_periph/uart.h"
#include "messages.h"

#include "my_debug_servo.h"

static inline void main_init( void );
static inline void main_periodic( void );
static inline void main_event( void );


float setting_a;
float setting_b;
float setting_c;
float setting_d;

int main( void ) {
  main_init();
  while(1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic();
    main_event();
  }
  return 0;
}


static inline void main_init( void ) {

  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  settings_init();
  //  DEBUG_SERVO2_INIT();
  //  LED_ON(1);
  //  LED_ON(2);
  //  DEBUG_S4_ON();
  //  DEBUG_S5_ON();
  //  DEBUG_S6_ON();
  mcu_int_enable();

}

static inline void main_periodic( void ) {

  RunOnceEvery(100, {
      DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice,  16, MD5SUM);
      PeriodicSendDlValue(DefaultChannel);
    });

}

static inline void main_event( void ) {

  DatalinkEvent();

}

void dl_parse_msg(void) {
  datalink_time = 0;
  uint8_t msg_id = dl_buffer[1];
  switch (msg_id) {

  case  DL_PING: {
    DOWNLINK_SEND_PONG(DefaultChannel);
  }
    break;
  case DL_SETTING:
    if(DL_SETTING_ac_id(dl_buffer) == AC_ID) {
      uint8_t i = DL_SETTING_index(dl_buffer);
      float val = DL_SETTING_value(dl_buffer);
      DlSetting(i, val);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
    }
    break;
  default:
    break;
  }
}
