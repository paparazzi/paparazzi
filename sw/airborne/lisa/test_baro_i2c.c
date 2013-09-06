/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

/*
 *
 * test baro using interrupts
 *
 */

#include BOARD_CONFIG

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/i2c.h"
#include "led.h"

#include "subsystems/datalink/downlink.h"

#include "subsystems/sensors/baro.h"
#include "subsystems/air_data.h"
//#include "my_debug_servo.h"

#define ABI_C
#include "subsystems/abi.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_on_baro_diff(void);
static inline void main_on_baro_abs(void);


int main(void) {
  main_init();

  while(1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic_task();
    main_event_task();
  }

  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  baro_init();

  //  DEBUG_SERVO1_INIT();
  //  DEBUG_SERVO2_INIT();


}



static inline void main_periodic_task( void ) {

  RunOnceEvery(2, {baro_periodic();});
  LED_PERIODIC();
  RunOnceEvery(256, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);});
  RunOnceEvery(256,
    {
      uint16_t i2c2_queue_full_cnt        = i2c2.errors->queue_full_cnt;
      uint16_t i2c2_ack_fail_cnt          = i2c2.errors->ack_fail_cnt;
      uint16_t i2c2_miss_start_stop_cnt   = i2c2.errors->miss_start_stop_cnt;
      uint16_t i2c2_arb_lost_cnt          = i2c2.errors->arb_lost_cnt;
      uint16_t i2c2_over_under_cnt        = i2c2.errors->over_under_cnt;
      uint16_t i2c2_pec_recep_cnt         = i2c2.errors->pec_recep_cnt;
      uint16_t i2c2_timeout_tlow_cnt      = i2c2.errors->timeout_tlow_cnt;
      uint16_t i2c2_smbus_alert_cnt       = i2c2.errors->smbus_alert_cnt;
      uint16_t i2c2_unexpected_event_cnt  = i2c2.errors->unexpected_event_cnt;
      uint32_t i2c2_last_unexpected_event = i2c2.errors->last_unexpected_event;
      const uint8_t _bus2 = 2;
      DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, DefaultDevice,
                               &i2c2_queue_full_cnt,
                               &i2c2_ack_fail_cnt,
                               &i2c2_miss_start_stop_cnt,
                               &i2c2_arb_lost_cnt,
                               &i2c2_over_under_cnt,
                               &i2c2_pec_recep_cnt,
                               &i2c2_timeout_tlow_cnt,
                               &i2c2_smbus_alert_cnt,
                               &i2c2_unexpected_event_cnt,
                               &i2c2_last_unexpected_event,
                               &_bus2);
    });
}



static inline void main_event_task( void ) {
  BaroEvent();
}



static inline void main_on_baro_diff(void) {

}

static inline void main_on_baro_abs(void) {
  RunOnceEvery(5,{DOWNLINK_SEND_BARO_RAW(DefaultChannel, DefaultDevice, &air_data.pressure, &air_data.differential);});
}
