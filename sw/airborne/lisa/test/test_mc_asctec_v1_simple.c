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
 * That would be a quick asctech protocol test
 *
 */


#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "led.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static uint8_t i2c_done;

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
}



static inline void main_periodic_task( void ) {


  RunOnceEvery(256, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);});
  RunOnceEvery(256,
    {
      uint16_t i2c1_queue_full_cnt        = i2c1.errors->queue_full_cnt;
      uint16_t i2c1_ack_fail_cnt          = i2c1.errors->ack_fail_cnt;
      uint16_t i2c1_miss_start_stop_cnt   = i2c1.errors->miss_start_stop_cnt;
      uint16_t i2c1_arb_lost_cnt          = i2c1.errors->arb_lost_cnt;
      uint16_t i2c1_over_under_cnt        = i2c1.errors->over_under_cnt;
      uint16_t i2c1_pec_recep_cnt         = i2c1.errors->pec_recep_cnt;
      uint16_t i2c1_timeout_tlow_cnt      = i2c1.errors->timeout_tlow_cnt;
      uint16_t i2c1_smbus_alert_cnt       = i2c1.errors->smbus_alert_cnt;
      uint16_t i2c1_unexpected_event_cnt  = i2c1.errors->unexpected_event_cnt;
      uint32_t i2c1_last_unexpected_event = i2c1.errors->last_unexpected_event;
      const uint8_t _bus1 = 1;
      DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, DefaultDevice,
                               &i2c1_queue_full_cnt,
                               &i2c1_ack_fail_cnt,
                               &i2c1_miss_start_stop_cnt,
                               &i2c1_arb_lost_cnt,
                               &i2c1_over_under_cnt,
                               &i2c1_pec_recep_cnt,
                               &i2c1_timeout_tlow_cnt,
                               &i2c1_smbus_alert_cnt,
                               &i2c1_unexpected_event_cnt,
                               &i2c1_last_unexpected_event,
                               &_bus1);
    });


  i2c1_buf[0] = 100 + 0;
  i2c1_buf[1] = 100 + 0;
  i2c1_buf[2] = 100 + 0;
  i2c1_buf[3] = 6;
  i2c1_transmit(0x02, 4, &i2c_done);

  LED_PERIODIC();

}



static inline void main_event_task( void ) {

}
