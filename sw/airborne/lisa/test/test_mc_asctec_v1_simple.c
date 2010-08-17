/*
 * $Id$
 *  
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


#include "init_hw.h"
#include "sys_time.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static uint8_t i2c_done;

int main(void) {
  main_init();

  while(1) {
    if (sys_time_periodic())
      main_periodic_task();
    main_event_task();
  }

  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
}



static inline void main_periodic_task( void ) {
  

  RunOnceEvery(256, {DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);});
  RunOnceEvery(256, 
    {
      DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, 
			       &i2c2_errors.ack_fail_cnt,
			       &i2c2_errors.miss_start_stop_cnt,
			       &i2c2_errors.arb_lost_cnt,
			       &i2c2_errors.over_under_cnt,
			       &i2c2_errors.pec_recep_cnt,
			       &i2c2_errors.timeout_tlow_cnt,
			       &i2c2_errors.smbus_alert_cnt,
			       &i2c2_errors.unexpected_event_cnt,
			       &i2c2_errors.last_unexpected_event);
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
