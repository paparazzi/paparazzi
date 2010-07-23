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
 * test baro using interrupts
 *
 */

#include BOARD_CONFIG

#include "init_hw.h"
#include "sys_time.h"

#include "downlink.h"

#include "lisa/lisa_baro.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_on_baro_diff(void);
static inline void main_on_baro_abs(void);


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
  baro_init();
}



static inline void main_periodic_task( void ) {
  
  RunOnceEvery(2, {baro_periodic();});
  LED_PERIODIC();
  RunOnceEvery(256, {DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);});
  RunOnceEvery(256, {DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, &i2c2.errc_ack_fail, &i2c2.errc_miss_start_stop,
					      &i2c2.errc_arb_lost, &i2c2.errc_over_under,
					      &i2c2.errc_pec_recep, &i2c2.errc_timeout_tlow,
					      &i2c2.errc_smbus_alert);})
    
}



static inline void main_event_task( void ) {
  BaroEvent(main_on_baro_abs, main_on_baro_diff);
}



static inline void main_on_baro_diff(void) {

}

static inline void main_on_baro_abs(void) {
  RunOnceEvery(5,{DOWNLINK_SEND_BOOZ_BARO2_RAW(DefaultChannel, &baro.abs_raw, &baro.diff_raw);});
}
