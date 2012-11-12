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


#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "mcu_periph/i2c.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static struct i2c_transaction trans;

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

  trans.type = I2CTransTx;
  trans.slave_addr = 0x02;
  trans.len_w = 4;
  trans.buf[0] = 100;
  trans.buf[1] = 100;
  trans.buf[2] = 100;
  trans.buf[3] = 1;
  i2c_submit(&i2c1,&trans);

  LED_PERIODIC();

}



static inline void main_event_task( void ) {

}
