/*
 * $Id: booz2_test_mc.c 3701 2009-07-13 09:57:55Z poine $
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include <inttypes.h>

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "interrupt_hw.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/datalink.h"

#include "booz2_test_buss_bldc_hexa.h"

#define NB_MOTORS 6
static const uint8_t motor_addr[] = {0x52, 0x54, 0x56, 0x58, 0x5A, 0x5C};
uint8_t motor = 0;
uint8_t thrust = 10;
static bool_t  i2c_done;

//static uint8_t addr = 0x52; /* 1 : back right   */
//static uint8_t addr = 0x54; /* 2 : back left   bad balanced */
//static uint8_t addr = 0x56; /* 3 : center right  not so well balanced */
//static uint8_t addr = 0x58; /* 4 : center left  */
//static uint8_t addr = 0x5A; /* 5 : front right  */
//static uint8_t addr = 0x5C; /* 6 : front left   */

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

int main( void ) {
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
  mcu_int_enable();
}

static inline void main_periodic_task( void ) {
  i2c0_buf[0] = thrust;
  i2c0_transmit(motor_addr[motor], 1, &i2c_done);

  RunOnceEvery(128, { DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);});

}

static inline void main_event_task( void ) {
  DatalinkEvent();
}

