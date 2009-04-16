/*
 * $Id: booz2_main.c 3049 2009-02-24 16:51:25Z poine $
 *  
 * Copyright (C) 2008  Antoine Drouin
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

#include <inttypes.h>

#include "csc_main.h"

#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart.h"
#include "downlink.h"

#include "csc_servos.h"
#include "uart_throttle.h"

#include "csc_can.h"
#include "csc_ap_link.h"
static inline void on_servo_cmd(void);
static inline void on_motor_cmd(void);
static inline void on_can_msg(void);


int main( void ) {
  csc_main_init();
  while(1) {
    if (sys_time_periodic())
      csc_main_periodic();
    csc_main_event();
  }
  return 0;
}


STATIC_INLINE void csc_main_init( void ) {

  hw_init();
  sys_time_init();
  led_init();

  Uart0Init();
  //  Uart1Init();

  csc_can1_init();
  csc_can2_init();

  csc_servos_init();
  //  uart_throttle_init();
  int_enable();

}


STATIC_INLINE void csc_main_periodic( void ) {

  LED_TOGGLE(2);
  static uint32_t cnt = 0;
  RunOnceEvery(10, {
      cnt++;
      struct CscCanMsg out_msg;
      out_msg.frame = (8<<16);
      out_msg.id = 42;
      out_msg.dat_a = cnt;
      out_msg.dat_b = cnt;
      csc_can2_send(&out_msg);
    });

  const float omega = 6.14 * 0.25;
  float now = GET_CUR_TIME_FLOAT();
  float srv_us = 1500. + 500. * cosf(omega*now);
  DOWNLINK_SEND_ATTITUDE(&srv_us, &srv_us, &srv_us);
  // FIXME : why *2 ?
  uint32_t srv_tic = 2 * SYS_TICS_OF_USEC(rint(srv_us));
  //uint32_t srv_tic = SYS_TICS_OF_USEC(1500);
  
  //  int32_t foobar[] = { 0, 0, 0, 0};
  int32_t foobar[] = { srv_tic, srv_tic, srv_tic, srv_tic};
  csc_servos_set(foobar);


  //  RunOnceEvery(100, {DOWNLINK_SEND_BOOT(&cpu_time_sec);});
      
}

STATIC_INLINE void csc_main_event( void ) {

  Can1Event(on_can_msg);
  Can2Event(on_can_msg);

}



static inline void on_can_msg(void) {

  //  DOWNLINK_SEND_CSC_CAN_MSG(&can1_rx_msg.frame, &can1_rx_msg.id, 
  //			    &can1_rx_msg.dat_a, &can1_rx_msg.dat_b);

  // FIXME : why *2 ?
  int32_t foo = SYS_TICS_OF_USEC(2 * 1500.);
  int32_t foobar[] = { foo, foo, foo, foo};
  csc_servos_set(foobar);
}






static inline void on_servo_cmd(void) {
  //  LED_TOGGLE(1);
}

static inline void on_motor_cmd(void) {
  //  LED_TOGGLE(1);
}


