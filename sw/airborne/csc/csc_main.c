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
#include "csc_can.h"

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
  csc_can1_init();
  csc_can2_init();
  csc_servos_init();
  int_enable();

}


STATIC_INLINE void csc_main_periodic( void ) {
  static uint32_t cnt = 0;
  RunOnceEvery(10, {
      cnt++;
      //      LED_TOGGLE(2);
      struct CscCanMsg out_msg;
      out_msg.dat_a = cnt;
      out_msg.dat_b = cnt;
      csc_can1_send(&out_msg);
    });

  uint16_t foo = 42;
  RunOnceEvery(100, {DOWNLINK_SEND_BOOT(&foo);});
      
}

STATIC_INLINE void csc_main_event( void ) {
  Can2Event(on_can_msg);
}


static inline void on_can_msg(void) {
  uint8_t foo = 0;
  static int32_t bar;
  int32_t new_bar = can2_rx_msg.dat_a;

  if (new_bar - bar == 1)
    LED_TOGGLE(2);
  
  bar = new_bar;
  DOWNLINK_SEND_CHRONO(&foo, &bar);
  int32_t foobar[] = { 0, 0, 0, 0};
  csc_servos_set(foobar);
  
}

