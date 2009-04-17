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

#include "std.h"

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
  Uart1Init();

  csc_can2_init();

  csc_servos_init();
  uart_throttle_init();
  int_enable();

}


STATIC_INLINE void csc_main_periodic( void ) {


      
}

STATIC_INLINE void csc_main_event( void ) {

  CscApLinkEvent(on_servo_cmd, on_motor_cmd);

}


#define MIN_SERVO 2*SYS_TICS_OF_USEC(1000)
#define MAX_SERVO 2*SYS_TICS_OF_USEC(2000)

static inline void on_servo_cmd(void) {

  uint16_t* servos = (uint16_t*)(&csc_servo_cmd);
  uint32_t servos_checked[4];
  uint32_t i;
  for (i=0; i<4; i++)
    servos_checked[i] = Chop(servos[i],MIN_SERVO, MAX_SERVO);
  csc_servos_set(servos_checked);

  //  DOWNLINK_SEND_CSC_CAN_MSG(&can1_rx_msg.frame, &can1_rx_msg.id, 
  //  			    &can1_rx_msg.dat_a, &can1_rx_msg.dat_b);
  //  DOWNLINK_SEND_ADC_GENERIC(&servos[0], &servos[1]);

}


static inline void on_motor_cmd(void)
{
	// always send to throttle_id zero, only one motorcontrol per csc board
	const static uint8_t throttle_id = 0;

	throttle_send_command(throttle_id, csc_motor_cmd.cmd_id, csc_motor_cmd.arg1, csc_motor_cmd.arg2);
}
