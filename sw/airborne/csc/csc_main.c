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
#include "csc_throttle.h"

#include "csc_can.h"
#include "csc_ap_link.h"
static inline void on_servo_cmd(void);
static inline void on_motor_cmd(void);

#define SERVO_TIMEOUT (SYS_TICS_OF_SEC(0.1) / PERIODIC_TASK_PERIOD)
#define CSC_STATUS_TIMEOUT (SYS_TICS_OF_SEC(0.2) / PERIODIC_TASK_PERIOD)

static uint32_t servo_cmd_timeout = 0;
static uint32_t can_msg_count = 0;

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

#ifdef USE_UART0
  Uart0Init();
#endif
//  Uart1Init();

  csc_can_init();

  csc_servos_init();
  csc_throttle_init();
  int_enable();

}


STATIC_INLINE void csc_main_periodic( void ) {
  static uint32_t zeros[4] = {0, 0, 0, 0};
  static uint32_t csc_loops = 0;

  if (servo_cmd_timeout > SERVO_TIMEOUT) {
    LED_OFF(CAN_LED);
    csc_servos_set(zeros);
  } else {
    servo_cmd_timeout++;
  }
  
  if ((++csc_loops % CSC_STATUS_TIMEOUT) == 0) {
    csc_ap_link_send_status(csc_loops, can_msg_count);
  }

}

STATIC_INLINE void csc_main_event( void ) {

  CscApLinkEvent(on_servo_cmd, on_motor_cmd);
  csc_throttle_event_task();
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

  servo_cmd_timeout = 0;
  ++can_msg_count;

  //  DOWNLINK_SEND_CSC_CAN_MSG(&can1_rx_msg.frame, &can1_rx_msg.id, 
  //  			    &can1_rx_msg.dat_a, &can1_rx_msg.dat_b);
  //  DOWNLINK_SEND_ADC_GENERIC(&servos[0], &servos[1]);

}


static inline void on_motor_cmd(void)
{
  // always send to throttle_id zero, only one motorcontrol per csc board
  const static uint8_t throttle_id = 0;

  csc_throttle_send_msg(throttle_id, csc_motor_msg.cmd_id, csc_motor_msg.arg1, csc_motor_msg.arg2);
}
