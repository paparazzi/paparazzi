/*
 * $Id:$
 *
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
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


#include "init_hw.h"
#include "sys_time.h"
#include "csc_msg_def.h"
#include "csc_protocol.h"
#include "stm32/can.h"
#include "downlink.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

uint16_t servos[4];

FlagStatus can_error_warning = RESET;
FlagStatus can_error_passive = RESET;
FlagStatus can_bus_off = RESET;

struct CscVaneMsg csc_vane_msg;

void main_on_vane_msg(void *data);

int main(void) {
  main_init();

  servos[0] = 1;
  servos[1] = 2;
  servos[2] = 3;
  servos[3] = 4;

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
	cscp_init();
	cscp_register_callback(CSC_VANE_MSG_ID, main_on_vane_msg, (void *)&csc_vane_msg);
}

static inline void main_periodic_task( void ) {
	servos[0]+=10;
	servos[1]+=10;
	servos[2]+=10;
	servos[3]+=10;

	if((can_error_warning = CAN_GetFlagStatus(CAN1, CAN_FLAG_EWG)) == SET){
		LED_ON(2);
	}else{
		LED_OFF(2);
	}
	if((can_error_passive = CAN_GetFlagStatus(CAN1, CAN_FLAG_EPV)) == SET){
		LED_ON(3);
	}else{
		LED_OFF(3);
	}
	if((can_bus_off = CAN_GetFlagStatus(CAN1, CAN_FLAG_BOF)) == SET){
		LED_ON(0);
	}else{
		LED_OFF(0);
	}

	cscp_transmit(0, 0, (uint8_t *)servos, 8);

	LED_PERIODIC();
}



static inline void main_event_task( void ) {
	cscp_event();
}

void main_on_vane_msg(void *data)
{
	int zero = 0;
	RunOnceEvery(10, {DOWNLINK_SEND_VANE_SENSOR(DefaultChannel, &(csc_vane_msg.vane_angle1), &zero, &csc_vane_msg.vane_angle2, &zero, &zero, &zero, &zero);});
}
