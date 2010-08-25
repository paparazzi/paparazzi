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

#include <stm32/flash.h>
#include <stm32/misc.h>

#include BOARD_CONFIG
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "adc.h"
#include "adc_hw.h"
#include "downlink.h"

int main_periodic(void);
static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static struct adc_buf adc0_buf; 
static struct adc_buf adc1_buf; 
static struct adc_buf adc2_buf; 
static struct adc_buf adc3_buf; 

extern uint8_t adc_new_data_trigger;

static inline void main_init( void ) {
	hw_init();
	sys_time_init();
	led_init(); 
	adc_init(); 
	
	adc_buf_channel(0, &adc0_buf, 3);
	adc_buf_channel(1, &adc1_buf, 3);
	adc_buf_channel(2, &adc2_buf, 3);
	adc_buf_channel(3, &adc3_buf, 3);
}

int main( void ) {
	main_init(); 

	while(1) {
	      if (sys_time_periodic()) { 
	      	main_periodic_task();
	      	DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM); 
	      }
	
	      main_event_task();
	}
	return 0;
}

static inline void main_periodic_task( void ) {
	LED_PERIODIC();
}

static inline void main_event_task( void ) {
	uint8_t down = 123;
	
	if (adc_new_data_trigger) { 
		DOWNLINK_SEND_PONG(DefaultChannel);
		adc_new_data_trigger = 0; 
		LED_TOGGLE(7);
//		down = (((adc0_buf.values[0]) >> 4) & 0x00ff);
		DOWNLINK_SEND_BOOZ_DEBUG_FOO(DefaultChannel, &down);
	}
}

