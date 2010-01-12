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

#include <stm32/rcc.h>
#include <stm32/gpio.h>

#include <stm32/flash.h>
#include <stm32/misc.h>

#define DATALINK_C

#include CONFIG
#include "init_hw.h"
#include "sys_time.h"
#include "downlink.h"

#include "datalink.h"

static inline void main_init( void );
static inline void main_periodic( void );
static inline void main_event( void );

int main(void) {

  main_init();

  while (1) {
    if (sys_time_periodic())
      main_periodic();
    main_event();
  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
}

static inline void main_periodic( void ) {
  RunOnceEvery(50, {DOWNLINK_SEND_BOOT(&cpu_time_sec);});
}

static inline void main_event( void ) {
  DatalinkEvent();
}

void dl_parse_msg(void) {
  // FIXME : when i remove the datalink=0 line it stops working !!!!
  datalink_time = 0;
  uint8_t msg_id = dl_buffer[1];
  switch (msg_id) {
  
  case  DL_PING:
    {
      DOWNLINK_SEND_PONG();
    }
    break;
  }
}
