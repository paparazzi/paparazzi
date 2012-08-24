/*
 * $Id$
 *
 * Copyright (C) 2011 Gautier Hattenberger
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

#define ABI_C

#include "mcu.h"
#include "sys_time.h"

struct abi_boo {
  int a;
  int b;
};

#include "subsystems/abi.h"

// ABI event and callback for TEST_ABI message
abi_event ev;

static void test_cb (const int8_t value, const struct abi_boo boo, const struct Int32Vect3 * vect) {
  switch (value) {
    case 0 :
      LED_TOGGLE(1); break;
    case 1 :
      LED_TOGGLE(2); break;
    case 2 :
      LED_TOGGLE(3); break;
  };
}


static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );


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
  mcu_init();
  sys_time_init();

  mcu_int_enable();

  // Bind to ABI message
  AbiBindMsgTEST_ABI(&ev, test_cb);
}



static inline void main_periodic_task( void ) {
  static int8_t val = 0;
  struct abi_boo b = { 1, 2 };
  struct Int32Vect3 v = { 3, 4, 5 };
  RunOnceEvery(60,{ AbiSendMsgTEST_ABI(val,b,&v); val=(val+1)%3; });
}



static inline void main_event_task( void ) {

}
