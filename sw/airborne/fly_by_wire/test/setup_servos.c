/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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
#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer.h"
#include "servo.h"
#include "uart.h"

#define MSG_START '\0'
#define MSG_END '\n'

#define UNINIT        0
#define GOT_START     1
#define GOT_CHANNEL   2
#define GOT_LOW       3
#define GOT_HI        4


static uint8_t msg_status;
static uint8_t servo;
static uint16_t value; /* microseconds */
static volatile bool_t msg_valid;

static inline void parse_msg(uint8_t c) {
  switch (msg_status) {
  case UNINIT:
    if (c==MSG_START)
      msg_status++;
    else
      goto restart;
    break;
  case GOT_START:
    servo=c;
    msg_status++;
    break;
  case GOT_CHANNEL:
    value=c << 8;
    msg_status++;
    break;
  case GOT_LOW:
    value |= c;
    msg_status++;
    break;
  case GOT_HI:
    if (c == MSG_END)
      msg_valid = TRUE;
    goto restart;
  }
  return;
 restart:
  msg_status = UNINIT;
}

/* RxUartCb(parse_msg) */

SIGNAL( SIG_UART_RECV ) {
  uint8_t c = inp( UDR );
  parse_msg(c);
}


int main( void ) {
  uart_init_tx();
  uart_init_rx();
  timer_init();
  servo_init();
  sei();

  uart_print_string("$Id$\n");

  while (1) {
    if (msg_valid) {
      msg_valid = FALSE;
      servo_set_one(servo, value);
    }


/*     if (timer_periodic()) { */
/*       servo_set_one(SERVO, value); */
/*       value += 10; */
/*       if (value > 2000) value = 1000; */
/*     } */

/*     if (timer_periodic()) { */
/*       static uint8_t foo; */
/*       if (!foo) { */
/* 	uart_transmit('A'); */
/* 	uart_transmit('\n'); */
/*       } */
/*       foo++; */
/*     } */
  }

  return 0;
}
