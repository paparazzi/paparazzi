/*
 * $Id$
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

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "fms_debug.h"
#include "fms_spi_link.h"
#include "fms_autopilot_msg.h"

static void fill_msg(struct AutopilotMessageFoo* msg);

static uint32_t passed_cnt = 0;

int main(int argc, char *argv[]) {
  
  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return -1;
  }
  while (1) {
    struct AutopilotMessageFoo msg_out_prev;
    struct AutopilotMessageFoo msg_out;
    memcpy(&msg_out_prev, &msg_out, sizeof(msg_out));
    fill_msg(&msg_out);
    struct AutopilotMessageFoo msg_in;
    spi_link_send(&msg_out, sizeof(struct AutopilotMessageFoo), &msg_in);
    if (memcmp(&msg_in, &msg_out_prev, sizeof(msg_in))) {
      printf("compare failed\n");
      printf("expected %d %d %d %d\n", msg_out_prev.foo, msg_out_prev.bar, msg_out_prev.bla, msg_out_prev.ble);
      printf("got      %d %d %d %d\n\n", msg_in.foo, msg_in.bar, msg_in.bla, msg_in.ble);
    }
    else {
      passed_cnt++;
      if (!(passed_cnt%1000)) {
	printf("passed %d\n", passed_cnt );
      }
    }
    usleep(1953);
    //    usleep(50000);
  }

  return 0;
}



static void fill_msg(struct AutopilotMessageFoo* msg) {
  static uint32_t foo = 0;
  msg->foo  = foo;
  msg->bar  = foo+2;
  msg->bla  = foo+4;
  msg->ble  = foo+8;
  msg->bli  = foo+4;
  msg->blo  = foo+4;
  msg->blu  = foo+4;
  msg->bly  = foo+4;
 
  foo++;
}
