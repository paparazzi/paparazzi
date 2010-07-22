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


//static union AutopilotMessageBeth my_buffers[2];
//static struct AutopilotMessageBethUp   msg_in;
//static struct AutopilotMessageBethDown msg_out;
static struct AutopilotMessageFoo   msg_in;
static struct AutopilotMessageFoo   msg_out;
static void send_message(void);

int main(int argc, char *argv[]) {
  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return -1;
  }
  while (1) {
    send_message();
    usleep(1953);
    //usleep(500000);
  }

  return 0;
}

uint16_t az,elev,tilt;

static uint32_t foo = 0;
static void send_message() {

  msg_out.foo = 0x0123;
  msg_out.bar = 0x4567;

  spi_link_send(&msg_out, sizeof(struct AutopilotMessageFoo) , &msg_in);
//  if (msg_in.bli == "0xdead") {
    az = msg_in.foo;
    elev = msg_in.bar;
    tilt = msg_in.blaa;
//  }
  if (!(foo%100)) { 
    printf("%d %d %d %x\r\n",az,elev,tilt,msg_in.bli);
    
  }
  foo++;
}


