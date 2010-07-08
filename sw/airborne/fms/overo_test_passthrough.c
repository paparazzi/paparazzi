/*
 * $Id$
 *
 * Copyright (C) 2008-2010 Antoine Drouin <poinix@gmail.com>
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

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fms_debug.h"
#include "fms_spi_link.h"
#include "fms_autopilot_msg.h"


static union AutopilotMessagePT my_buffers[2];
static struct AutopilotMessagePTUp*   msg_in  = &my_buffers[0].up;
static struct AutopilotMessagePTDown* msg_out = &my_buffers[1].down;
static void send_message(void);

int main(int argc, char *argv[]) {

  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return -1;
  }
  while (1) {
    send_message();
    usleep(1953);
    //usleep(50000);
  }

  return 0;
}



static void send_message() {
	static uint32_t foo = 0;

	spi_link_send(msg_out, sizeof(union AutopilotMessagePT), msg_in);
	if (!(foo % 100)) {
		printf("0x%08x -> gx%+06d gy%+06d gz%+06d ax%+06d ay%+06d az%+06d rs%02x\n",
			foo,
			msg_in->gyro.x, msg_in->gyro.y, msg_in->gyro.z,
			msg_in->accel.x, msg_in->accel.y, msg_in->accel.z,
			msg_in->rc_status);
	}
	foo++;
}


