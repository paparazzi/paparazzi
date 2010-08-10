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

#include "crc.h"
#include "fms_debug.h"
#include "fms_spi_link.h"
#include "fms_autopilot_msg.h"

static struct AutopilotMessageCRCFrame msg_in;
static struct AutopilotMessageCRCFrame msg_out;
static void send_message(void);
static void print_up_msg(struct AutopilotMessageCRCFrame * msg);
static void print_down_msg(struct AutopilotMessageCRCFrame * msg);

int main(int argc, char *argv[]) {

  uint32_t us_delay; 

  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return -1;
  }
  
  if(argc > 1) { 
    us_delay = atoi(argv[1]);
  }
  else { 
    us_delay = 1953; 
  }

  printf("Delay: %dus\n", us_delay); 

  crc__init(0x31); 
  printf("CRC initialized\n");

  printf("AutopilotMessage size: %d\n",          sizeof(union AutopilotMessage));
  printf("AutopilotMessageCRCFrame size: %d\n",  sizeof(struct AutopilotMessageCRCFrame));
  
  while (1) {
    send_message();
    usleep(us_delay);
  }

  return 0;
}



static void send_message() {
	static uint32_t foo = 0;
  static uint32_t crc_errors = 0; 
  crc_t crc_in; 

	spi_link_send(&msg_out, sizeof(struct AutopilotMessageCRCFrame), &msg_in);
  crc_in      = crc__calc_block_crc8(&msg_in.payload.msg_up, sizeof(union AutopilotMessage));
  
  if(msg_in.crc != crc_in) {
    crc_errors++;
  }
  
	if (!(foo % 100)) {
    printf("CRC errors: %d \n", crc_errors);
//    print_up_msg(&msg_in);
//    print_down_msg(&msg_out);
		printf("0x%08x -> gx%+02f gy%+02f gz%+02f ax%+02f ay%+02f az%+02f rs%02x | CRC errors: %d \n",
			foo,
			DegOfRad(RATE_FLOAT_OF_BFP(msg_in.payload.msg_up.gyro.p)), 
      DegOfRad(RATE_FLOAT_OF_BFP(msg_in.payload.msg_up.gyro.q)), 
      DegOfRad(RATE_FLOAT_OF_BFP(msg_in.payload.msg_up.gyro.r)),
			ACCEL_FLOAT_OF_BFP(msg_in.payload.msg_up.accel.x), 
      ACCEL_FLOAT_OF_BFP(msg_in.payload.msg_up.accel.y), 
      ACCEL_FLOAT_OF_BFP(msg_in.payload.msg_up.accel.z),
			msg_in.payload.msg_up.rc_status, 
      crc_errors);
	}
	foo++;
}


static void print_up_msg(struct AutopilotMessageCRCFrame * msg) { 
  printf("UP: %04X %04X %04X %04X %04x %04X %04X %04X %04X \n", 
          msg->payload.msg_up.gyro.p, 
          msg->payload.msg_up.gyro.q, 
          msg->payload.msg_up.gyro.r, 
          msg->payload.msg_up.accel.x, 
          msg->payload.msg_up.accel.y, 
          msg->payload.msg_up.accel.z, 
          msg->payload.msg_up.mag.x, 
          msg->payload.msg_up.mag.y, 
          msg->payload.msg_up.mag.z);
  printf("    %04X %04X %04X %04X %04X %04X %04X %04X %04X %02X [%d %d %d %d] CRC: %d\n", 
          msg->payload.msg_up.rc_pitch, 
          msg->payload.msg_up.rc_roll, 
          msg->payload.msg_up.rc_yaw, 
          msg->payload.msg_up.rc_thrust, 
          msg->payload.msg_up.rc_mode, 
          msg->payload.msg_up.rc_kill, 
          msg->payload.msg_up.rc_gear, 
          msg->payload.msg_up.rc_aux3, 
          msg->payload.msg_up.rc_aux4, 
          msg->payload.msg_up.rc_status, 
          msg->payload.msg_up.valid.rc, 
          msg->payload.msg_up.valid.pressure, 
          msg->payload.msg_up.valid.vane, 
          msg->payload.msg_up.valid.imu, 
          msg->crc);
}
static void print_down_msg(struct AutopilotMessageCRCFrame * msg) { 
  printf("%04X %04X %04X %04X %04X %04X CRC: %d\n", msg->payload.msg_down.pwm_outputs_usecs[0], 
                                                    msg->payload.msg_down.pwm_outputs_usecs[1], 
                                                    msg->payload.msg_down.pwm_outputs_usecs[2], 
                                                    msg->payload.msg_down.pwm_outputs_usecs[3], 
                                                    msg->payload.msg_down.pwm_outputs_usecs[4], 
                                                    msg->payload.msg_down.pwm_outputs_usecs[5], 
                                                    msg->crc);
}


