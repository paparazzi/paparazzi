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

#include <event.h>

#include "std.h"
#include "fms_debug.h"
#include "fms_periodic.h"

/* stuff for io processor link */
#include "fms_spi_link.h"
#include "fms_autopilot_msg.h"

/* stuff for telemetry/datalink */
#include "fms_gs_com.h"

/* let's store those data somewhere */
#include "booz/booz_imu.h"

struct BoozImuFloat imu;

static void main_periodic(int my_sig_num);





static void send_message(void);
static void print_up_msg(struct AutopilotMessageCRCFrame * msg);
static void print_down_msg(struct AutopilotMessageCRCFrame * msg);

int main(int argc, char *argv[]) {

  /* Initalize our SPI link to IO processor */
  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return -1;
  }
  
  /* Initalize the event library */
  event_init();
  
  /* Initalize our ô so accurate periodic timer */
  if (fms_periodic_init(main_periodic)) {
    TRACE(TRACE_ERROR, "%s", "failed to start periodic generator\n");
    return -1; 
  }
  
  /* Initialize our communications with ground segment */
  fms_gs_com_init("10.31.4.7", 4242, 4243, FALSE);

  /* Enter our mainloop */
  event_dispatch();
  
  printf("leaving... goodbye!\n");

  return 0;

}

static void main_periodic(int my_sig_num) {

  send_message();
  
  fms_gs_com_periodic();

}



static void send_message() {
  static uint32_t foo = 0;

  struct AutopilotMessageCRCFrame msg_in;
  struct AutopilotMessageCRCFrame msg_out;
  uint8_t crc_valid; 

  uint16_t val = 1500 + 500*sin(foo*0.001);
  msg_out.payload.msg_down.pwm_outputs_usecs[0] = val;
  msg_out.payload.msg_down.pwm_outputs_usecs[1] = val;
  msg_out.payload.msg_down.pwm_outputs_usecs[2] = val;

  spi_link_send(&msg_out, sizeof(struct AutopilotMessageCRCFrame), &msg_in, &crc_valid);
  

  struct AutopilotMessagePTUp *in = &msg_in.payload.msg_up; 
  RATES_FLOAT_OF_BFP(imu.gyro, in->gyro);
  ACCELS_FLOAT_OF_BFP(imu.accel, in->accel); 

  if (!(foo % 200)) {
    //    printf("msg %d, CRC errors: %d\n", spi_link.msg_cnt, spi_link.crc_err_cnt);
    //    print_up_msg(&msg_in);
    //    print_down_msg(&msg_out);
    printf("%08d -> gx%+02.1f gy%+02.1f gz%+02.1f ax%+02.1f ay%+02.1f az%+02.1f rs%02x stm_msg %08d stm_err %d | CRC errors: %d\n",
	   foo,
	   DegOfRad(RATE_FLOAT_OF_BFP(in->gyro.p)), 
	   DegOfRad(RATE_FLOAT_OF_BFP(in->gyro.q)), 
	   DegOfRad(RATE_FLOAT_OF_BFP(in->gyro.r)),
	   ACCEL_FLOAT_OF_BFP(in->accel.x), 
	   ACCEL_FLOAT_OF_BFP(in->accel.y), 
	   ACCEL_FLOAT_OF_BFP(in->accel.z),
	   in->rc_status, 
	   in->stm_msg_cnt, 
	   in->stm_crc_err_cnt, 
	   spi_link.crc_err_cnt);
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


