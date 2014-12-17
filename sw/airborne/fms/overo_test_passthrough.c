/*
 * Copyright (C) 2010 The Paparazzi Team
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

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <event.h>

#include "fms/overo_test_passthrough.h"

#include "std.h"
#include "fms_debug.h"
#include "fms_periodic.h"

/* stuff for io processor link */
#include "fms_spi_link.h"
#include "fms_autopilot_msg.h"

/* stuff for telemetry/datalink */
#include "fms_gs_com.h"

struct OveroTestPassthrough otp;

static void parse_command_line(int argc, char **argv);
static void main_init(void);
static void main_periodic(int my_sig_num);
static void dialog_with_io_proc(void);


int main(int argc, char *argv[])
{

  parse_command_line(argc, argv);

  main_init();
  TRACE(TRACE_DEBUG, "%s", "Entering mainloop\n");

  /* Enter our mainloop */
  event_dispatch();

  TRACE(TRACE_DEBUG, "%s", "leaving mainloop... goodbye!\n");

  return 0;

}

static void main_periodic(int my_sig_num)
{

  dialog_with_io_proc();

  fms_gs_com_periodic();

}



static void dialog_with_io_proc()
{

  struct AutopilotMessageCRCFrame msg_in;
  struct AutopilotMessageCRCFrame msg_out;
  uint8_t crc_valid;

  for (uint8_t i = 0; i < 6; i++) { msg_out.payload.msg_down.pwm_outputs_usecs[i] = otp.servos_outputs_usecs[i]; }
//  for (uint8_t i=0; i<4; i++) msg_out.payload.msg_down.csc_servo_cmd[i] = otp.csc_servo_outputs[i];

  spi_link_send(&msg_out, sizeof(struct AutopilotMessageCRCFrame), &msg_in, &crc_valid);

  struct AutopilotMessagePTUp *in = &msg_in.payload.msg_up;
  RATES_FLOAT_OF_BFP(otp.imu.gyro, in->gyro);
  ACCELS_FLOAT_OF_BFP(otp.imu.accel, in->accel);
  MAGS_FLOAT_OF_BFP(otp.imu.mag, in->mag);

  otp.io_proc_msg_cnt = in->stm_msg_cnt;
  otp.io_proc_err_cnt = in->stm_crc_err_cnt;

  otp.rc_status = in->rc_status;

  otp.baro_abs  = in->pressure_absolute;
  otp.baro_diff = in->pressure_differential;

}


static void main_init(void)
{

  TRACE(TRACE_DEBUG, "%s", "Starting initialization\n");

  /* Initalize our SPI link to IO processor */
  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return;
  }

  /* Initalize the event library */
  event_init();

  /* Initalize our ô so accurate periodic timer */
  if (fms_periodic_init(main_periodic)) {
    TRACE(TRACE_ERROR, "%s", "failed to start periodic generator\n");
    return;
  }

  /* Initialize our communications with ground segment */
  fms_gs_com_init(otp.gs_gw, 4242, 4243, FALSE);

  /* Initialize blaaa */
  for (uint8_t i = 0; i < 6; i++) { otp.servos_outputs_usecs[i] = 1500; }
  for (uint8_t i = 0; i < 4; i++) { otp.csc_servo_outputs[i] = 1500; }

  TRACE(TRACE_DEBUG, "%s", "Initialization completed\n");
}



static void parse_command_line(int argc, char **argv)
{

  if (argc > 1) {
    otp.gs_gw = strdup(argv[1]);
  } else {
    otp.gs_gw = strdup("10.31.4.7");
  }
  TRACE(TRACE_DEBUG, "%s", "Parsing command line:\n");
  TRACE(TRACE_DEBUG, " gw: %s\n", otp.gs_gw);

}
