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

#include "fms/overo_blmc_calibrate.h"

#include "std.h"
#include "fms_debug.h"

/* stuff for io processor link */
#include "fms_spi_link.h"
#include "fms_autopilot_msg.h"

struct OveroBLMCCalibrate blmc_calibrate;

static void main_init(void);
static void dialog_with_io_proc(void);

int main(int argc, char *argv[])
{

  main_init();

  return 0;

}

static void dialog_with_io_proc()
{

  struct AutopilotMessageCRCFrame msg_in;
  struct AutopilotMessageCRCFrame msg_out;
  uint8_t crc_valid;

  for (uint8_t i = 0; i < LISA_PWM_OUTPUT_NB; i++) { msg_out.payload.msg_down.pwm_outputs_usecs[i] = blmc_calibrate.servos_outputs_usecs[i]; }

  spi_link_send(&msg_out, sizeof(struct AutopilotMessageCRCFrame), &msg_in, &crc_valid);

}

static void main_init(void)
{

  /* Initalize our SPI link to IO processor */
  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return;
  }

  printf("Starting at 2000us\n");
  /* Initialize blaaa */
  for (uint8_t i = 0; i < LISA_PWM_OUTPUT_NB; i++) { blmc_calibrate.servos_outputs_usecs[i] = 2000; }
  dialog_with_io_proc();
  getchar();
  printf("At 1000us\n");
  for (uint8_t i = 0; i < LISA_PWM_OUTPUT_NB; i++) { blmc_calibrate.servos_outputs_usecs[i] = 1000; }
  dialog_with_io_proc();
  getchar();
  printf("At 1500us\n");
  for (uint8_t i = 0; i < LISA_PWM_OUTPUT_NB; i++) { blmc_calibrate.servos_outputs_usecs[i] = 1500; }
  dialog_with_io_proc();

}
