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
#include <time.h>

#include <event.h>

#include "std.h"
#include "fms_debug.h"
#include "fms_periodic.h"

/* stuff for io processor link */
#include "fms_spi_link.h"
#include "fms_autopilot_msg.h"

/* stuff for telemetry/datalink */

#define OVERO_ENV
#include "lisa/lisa_spistream.h"

static void parse_command_line(int argc, char **argv);
static void main_init(void);
static void main_periodic(int my_sig_num);
static void spistream_event(void);

static void on_spistream_msg_received(uint8_t msg_id, uint8_t *data, uint16_t num_bytes);
static void on_spistream_msg_sent(void);

static uint8_t spistream_msg[123];

int main(int argc, char *argv[])
{

  parse_command_line(argc, argv);

  main_init();
  TRACE(TRACE_DEBUG, "%s", "Entering mainloop\n");

  /* Enter our mainloop */
  event_dispatch();
  while (1) {
    sleep(100);
  }

  TRACE(TRACE_DEBUG, "%s", "leaving mainloop, goodbye!\n");

  return 0;
}

static void main_periodic(int my_sig_num)
{
#if 0
  static int32_t every_100 = 1000;
  if (every_100-- == 0) {
    every_100 = 1000;
    spistream_send_msg(spistream_msg, 21, SPISTREAM_NO_WAIT);
    /*
        spistream_send_msg(spistream_msg, 15);
        spistream_send_msg(spistream_msg, 25);
    */
  }
#endif
  spistream_event();
}

static void spistream_event()
{

  struct AutopilotMessagePTStream msg_in;
  struct AutopilotMessagePTStream msg_out;
  static uint8_t pkg_size = sizeof(msg_in.pkg_data);
  uint8_t crc_valid;
  uint8_t cnt;

  spistream_write_pkg(&msg_out);

  if (msg_out.message_cnt != 0) {
    printf("Package out: Size: %3d MID: %3d PCNTD: %3d | ",
           pkg_size, msg_out.message_cnt, msg_out.package_cntd);
    for (cnt = 0; cnt < pkg_size; cnt++) {
      printf("%3d ", msg_out.pkg_data[cnt]);
    }
    printf("\n");
  }

  spi_link_send(&msg_out, sizeof(struct AutopilotMessageCRCFrame), &msg_in, &crc_valid);
  /*
    if(msg_in.message_cnt != 0) {
      printf("Package in:  Size: %3d MID: %3d PCNTD: %3d | ",
          pkg_size, msg_in.message_cnt, msg_in.package_cntd);
      for(cnt = 0; cnt < pkg_size; cnt++) {
        printf("%3d ", msg_in.pkg_data[cnt]);
      }
      printf("\n");
    }
  */
  spistream_read_pkg(&msg_in);
}

static void on_spistream_msg_received(uint8_t msg_id,
                                      uint8_t *data,
                                      uint16_t num_bytes)
{
  static uint16_t plot_freq = 100;

  uint16_t log_bytes;
  uint8_t cnt;
  struct tm *timeinfo;
  time_t c_time;
  char time_str[30];

  plot_freq = 100;
  time(&c_time);

  timeinfo = localtime(&c_time);
  strftime(time_str, 30, " %X ", timeinfo);

  log_bytes = num_bytes;
  if (log_bytes > 48) { log_bytes = 48; }
  printf("SPI message received: ");
  printf("%s | Length: %3d | id: %3d | UART%d | ", time_str, num_bytes, msg_id, data[0]);
  for (cnt = 1; cnt < log_bytes; cnt++) {
    printf("%02X ", data[cnt]);
  }
  printf("\n");
}

static void on_spistream_msg_sent(void)
{
//  TRACE(TRACE_DEBUG, "%s", "SPI message sent \n");
}

static void main_init(void)
{
  uint8_t byte_idx;

  TRACE(TRACE_DEBUG, "%s", "Starting initialization\n");

  /* Initalize our SPI link to IO processor */
  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return;
  }

  spistream_init(&on_spistream_msg_received,
                 &on_spistream_msg_sent);
  /*
    spistream_msg[0] = 0;
    for(byte_idx=1; byte_idx < 123; byte_idx += 4) {
      spistream_msg[byte_idx]   = 0xDE;
      spistream_msg[byte_idx+1] = 0xAD;
      spistream_msg[byte_idx+2] = 0xBE;
      spistream_msg[byte_idx+3] = 0xEF;
    }
  */
  for (byte_idx = 1; byte_idx < 123; byte_idx++) {
    spistream_msg[byte_idx] = byte_idx;
  }
  /* Initalize the event library */
  event_init();

  /* Initalize our ô so accurate periodic timer */
  if (fms_periodic_init(main_periodic)) {
    TRACE(TRACE_ERROR, "%s", "failed to start periodic generator\n");
    return;
  }

  TRACE(TRACE_DEBUG, "%s", "Initialization completed\n");
}

static void parse_command_line(int argc, char **argv)
{
}
