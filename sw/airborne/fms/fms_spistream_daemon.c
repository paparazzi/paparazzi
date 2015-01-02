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

// Socket stuff
#include <sys/types.h>
#include <sys/un.h>
#include <signal.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
// End socket stuff

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
#include "fms_spistream.h"

#define LOG_OUT stdout


static void parse_command_line(int argc, char **argv);
static void main_init(void);
static void main_exit(void);
static void main_periodic(int my_sig_num);
static void spistream_event(void);

static int open_stream(void);
static void close_stream(void);

static void on_timeout(int signum);
static void on_kill(int signum);
static void on_dead_pipe(int signum);

static void on_spistream_msg_received(uint8_t msg_id, uint8_t *data, uint16_t num_bytes);
static void on_spistream_msg_sent(uint8_t msg_id);

static void send_to_client(uint8_t *data, uint16_t num_bytes, uint8_t fifo_idx);

static uint8_t spistream_msg[123];

static int dfifo[4];
static int cfifo[4];
static char dfifo_files[4][40];
static char cfifo_files[4][40];



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

  main_exit();

  TRACE(TRACE_DEBUG, "%s", "leaving mainloop, goodbye!\n");

  return 0;
}

static void main_periodic(int my_sig_num)
{

  static int32_t every_100 = 1000;
  if (every_100-- == 0) {
    every_100 = 1000;
    spistream_send_msg(spistream_msg, 21, SPISTREAM_NO_WAIT);
  }

  spistream_event();
}

static void spistream_event()
{

  static struct AutopilotMessagePTStream msg_in;
  static struct AutopilotMessagePTStream msg_out;
  static uint8_t crc_valid;

  spistream_read_pkg(&msg_in);
  /*
    uint8_t cnt;
    static uint8_t pkg_size = sizeof(msg_in.pkg_data);
    if(msg_out.message_cnt != 0) {
      printf("Package out: Size: %3d MID: %3d PCNTD: %3d | ",
          pkg_size, msg_out.message_cnt, msg_out.package_cntd);
      for(cnt = 0; cnt < pkg_size; cnt++) {
        printf("%3d ", msg_out.pkg_data[cnt]);
      }
      printf("\n");
    }
  */
  spi_link_send(&msg_out, sizeof(struct AutopilotMessageCRCFrame), &msg_in, &crc_valid);
  /*
    if(msg_in.message_cnt != 0) {
      printf("PKG in (spi trx: %d): Size: %3d MID: %3d PCNTD: %3d | ", SPISTREAM_PACKAGE_SIZE,
             pkg_size, msg_in.message_cnt, msg_in.package_cntd);
      for(cnt = 0; cnt < pkg_size; cnt++) {
        printf("%02X ", msg_in.pkg_data[cnt]);
      }
      printf("\n");
    }
  */
  spistream_write_pkg(&msg_out);
}

static void on_spistream_msg_received(uint8_t msg_id,
                                      uint8_t *data,
                                      uint16_t num_bytes)
{
  uint8_t uart;
  uint8_t buf[SPISTREAM_MAX_MESSAGE_LENGTH + 3];

  print_message("<< Daemon", msg_id, data, num_bytes);

  uart = data[0];
  // Check for valid uart ID
  if (uart >= 0 && uart <= 3) {
    if (msg_id > 0) {
      buf[0] = (uint8_t)(num_bytes & 0x00ff);
      buf[1] = (uint8_t)((num_bytes << 8) & 0x00ff);
      buf[2] = msg_id;
      if (num_bytes > SPISTREAM_MAX_MESSAGE_LENGTH) {
        fprintf(LOG_OUT, "Warning: Message has length %d, but limit "
                "is %d - truncating message\n",
                num_bytes, SPISTREAM_MAX_MESSAGE_LENGTH);
        num_bytes = SPISTREAM_MAX_MESSAGE_LENGTH;
      }
      memcpy(buf + 3, data, num_bytes);
      send_to_client(buf, num_bytes + 3, uart);
    }
  }
}

static void send_to_client(uint8_t *data, uint16_t num_bytes, uint8_t fifo_idx)
{
  if (dfifo[fifo_idx] <= 0) {
    // No client connected to this fifo, yet
    dfifo[fifo_idx] = open(dfifo_files[fifo_idx], O_WRONLY | O_NONBLOCK);
    if (dfifo[fifo_idx] <= 0) {
      fprintf(LOG_OUT, "No client for data fifo %d (%s)\n",
              fifo_idx, dfifo_files[fifo_idx]);
      return;
    }
  } else {
    // Client connected to this fifo
    if (write(dfifo[fifo_idx], data, num_bytes) == -1) {
      fprintf(LOG_OUT, "Write error on data fifo %d\n", fifo_idx);
    }
  }
}

static void on_spistream_msg_sent(uint8_t msg_id)
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

  signal(SIGKILL, on_kill);
  signal(SIGINT,  on_kill);
  signal(SIGILL,  on_kill);
  signal(SIGHUP,  on_kill);
  signal(SIGQUIT, on_kill);
  signal(SIGTERM, on_kill);
  signal(SIGSEGV, on_kill);
  signal(SIGPIPE, on_dead_pipe);

  if (!open_stream()) {
    fprintf(LOG_OUT, "Could not open stream, sorry\n");
    exit(1);
  }

  TRACE(TRACE_DEBUG, "%s", "Initialization completed\n");
}

static void main_exit(void)
{
  fprintf(LOG_OUT, "Closing socket\n");
  close_stream();
}

static void parse_command_line(int argc, char **argv)
{
  /*
    while ((ch = getopt(argc, argv, "d:")) != -1) {
      switch (ch) {
      case 'd':
        daemon_mode = 1;
        break;
      }
    }
  */
}

static int open_stream(void)
{
  uint8_t fifo_idx;
  int ret;

  strcpy(dfifo_files[0], "/tmp/spistream_d0.fifo"); // FIFOs for data
  strcpy(dfifo_files[1], "/tmp/spistream_d1.fifo"); // (STM -> daemon -> client)
  strcpy(dfifo_files[2], "/tmp/spistream_d2.fifo");
  strcpy(dfifo_files[3], "/tmp/spistream_d3.fifo");
  strcpy(cfifo_files[0], "/tmp/spistream_c0.fifo"); // FIFOs for commands
  strcpy(cfifo_files[1], "/tmp/spistream_c1.fifo"); // (client -> daemon -> STM)
  strcpy(cfifo_files[2], "/tmp/spistream_c2.fifo");
  strcpy(cfifo_files[3], "/tmp/spistream_c3.fifo");

  for (fifo_idx = 0; fifo_idx < 4; fifo_idx++) {
    fprintf(LOG_OUT, "Creating data stream %s ...", dfifo_files[fifo_idx]);
    if ((ret = mkfifo(dfifo_files[fifo_idx], 0777)) < 0) {
      fprintf(LOG_OUT, " failed\n");
      fprintf(LOG_OUT, "Could not create data fifo %d: %s\n",
              fifo_idx, dfifo_files[fifo_idx]);
      close_stream();
      return 0;
    } else {
      fprintf(LOG_OUT, " ok\n");
      dfifo[fifo_idx] = open(dfifo_files[fifo_idx], O_WRONLY | O_NONBLOCK);
    }
  }

  for (fifo_idx = 0; fifo_idx < 4; fifo_idx++) {
    fprintf(LOG_OUT, "Creating command stream %s ... ", cfifo_files[fifo_idx]);
    if ((ret = mkfifo(cfifo_files[fifo_idx], 0777)) < 0) {
      fprintf(LOG_OUT, " failed\n");
      fprintf(LOG_OUT, "Could not create command fifo %d: %s\n",
              fifo_idx, cfifo_files[fifo_idx]);
      close_stream();
      return 0;
    } else {
      fprintf(LOG_OUT, " ok\n");
      cfifo[fifo_idx] = open(cfifo_files[fifo_idx], O_RDONLY | O_NONBLOCK);
    }
  }
  return 1;
}

static void close_stream(void)
{
  uint8_t fifo_idx;
  fprintf(LOG_OUT, "Closing streams\n");
  for (fifo_idx = 0; fifo_idx < 4; fifo_idx++) {
    if (dfifo[fifo_idx] >= 0) {
      close(dfifo[fifo_idx]);
    }
    unlink(dfifo_files[fifo_idx]);
    if (cfifo[fifo_idx] >= 0) {
      close(cfifo[fifo_idx]);
    }
    unlink(cfifo_files[fifo_idx]);
  }
}

static void on_timeout(int signum)
{
  fprintf(LOG_OUT, "Timeout, stopping spistream daemon\n");
  exit(6);
}

static void on_kill(int signum)
{
  fprintf(LOG_OUT, "Exiting, got signal %d\n", signum);
  main_exit();
  exit(1);
}

static void on_dead_pipe(int signum)
{
  uint8_t fifo_idx;
  fprintf(LOG_OUT, "Got SIGPIPE (signal %d)\n", signum);
  // *Pop* goes the pipe. Looks like our client got AWOL.
  // Let's be nice and invalidate the file descriptors:
  for (fifo_idx = 0; fifo_idx < 4; fifo_idx++) {
    close(dfifo[fifo_idx]);
    dfifo[fifo_idx] = -1;
  }
}

