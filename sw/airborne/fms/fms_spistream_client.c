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

#define OVERO_ENV
#include "lisa/lisa_spistream.h"
#include "fms_spistream.h"


static void parse_command_line(int argc, char **argv);
static void main_init(void);
static void main_exit(void);
static void main_periodic(int my_sig_num);

static int open_stream(void);

static void on_kill(int signum);

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
  uint8_t fifo_idx;
  uint8_t msg_id;
  uint16_t num_bytes;
  int16_t ret;
  static uint8_t buf[SPISTREAM_MAX_MESSAGE_LENGTH * 10];

  for (fifo_idx = 0; fifo_idx < 4; fifo_idx++) {
    // The periodic is triggered before fifo
    // connections have been initialized, so
    // check for a valid fd first:
    if (dfifo[fifo_idx] > 0) {
      ret = read(dfifo[fifo_idx], (uint8_t *)(&num_bytes), 2);
      ret = read(dfifo[fifo_idx], (uint8_t *)(&msg_id), 1);

      memset(&buf, 0, SPISTREAM_MAX_MESSAGE_LENGTH);
      if (num_bytes > SPISTREAM_MAX_MESSAGE_LENGTH) {
        fprintf(stderr, "Warning: Message has length %d, but limit "
                "is %d\n",
                num_bytes, SPISTREAM_MAX_MESSAGE_LENGTH);
        num_bytes = SPISTREAM_MAX_MESSAGE_LENGTH;
      }
      ret = read(dfifo[fifo_idx], &buf, num_bytes);
      if (ret > 0 && ret == num_bytes) {
        // Message received
        print_message(">> Client", msg_id, buf, num_bytes);
      } else if (ret > 0 && ret < num_bytes) {
        fprintf(stderr, "Tried to read %d bytes, but only got %d\n",
                num_bytes, ret);
      }
    } else {
      // FIFO file descriptor is invalid,
      // retry to open it:
      dfifo[fifo_idx] = open(dfifo_files[fifo_idx], O_RDONLY | O_NONBLOCK);
    }
  }

}

static void main_init(void)
{

  TRACE(TRACE_DEBUG, "%s", "Starting initialization\n");

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

  if (!open_stream()) {
    fprintf(stderr, "Could not open stream, sorry\n");
    exit(1);
  }

  TRACE(TRACE_DEBUG, "%s", "Initialization completed\n");
}

/**
 * For every FIFO, a non-blocking connection try is called
 * via open(..., O_NONBLOCK).
 * This immediately returns a file descriptor or 0 if
 * the other end of the fifo is closed.
 * In the transmission, we check the FIFO file descriptors
 * and retry to open them, in case they are invalid.
 * You can also just open() them without O_NONBLOCK in
 * the client app, but the daemon should be running before
 * starting the client then, otherwise open() would block.
 *
 * When using this strategy, we get connection
 * recovery for free when either daemon or client die.
 *
 * After the connections are established, you can use them
 * for read() and write(), as well as register an event
 * trigger on them, like select() or libevent.
 *
 */
static int open_stream(void)
{
  uint8_t fifo_idx;

  strcpy(dfifo_files[0], "/tmp/spistream_d0.fifo"); // FIFOs for data
  strcpy(dfifo_files[1], "/tmp/spistream_d1.fifo"); // (STM -> daemon -> client)
  strcpy(dfifo_files[2], "/tmp/spistream_d2.fifo");
  strcpy(dfifo_files[3], "/tmp/spistream_d3.fifo");
  strcpy(cfifo_files[0], "/tmp/spistream_c0.fifo"); // FIFOs for commands
  strcpy(cfifo_files[1], "/tmp/spistream_c1.fifo"); // (client -> daemon -> STM)
  strcpy(cfifo_files[2], "/tmp/spistream_c2.fifo");
  strcpy(cfifo_files[3], "/tmp/spistream_c3.fifo");

  for (fifo_idx = 0; fifo_idx < 4; fifo_idx++) {
    fprintf(stderr, "Open data stream %s ... \n", dfifo_files[fifo_idx]);
    dfifo[fifo_idx] = open(dfifo_files[fifo_idx], O_RDONLY | O_NONBLOCK);
    fprintf(stderr, " ...\n");
  }

  return 1;

  for (fifo_idx = 0; fifo_idx < 3; fifo_idx++) {
    fprintf(stderr, "Open command stream %s ... \n", cfifo_files[fifo_idx]);
    cfifo[fifo_idx] = open(cfifo_files[fifo_idx], O_WRONLY);
    if (cfifo[fifo_idx] < 0) {
      fprintf(stderr, " failed\n");
      return 0;
    }
  }
  return 1;
}

static void main_exit(void)
{
  fprintf(stderr, "Bye!\n");
}

static void parse_command_line(int argc, char **argv)
{
}

static void on_kill(int signum)
{
  fprintf(stderr, "Exiting, got signal %d\n", signum);
  main_exit();
  exit(1);
}
