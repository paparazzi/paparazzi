/*
 * Copyright (C) 2011 Andreas Gaeb
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

/** \file davis2ivy.c
 *  \brief Connect a Davis VantagePro weather station to the Paparazzi system
 *
 * The program communicates with a Davis VantagePro(2) weather station connected
 * to a serial port.  It asks for new data (Davis' LOOP command) in the
 * specified intervals, extracts the relevant data (ambient pressure and
 * temperature, wind speed and direction) and broadcasts this via the Ivy bus.
 *
 * At the moment, the Ivy messages should be sent with the ID of the actually
 * flying aircraft, which integrates them into the log file, as long as the
 * aircraft sends its alive message.
 *
 * Useful links:
 * - <a href="http://www.davisnet.com/weather/products/vantagepro.asp">Weather Stations</a>
 * - <a href="http://www.davisnet.com/support/weather/download/VantageSerialProtocolDocs_v230.pdf">Communication docs</a>
 *
 */


 #include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include <signal.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include <Ivy/timer.h>
#include <Ivy/version.h>


typedef enum { FALSE = 0, TRUE } BOOL;

#define PACKET_LENGTH 99


// global variables
int fd, ac_id = 1;
const char *device;
unsigned char packet[PACKET_LENGTH];
TimerId tid;
BOOL want_alive_msg = FALSE;


/// Handler for Ctrl-C, exits the main loop
void sigint_handler(int sig) {
  IvyStop();
  TimerRemove(tid);
  close(fd);
}

/// open the serial port with the appropiate settings
void open_port(const char* device) {
  fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    fprintf(stderr, "open_port: unable to open device %s - ", device);
    perror(NULL);
    exit(EXIT_FAILURE);
  }
  // setup connection options
  struct termios options;

  // get the current options
  tcgetattr(fd, &options);

  // set local mode, enable receiver, set comm. options:
  // 8 data bits, 1 stop bit, no parity, 19200 Baud
  options.c_cflag = CLOCAL | CREAD | CS8 | B19200;

  // write options back to port
  tcsetattr(fd, TCSANOW, &options);

}

/// disable transactions and empty queue
void reset_station() {
  char newline = '\n';
  char bytes __attribute__ ((unused));
  fprintf(stderr, "Resetting communication\n");
  // send a \n (wakeup and cancel all running transmits)
  bytes = write(fd, &newline, 1);
  // read and discard everything that might be left in the queue
  close(fd);
  sleep(1);
  open_port(device);
}

/// send a wakeup call to the station
BOOL wakeup(int tries) {
  int loops = tries, bytes;
  BOOL woken = FALSE;
  char buf[] = {0, 0};
  char newline = '\n';
  do {
    // send a \n
    bytes = write(fd, &newline, 1);
    // wait until station answers with \n\r
    usleep(30000);
    bytes = read(fd, buf, sizeof(buf));
    woken = (buf[0] == 10) && (buf[1] == 13);
  } while (!woken && loops-- > 0);
  if (!woken) {
    fprintf(stderr, "Could not wake up station: ");
    if (bytes < 1) fprintf(stderr, "no bytes received\n");
    else fprintf(stderr, "received %02x:%02x instead of \\n\\r\n", buf[0], buf[1]);
    reset_station();
  }
  return woken;
}

/// send a LOOP command (read sensor data) to the station and get the packet back
BOOL send_loop() {
  char msg[32], ack;
  // TODO maybe ask for more packets?
  snprintf(msg, sizeof(msg), "LOOP %i\n", 1);
  int bytes = write(fd, msg, strlen(msg));
  usleep(120000);
  bytes = read(fd, &ack, 1);
  if (bytes < 1 || ack != 0x06) {
    fprintf(stderr, "Failed to receive ACK from station\n");
    reset_station();
    return FALSE;
  }
  bytes = read(fd, packet, PACKET_LENGTH);
  if (bytes < PACKET_LENGTH) {
    fprintf(stderr, "Received packet is incomplete, only %i of %i bytes\n",
            bytes, PACKET_LENGTH);
    reset_station();
    return FALSE;
  } else {
    return TRUE;
  }
}

/// get the relevant data from the packet and sent it as Ivy message
void decode_and_send_to_ivy() {

  // check packet integrity
  char expected[] = "LOO";
  if (strncmp((char *)packet, expected, 3) != 0) {
    fprintf(stderr, "Received packet from the weather station which does not match the expected format\n");
    reset_station();
    return;
  }

  // TODO CRC checking (is rather involved for the Davis protocol)

  // get relevant data and convert to SI units
  // see chapter IX.1 of the protocol definition
  float
    pstatic_Pa = (packet[7] | packet[8] << 8)*3.386388640341, // original is inches Hg / 1000
    temp_degC = ((packet[12] | packet[13] << 8)/10.0 - 32.0)*5.0/9.0, // original is deg F / 10
    windspeed_mps = packet[14]*0.44704, // original is miles per hour
    winddir_deg = packet[16] | packet[17] << 8,
    rel_humidity = -1.; // TODO Get the real humidity value from message


  // TODO get the real MD5 for the aircraft id
  if (want_alive_msg)
    IvySendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n", ac_id);

  // format has to match declaration in var/messages.xml
  IvySendMsg("%d WEATHER %f %f %f %f %f\n",
    ac_id, pstatic_Pa, temp_degC, windspeed_mps, winddir_deg, rel_humidity);
}

/// Get data from the station and send it via Ivy
/** This function is executed by the timer
 */
void handle_timer (TimerId id, void *data, unsigned long delta) {
  if (wakeup(3) && send_loop()) decode_and_send_to_ivy();
}

void print_usage(int argc, char ** argv) {
  fprintf(stderr, "Usage: %s [-a] [-b <bus>] [-d <device>] [-i <aircraft_id>] [-s <delay time in seconds>]\n",
          argv[0]);
};

/// Main function
int main(int argc, char **argv) {
  // default values for options
  const char
    *defaultbus = "127.255.255.255:2010",
    *bus = defaultbus,
    *defaultdevice = "/dev/ttyUSB1";
  device = defaultdevice;
  long delay = 1000;

  // parse options
  char c;
  while ((c = getopt (argc, argv, "hab:d:i:s:")) != EOF) {
    switch (c) {
    case 'h':
      print_usage(argc, argv);
      exit(EXIT_SUCCESS);
      break;
    case 'a':
      want_alive_msg = TRUE;
      break;
    case 'b':
      bus = optarg;
      break;
    case 'd':
      device = optarg;
      break;
    case 'i':
      ac_id = atoi(optarg);
      break;
    case 's':
      delay = atoi(optarg)*1000;
      break;
    case '?':
      if (optopt == 'a' || optopt == 'b' || optopt == 'd' || optopt == 's')
        fprintf (stderr, "Option -%c requires an argument.\n", optopt);
      else if (isprint (optopt))
        fprintf (stderr, "Unknown option `-%c'.\n", optopt);
      else
        fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
      print_usage(argc, argv);
      exit(EXIT_FAILURE);
    default:
      abort ();
    }
  }


  // make Ctrl-C stop the main loop and clean up properly
  signal(SIGINT, sigint_handler);

  bzero (packet, PACKET_LENGTH);
  open_port(device);

  // setup Ivy communication
  IvyInit("davis2ivy", "READY", 0, 0, 0, 0);
  IvyStart(bus);

  // create timer
  tid = TimerRepeatAfter (0, delay, handle_timer, 0);

#if IVYMINOR_VERSION == 8
  IvyMainLoop (NULL,NULL);
#else
  IvyMainLoop ();
#endif

  return 0;
}
