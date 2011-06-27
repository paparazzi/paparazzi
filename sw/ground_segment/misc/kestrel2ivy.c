/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2011 Andreas Gaeb, Max Chtangeev
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

/** \file kestrel2ivy.c
 *  \brief Connect a Kestrel 4500 Bluetooth weather station to the Paparazzi system
 *
 * The program communicates with a Kestrel 4500 weather station connected
 * to a serial port via bluethooth.  It asks for new data (Kestrel S command) in the
 * specified intervals, extracts the relevant data (ambient pressure and
 * temperature, wind speed and direction) and broadcasts this via the Ivy bus.
 *
 * At the moment, the Ivy messages should be sent with the ID of the actually
 * flying aircraft, which integrates them into the log file, as long as the
 * aircraft sends its alive message.
 *
 * Useful links:
 * - <a href="http://www.nkhome.com/kestrel/kestrel-4500</a>
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
enum {DT = 0,MG,TR,WS,CW,HW,TP,WC,RH,HI,DP,WB,BP,AL,DA};

#define PACKET_LENGTH 256
#define MIN_PACKET_LENGTH 103
#define NUM_PARAMS DA+1
#define CMD_LENGTH 8

// global variables
int         fd;
int         ac_id = 1;
const char *device;
char        packet[PACKET_LENGTH];

TimerId     tid;
BOOL        want_alive_msg = FALSE;


/// Handler for Ctrl-C, exits the main loop
void sigint_handler(int sig) {
  IvyStop();
  TimerRemove(tid);
  close(fd);
}

/// Open the serial port with the appropiate settings
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
  // 8 data bits, 1 stop bit, no parity, 9600 Baud
  options.c_cflag = CLOCAL | CREAD | CS8 | B9600;

  // write options back to port
  tcsetattr(fd, TCSANOW, &options);
}

/// Send an 'S' command (read sensor data) to the station and get the packet back
BOOL send_S() {

  char cmd[CMD_LENGTH] = "S\n\r";
  
  int bytes = write(fd, cmd, strlen(cmd));
  
  bytes = read(fd, packet, PACKET_LENGTH);

  if (bytes < MIN_PACKET_LENGTH) {
    fprintf(stderr, "Received packet is incomplete, only %i of %i bytes\n",
	    bytes, PACKET_LENGTH);
    return FALSE;
  
  } else {
    return TRUE;
  }
}

/// Populate values[] array with floats extracted from the packet string
int getValues(char* msg, float* values){
  int i;
  char* tok;
  const char delim = ',';

  tok = strtok(msg, &delim);

  for(i=0; i<NUM_PARAMS; i++) {
    if (tok == NULL) return 0;
    
    values[i] = strtof(tok, NULL);
    tok = strtok(NULL, &delim);
  }
  return 1;
}

/// Get the relevant data from the packet and sent it as Ivy message
void decode_and_send_to_ivy() {
  // get relevant data and convert to SI units

  // contents of the kestrel S message response:
  // DT,  MG,  TR,  WS,  CW,  HW, TP, WC, RH, HI, DP, WB,  BP,  AL, DA
  //  s, Mag, True, mph, mph, mph, F,  F,  %,  F,  F,  F, inHg, ft, ft
  //
  // where:
  //
  //   DT is the date and time in seconds since 1st January 2000, 
  //   AV is air velocity 
  //   AF is air flow
  //   MG is the compass magnetic direction
  //   TR is the compass true direction
  //   WS is the wind speed
  //   CW is the crosswind
  //   HW is the headwind
  //   TP is the temperature, 
  //   WC is the wind chill,
  //   RH is the humidity,
  //   EV is the evaporation rate
  //   CT is the concrete temperature
  //   HR, MO (moisture) are the humidity ratio
  //   HI is the heat index,
  //   DP is the dew point,
  //   WB is the wet bulb
  //   AP is absolute pressure
  //   BP is the pressure,   
  //   AL is the altitude,     
  //   DA is the density altitude
  //   AD is air density
  //   RA is relative air density


  float values[NUM_PARAMS];
  float pstatic_Pa, temp_degC, windspeed_mps, winddir_deg;

  if (want_alive_msg)
    IvySendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n", ac_id);

  if (getValues(packet + MIN_PACKET_LENGTH, values)){
    pstatic_Pa    = values[BP] * 3386.4;            // original is inches Hg
    temp_degC     = (values[TP] - 32.0) * 5.0/9.0;  // original is deg F
    windspeed_mps = values[WS] * 0.44704;           // original is miles per hour
    winddir_deg   = values[MG];
    
    // format has to match declaration in conf/messages.xml
    IvySendMsg("%d WEATHER %f %f %f %f\n",
	       ac_id, pstatic_Pa, temp_degC, windspeed_mps, winddir_deg);
  }
}

/// Get data from the station and send it via Ivy
// this function is executed by the timer
void handle_timer (TimerId id, void *data, unsigned long delta) {
  if (send_S()) decode_and_send_to_ivy();
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
    *defaultdevice = "/dev/rfcomm0";
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
  IvyInit("kestrel2ivy", "READY", 0, 0, 0, 0);
  IvyStart(bus);

  // create timer (Ivy)
  tid = TimerRepeatAfter (0, delay, handle_timer, 0);

	/* main loop */
#if IVYMINOR_VERSION == 8
        IvyMainLoop (NULL,NULL);
#else
        IvyMainLoop ();
#endif
  return 0;
}
