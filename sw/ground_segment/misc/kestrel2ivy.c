/*
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
 *
 * A few notes:
 * On OS X, the bluetooth pairing is a little bit shaky. The serial port availability
 * seems to time out and give a resource busy error, and the device must be woken up
 * again. Easiest way to do this is to open the Bluetooth Preferences, select the
 * Kestrel device, and in the settings dropdown, select the Edit Serial Ports... option
 * (OS X 10.7 Lion tested). This seems to wake up the device and allow a new connection.
 *
 * Also noted (at least on OS X) if the update period is set at faster than five seconds
 * (-s 5), then the link is not fast enough and data is occasionally missed, leading to
 * missed samples. The five second max was recommended by NKHome tech support.
 *
 * Circular buffer implementation from http://en.wikipedia.org/wiki/Circular_buffer
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
enum {UNINIT = 0,GOT_START,GOT_END};

/* Opaque buffer element type.  This would be defined by the application. */
typedef struct { char value; } ElemType;

/* Circular buffer object */
typedef struct {
    int         size;   /* maximum number of elements           */
    int         start;  /* index of oldest element              */
    int         end;    /* index at which to write new element  */
    ElemType   *elems;  /* vector of elements                   */
} CircularBuffer;

#define PACKET_LENGTH 255
#define BUF_LENGTH (PACKET_LENGTH*4)
#define MIN_PACKET_LENGTH (103-3) // using the > character as the start
#define NUM_PARAMS DA+1
#define CMD_LENGTH 8

// global variables
int         fd;
int         ac_id = 1;
const char *device;
char        packet[PACKET_LENGTH];
char        tempBuf[BUF_LENGTH];
int         packetidx = 0;
CircularBuffer cb;
ElemType    elem = {0};
int msgState = UNINIT;

TimerId     tid;
BOOL        want_alive_msg = FALSE;
BOOL        metric_input = FALSE;


/**
 * Circular buffer initialization.
 */
void cbInit(CircularBuffer *cb, int size) {
  cb->size  = size + 1; /* include empty elem */
  cb->start = 0;
  cb->end   = 0;
  cb->elems = (ElemType *)calloc(cb->size, sizeof(ElemType));
}

/**
 * Free buffer memory, call before exit.
 */
void cbFree(CircularBuffer *cb) {
  free(cb->elems); /* OK if null */ }

/**
 * @returns True if buffer completely full.
 */
int cbIsFull(CircularBuffer *cb) {
  return (cb->end + 1) % cb->size == cb->start; }

/**
 * @returns True if nothing in buffer.
 */
int cbIsEmpty(CircularBuffer *cb) {
  return cb->end == cb->start; }

/**
 * @returns The number of free slots in the buffer.
 */
int cbAvailable(CircularBuffer *cb) {
  if (cb->end >= cb->start)
    return (cb->size - abs(cb->end - cb->start) - 1);
  else
    return (abs(cb->end - cb->start) - 1);
}

/**
 * Write an element, overwriting oldest element if buffer is full. App can
 * choose to avoid the overwrite by checking cbIsFull().
 */
void cbWrite(CircularBuffer *cb, ElemType *elem) {
  cb->elems[cb->end] = *elem;
  cb->end = (cb->end + 1) % cb->size;
  if (cb->end == cb->start)
    cb->start = (cb->start + 1) % cb->size; /* full, overwrite */
}

/**
 * Read oldest element, app must ensure !cbIsEmpty() first.
 */
void cbRead(CircularBuffer *cb, ElemType *elem) {
  *elem = cb->elems[cb->start];
  cb->start = (cb->start + 1) % cb->size;
}

/**
 * Handler for Ctrl-C, exits the main loop.
 */
void sigint_handler(int sig) {
  cbFree(&cb);
  IvyStop();
  TimerRemove(tid);
  close(fd);
}

/**
 * Open the serial port with the appropiate settings.
 */
void open_port(const char* device) {
  fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    fprintf(stderr, "kestrel2ivy: open_port: unable to open device %s - ", device);
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

/**
 * Send an 'S' command (read sensor data) to the station and get the packet back.
 */
BOOL send_S() {

  char cmd[CMD_LENGTH] = "S\r";

  int bytes = write(fd, cmd, strlen(cmd));
  if (bytes == -1) {
    perror("kestrel2ivy: Error writing command bytes");
    fprintf(stderr,"You may need to reset the connection.\n");
  }

  bzero(tempBuf, BUF_LENGTH);

  // repeat a few times within time limit, just return if done
  int result = 0;
  int j = 0;
  for(j = 0; j < 3; j++) {
    // delay to allow kestrel to give some data
    usleep(250000); // 0.25sec
    // read in one chunk, skip reads for the period if there is a read error
    if (result >= 0) {
      result = read(fd, tempBuf, cbAvailable(&cb));
      if (result < 0) {
        //perror("kestrel2ivy: Got a read error on the port");
      } else {
        int i = 0;
        for (i = 0; i < result; i++) {
          ElemType elemBuf;
          elemBuf.value = tempBuf[i];
          cbWrite(&cb, &elemBuf);
        }
      }
    }

    while (!cbIsEmpty(&cb)) {
      // First get the (approximate) start indicator
      // Then copy in the rest of the data
      // Stop copying in data at the (approximate) end indicator
      // Return if it looks like the last packet in the buffer, otherwise keep processing buffer
      ElemType tempElem;
      cbRead(&cb,&tempElem);
      char tempChar = tempElem.value;

      switch (msgState) {
        case UNINIT:
          if (tempChar == '>') {
            bzero(packet, PACKET_LENGTH); // clear the packet after getting values
            packetidx = 0;
            packet[packetidx++] = tempChar;
            msgState = GOT_START;
          }
          break;
        case GOT_START:
          if (tempChar == '>') {
            if (packetidx < MIN_PACKET_LENGTH) {
              // False alarm, not actually the start
              bzero(packet, PACKET_LENGTH);
              packetidx = 0;
              packet[packetidx++] = tempChar;
            } else {
              msgState = UNINIT;
              // if we think we have another full packet available, don't return
              if (cbAvailable(&cb) > (BUF_LENGTH - (MIN_PACKET_LENGTH + 80)))
                return TRUE;
            }
          }
          else {
            packet[packetidx++] = tempChar;
          }
          break;
        default:
          fprintf(stderr, "kestrel2ivy: Unknown parser state!\n");
      }
    }
  }
  return FALSE;
}

/**
 * Populate values[] array with floats extracted from the packet string.
 */
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

/**
 * Get the relevant data from the packet and send it as Ivy message
 */
void decode_and_send_to_ivy() {
  // get relevant data and convert to SI units

  // contents of the kestrel S message response in imperial units:
  // DT,  MG,  TR,  WS,  CW,  HW, TP, WC, RH, HI, DP, WB,  BP,  AL, DA
  //  s, Mag, True, mph, mph, mph, F,  F,  %,  F,  F,  F, inHg, ft, ft
  // in metric units:
  //  s,Mag,True,km/h,km/h,km/h,�C,�C,%,�C,�C,�C,hPa,m,m
  //
  // Sample packet in metric units:
  // S
  // > DT,MG,TR,WS,CW,HW,TP,WC,RH,HI,DP,WB,BP,AL,DA
  // s,Mag,True,km/h,km/h,km/h,�C,�C,%,�C,�C,�C,hPa,m,m
  // 399124138,000,000,0.0,0.0,0.0,22.5,22.5,37.6,21.4,7.3,13.5,931.5,701,1171
  // >
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
  //   RH is the relative humidity,
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
  float pstatic_Pa, temp_degC, windspeed_mps, winddir_deg, rel_humidity;

  if (want_alive_msg)
    IvySendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n", ac_id);

  if (getValues(packet + MIN_PACKET_LENGTH, values)){
    if (metric_input) {
      //If the unit outputs are set for SI (or closest match), use these conversions
      pstatic_Pa     = values[BP] * 100.0;             // original is hPa
      temp_degC      = values[TP];                     // original is deg C
      windspeed_mps  = values[WS] * 0.2778;            // original is km/h
      winddir_deg    = values[MG];
      rel_humidity   = values[RH];
    } else {
      //Otherwise use the default imperial units and convert to SI
      pstatic_Pa     = values[BP] * 3386.4;            // original is inches Hg
      temp_degC      = (values[TP] - 32.0) * 5.0/9.0;  // original is deg F
      windspeed_mps  = values[WS] * 0.44704;           // original is miles per hour
      winddir_deg    = values[MG];
      rel_humidity   = values[RH];
    }

    // format has to match declaration in conf/messages.xml
    IvySendMsg("%d WEATHER %f %f %f %f %f\n",
	       ac_id, pstatic_Pa, temp_degC, windspeed_mps, winddir_deg, rel_humidity);
  }
}

/**
 * Get data from the station and send it via Ivy.
 * This function is executed by the timer.
 */
void handle_timer (TimerId id, void *data, unsigned long delta) {
  if (send_S()) decode_and_send_to_ivy();
}

void print_usage(int argc, char ** argv) {
  static const char* usage =
"Usage: %s [-a] [-b <bus>] [-d <device>] [-i <aircraft_id>] [-m] [-s <delay time in seconds>]\n"
"  Options:\n"
"    -h           Display this help\n"
"    -a           Send aircraft ALIVE message (default is no)\n"
"    -b <bus>     Ivy bus (default is 127.255.255.255:2010)\n"
"    -d <device>  Weatherstation Device (default is /dev/ttyUSB1)\n"
"    -i <number>  Specify aircraft ID (default is 1)\n"
"    -s <number>  Specify station polling period in seconds (default is 5)\n"
"\n"
"  Note: kestrel2ivy may send messages with different aircraft IDs. The simplest way is to use\n"
"  the ID of the actually flying aircraft, thus the messages will show up in the messages window\n"
"  and in the log file as if they were sent from that aircraft. If you use another aircraft ID,\n"
"  you have to add the -a option to get the messages into the log file.\n"
"\n"
"  The shortest recommended sampling period is 5 seconds. Less than this may result in poor\n"
"  weather meter performance, missed packets, etc. (It can sometimes work down to 1 second,\n"
"  but it may stop working unexpectedly\n";

  fprintf(stderr, usage, argv[0]);
}

/** Main function.
 */
int main(int argc, char **argv) {
  // default values for options
  const char
    *defaultbus = "127.255.255.255:2010",
    *bus = defaultbus,
    *defaultdevice = "/dev/rfcomm0";
  device = defaultdevice;
  long delay = 5000;

  // parse options
  char c;
  while ((c = getopt (argc, argv, "hamb:d:i:s:")) != EOF) {
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
    case 'm':
      metric_input = TRUE;
      break;
    case 's':
      delay = atoi(optarg)*1000;
      if (delay < 5000) {
        fprintf(stderr,"kestrel2ivy: Warning: Setting the sampling period less than 5 seconds\n"
          "may result in poor sampling performance!\n");
      }
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

  // zero out and initialize buffers
  int bufferSize = BUF_LENGTH;
  cbInit(&cb, bufferSize);

  bzero(packet, PACKET_LENGTH);
  msgState = UNINIT;
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
