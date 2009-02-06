/*
 * main_stick
 *
 * send joystick control to paparazzi through ivy 
 *
 * based on Force Feedback: Constant Force Stress Test
 * Copyright (C) 2001 Oliver Hamann
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "usb_stick.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#define TIMEOUT_PERIOD  100

#define DEFAULT_AC_ID   1

#define MODE_ATTITUDE   0
#define MODE_HOVER      1

/* Global vars */
int fp_received_once  = 0;
long int lon_sp,lat_sp,alt_sp,psi_sp;
int last_button = 0;
int8_t last_up = 0, last_down = 0;

/* Options */
char * device_name    = NULL;
int aircraft_id       = DEFAULT_AC_ID;
int mode              = MODE_ATTITUDE;

void parse_args(int argc, char * argv[])
{
  int i;

  for (i = 1; i < argc; i++) {
    if      (!strcmp(argv[i],"-d") && i<argc-1) device_name = argv[++i];
    else if (!strcmp(argv[i],"-a") && i<argc-1) aircraft_id = atoi(argv[++i]);
    else if (!strcmp(argv[i],"-m") && i<argc-1) mode        = atoi(argv[++i]);
    else if (!strcmp(argv[i],"-h")) goto l_help;
  }
  return;

l_help:
  printf("Usage:\n");
  printf("  %s <option> [<option>...]\n",argv[0]);
  printf("Options:\n");
  printf("  -d <string>  device name\n");
  printf("  -a <int>     aircraft id (default: %d)\n",DEFAULT_AC_ID);
  printf("  -m <int>     joystick mode: (%d) ATTITUDE, (%d) HOVER. (default: %d)\n",MODE_ATTITUDE,MODE_HOVER,MODE_ATTITUDE);
  printf("  -h           display this help\n");
  exit(1);
}


#define STICK_DEADBAND 10
#define STICK_APPLY_DEADBAND(_v) (abs(_v) >= STICK_DEADBAND ? _v : 0)

#define ATTITUDE_COEF 1430  // RadOfDeg(10 / 128) << 20
#define CLIMB_COEF_SHIFT 6  // around 1 m/s
#define CLIMB_INCR (10*127) // 1 m step
#define YAW_RATE_COEF 50   // 4 deg/s = ATTITUDE_COEF / 10 (update frequncy)

#define bit_is_set(x, b) ((x >> b) & 0x1)

// ATTITUDE
static gboolean joystick_attitude_periodic(gpointer data __attribute__ ((unused))) {

	stick_read();

	int8_t roll = STICK_APPLY_DEADBAND(stick_axis_values[0]);
	int8_t pitch = STICK_APPLY_DEADBAND(stick_axis_values[1]);
	int8_t yaw_rate = STICK_APPLY_DEADBAND(stick_axis_values[2]);
  int climb = STICK_APPLY_DEADBAND(stick_axis_values[3]);

  int8_t last_up = bit_is_set(last_button,3);
  int8_t up = bit_is_set(stick_button_values,3);
  int8_t last_down = bit_is_set(last_button,1);
  int8_t down = bit_is_set(stick_button_values,1);
  if (up != last_up && up > 0) climb = - CLIMB_INCR;
  if (down != last_down && down > 0) climb = CLIMB_INCR;
  last_button = stick_button_values;

  if (! fp_received_once) return 1;

  alt_sp += (climb >> CLIMB_COEF_SHIFT);
  psi_sp += YAW_RATE_COEF * yaw_rate;
  IvySendMsg("dl BOOZ2_FMS_COMMAND %d %d %d %d %d %d %d",
      2, 2,
      alt_sp,
      ATTITUDE_COEF * roll,
      ATTITUDE_COEF * pitch,
      psi_sp,
      aircraft_id);

	return 1;
}

#define SPEED_COEF_SHIFT 3  // around 2 m/s (128 >> 3 ~= 2 / 6378137 * 180 / pi * 10^7 / 10)

// HOVER
static gboolean joystick_hover_periodic(gpointer data __attribute__ ((unused))) {

	stick_read();

	int8_t vx = STICK_APPLY_DEADBAND(stick_axis_values[0]);
	int8_t vy = STICK_APPLY_DEADBAND(stick_axis_values[1]);
	int8_t yaw_rate = STICK_APPLY_DEADBAND(stick_axis_values[2]);
  int climb = STICK_APPLY_DEADBAND(stick_axis_values[3]);

  int8_t last_up = bit_is_set(last_button,3);
  int8_t up = bit_is_set(stick_button_values,3);
  int8_t last_down = bit_is_set(last_button,1);
  int8_t down = bit_is_set(stick_button_values,1);
  if (up != last_up && up > 0) climb = - CLIMB_INCR;
  if (down != last_down && down > 0) climb = CLIMB_INCR;
  last_button = stick_button_values;
  //printf("ok\n");

  if (! fp_received_once) return 1;

  alt_sp += (climb >> CLIMB_COEF_SHIFT);
  lon_sp += (vx >> SPEED_COEF_SHIFT);
  lat_sp -= (vy >> SPEED_COEF_SHIFT);
  psi_sp += YAW_RATE_COEF * yaw_rate;
  //printf("%ld %ld %ld %ld\n",alt_sp,lon_sp,lat_sp,psi_sp);
  IvySendMsg("dl BOOZ2_FMS_COMMAND %d %d %d %d %d %d %d",
      3, 2,
      alt_sp,
      lat_sp,
      lon_sp,
      psi_sp,
      aircraft_id);

	return 1;
}

void readBOOZ2_FPIvyBus(IvyClientPtr app, void *data, int argc, char **argv) {

  //printf("%s\n",argv[0]);
  if (argc > 0) {
    sscanf(argv[0],"%*d %*s %*d %*d %*d %*d %*d %*d %*d %*d %ld %ld %ld %ld",
        &lon_sp, &lat_sp, &alt_sp, &psi_sp);

    fp_received_once = 1;
  }
}

int main ( int argc, char** argv) {

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  parse_args(argc, argv);
  
  IvyInit ("IvyCtrlJoystick", "IvyCtrlJoystick READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  if (stick_init(device_name) != 0) return 0;

  char bindMsgBOOZ2_FP[32];
  snprintf(bindMsgBOOZ2_FP,32,"%s%d%s","(",aircraft_id," BOOZ2_FP .*)");
  IvyBindMsg(readBOOZ2_FPIvyBus,0,bindMsgBOOZ2_FP);
 
  switch(mode) {
    case MODE_ATTITUDE:
      g_timeout_add(TIMEOUT_PERIOD, joystick_attitude_periodic, NULL);
      break;
    case MODE_HOVER:
      g_timeout_add(TIMEOUT_PERIOD, joystick_hover_periodic, NULL);
      break;
  }

  
  g_main_loop_run(ml);

  return 0;
}
