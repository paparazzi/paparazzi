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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#define TIMEOUT_PERIOD  100

#define DEFAULT_AC_ID       1

/* Options */
char * device_name    = NULL;
int aircraft_id       = DEFAULT_AC_ID;

void parse_args(int argc, char * argv[])
{
  int i;

  for (i = 1; i < argc; i++) {
    if      (!strcmp(argv[i],"-d") && i<argc-1) device_name = argv[++i];
    else if (!strcmp(argv[i],"-a") && i<argc-1) aircraft_id = atoi(argv[++i]);
    else if (!strcmp(argv[i],"-h")) goto l_help;
  }
  return;

l_help:
  printf("Usage:\n");
  printf("  %s <option> [<option>...]\n",argv[0]);
  printf("Options:\n");
  printf("  -d <string>  device name\n");
  printf("  -a <int>     aircraft id (default: %d)\n",DEFAULT_AC_ID);
  printf("  -h           display this help\n");
  exit(1);
}


static gboolean joystick_periodic(gpointer data __attribute__ ((unused))) {

	stick_read();

	IvySendMsg("dl COMMANDS_RAW %d 2 %d %d", aircraft_id, stick_axis_values[0], stick_axis_values[1]);

	return 1;
}


int main ( int argc, char** argv) {

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  parse_args(argc, argv);
  
  IvyInit ("IvyCtrlJoystick", "IvyCtrlJoystick READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  if (stick_init(device_name) != 0) return 0;
  
  g_timeout_add(TIMEOUT_PERIOD, joystick_periodic, NULL);
  
  g_main_loop_run(ml);

  return 0;
}
