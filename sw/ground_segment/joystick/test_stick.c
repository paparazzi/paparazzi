/*
 *
 * This file is part of paparazzi.
 * test joystick detection and commands
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

#define TIMEOUT_PERIOD  100


/* Options */
char * device_name    = NULL;

void parse_args(int argc, char * argv[])
{
  int i;

  for (i = 1; i < argc; i++) {
    if      (!strcmp(argv[i],"-d") && i<argc-1) device_name = argv[++i];
    else if (!strcmp(argv[i],"-h")) goto l_help;
  }
  return;

l_help:
  printf("Usage:\n");
  printf("  %s <option> [<option>...]\n",argv[0]);
  printf("Options:\n");
  printf("  -d <string>  device name\n");
  printf("  -h           display this help\n");
  exit(1);
}


#define bit_is_set(x, b) ((x >> b) & 0x1)

static gboolean periodic(gpointer data __attribute__ ((unused))) {

	stick_read();

	return 1;
}

int main ( int argc, char** argv) {

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  parse_args(argc, argv);

  if (stick_init(device_name) != 0) return 1;

  g_timeout_add(TIMEOUT_PERIOD, periodic, NULL);

  g_main_loop_run(ml);

  return 0;
}
