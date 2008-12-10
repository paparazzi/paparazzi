/*
 * fdm_step
 *
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#define TIMEOUT_PERIOD  100

#define DEFAULT_AC_ID       1

/* Options */
static int aircraft_id       = DEFAULT_AC_ID;
static int counter;

void parse_args(int argc, char * argv[])
{
  int i;

  for (i = 1; i < argc; i++) {
    if (!strcmp(argv[i],"-a") && i<argc-1) aircraft_id = atoi(argv[++i]);
    else if (!strcmp(argv[i],"-h")) goto l_help;
  }
  return;

l_help:
  printf("Usage:\n");
  printf("  %s <option> [<option>...]\n",argv[0]);
  printf("Options:\n");
  printf("  -a <int>     aircraft id (default: %d)\n",DEFAULT_AC_ID);
  printf("  -h           display this help\n");
  exit(1);
}

#define STEP_VAL 127
static gboolean periodic(gpointer data __attribute__ ((unused))) {
  int roll, pitch;
  counter++;
  pitch = 0;
  roll = STEP_VAL;
  if ((counter%6) >=3) roll = -roll;
    
  IvySendMsg("dl COMMANDS_RAW %d %d,%d", aircraft_id, roll, pitch);
  return TRUE;
}


int main ( int argc, char** argv) {

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  parse_args(argc, argv);
  
  IvyInit ("IvyFdmStep", "IvyFdmStep READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  g_timeout_add(TIMEOUT_PERIOD, periodic, NULL);
  
  g_main_loop_run(ml);

  return 0;
}
