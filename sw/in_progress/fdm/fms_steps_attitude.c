/*
 * fms_steps_attitude
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

#include <inttypes.h>
#include "booz2_fms_extern.h"


#define TIMEOUT_PERIOD  100
#define DEFAULT_AC_ID   150
#define STEP_VAL 2
#define STEP_PERIOD 3

static int aircraft_id = DEFAULT_AC_ID;
static int counter;

static gboolean periodic(gpointer data __attribute__ ((unused)));


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

/*
   <field name="ac_id" type="uint8"/>
   <field name="h_mode" type="uint8" values="RATE|ATTITUDE|SPEED|POS"/>
   <field name="v_mode" type="uint8" values="DIRECT|CLIMB|HEIGHT"/>
   <field name="pad0" type="uint8"/>
   <field name="pad1" type="uint8"/>
   <field name="pad2" type="uint8"/>
   <field name="v_sp" type="int32"/>
   <field name="h_sp" type="int32[]"/>
*/

static gboolean periodic(gpointer data __attribute__ ((unused))) {
  
  counter++;

  uint8_t h_mode = FMS_H_MODE_ATTITUDE; 
  uint8_t v_mode = FMS_V_MODE_DIRECT; 
  int32_t pitch  = FMS_ATTITUDE_OF_DEG(0);
  int32_t roll   = FMS_ATTITUDE_OF_DEG(STEP_VAL);
  int32_t yaw    = FMS_ATTITUDE_OF_DEG(0);
  int32_t unused = 0;
  if ((counter%(2*STEP_PERIOD)) >= STEP_PERIOD) roll = -roll;

  IvySendMsg("dl BOOZ2_FMS_COMMAND %d %d %d %d d %d %d %d,%d,%d", aircraft_id, h_mode, v_mode, unused, unused, unused, unused, roll, pitch, yaw);
  return TRUE;
}


int main ( int argc, char** argv) {

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  parse_args(argc, argv);
  
  IvyInit ("IvyBooz2FmsAttStep", "IvyBooz2FmsAttStep READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  g_timeout_add(TIMEOUT_PERIOD, periodic, NULL);
  
  g_main_loop_run(ml);

  return 0;
}
