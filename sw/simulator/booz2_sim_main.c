/*
 * $Id$
 *  
 * Copyright (C) 2008 Antoine Drouin
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

#include <glib.h>
#include <getopt.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "booz_flight_model.h"
#include "booz_sensors_model.h"
#include "booz_wind_model.h"

#include "booz2_main.h"


char* fg_host = "10.31.4.107";
unsigned int fg_port = 5501;
char* joystick_dev = "/dev/input/js0";

/* 250Hz <-> 4ms */
#define TIMEOUT_PERIOD 4
#define DT (TIMEOUT_PERIOD*1e-3)
double sim_time;

#define DT_DISPLAY 0.04
double disp_time;

double booz_sim_actuators_values[] = {0., 0., 0., 0.};

static void     booz2_sim_parse_options(int argc, char** argv);
static void     booz2_sim_init(void);
static gboolean booz2_sim_periodic(gpointer data);
static void     booz2_sim_display(void);

static void ivy_transport_init(void);

static void booz2_sim_init(void) {

  sim_time = 0.;
  disp_time = 0.;
  
  booz_flight_model_init();

  booz_sensors_model_init(sim_time);

  booz_wind_model_init();

  ivy_transport_init();

  //  booz2_main_init();

}

static gboolean booz2_sim_periodic(gpointer data __attribute__ ((unused))) {
  /* read actuators positions */
  //  booz_sim_read_actuators();

  /* run our models */
  //  if (sim_time > 3.)
  //    bfm.on_ground = FALSE;
    /* no fdm at start to allow for filter initialisation */
    /* it sucks, I know */
  booz_flight_model_run(DT, booz_sim_actuators_values);

  booz_sensors_model_run(sim_time);
  
  booz_wind_model_run(DT);

  sim_time += DT;


  booz2_sim_display();
 
  return TRUE;
}

#define RPM_OF_RAD_S(a) ((a)*60./M_PI)
static void booz2_sim_display(void) {
  if (sim_time >= disp_time) {
    disp_time+= DT_DISPLAY;
    //    booz_flightgear_send();
    IvySendMsg("148 BOOZ_SIM_RPMS %f %f %f %f",  
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_F]), 
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_B]), 
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_L]),
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_R]) );
    IvySendMsg("148 BOOZ_SIM_RATE_ATTITUDE %f %f %f %f %f %f",  
	       DegOfRad(bfm.state->ve[BFMS_P]), 
	       DegOfRad(bfm.state->ve[BFMS_Q]), 
	       DegOfRad(bfm.state->ve[BFMS_R]),
	       DegOfRad(bfm.state->ve[BFMS_PHI]), 
	       DegOfRad(bfm.state->ve[BFMS_THETA]), 
	       DegOfRad(bfm.state->ve[BFMS_PSI]));
    IvySendMsg("148 BOOZ_SIM_SPEED_POS %f %f %f %f %f %f",  
	       (bfm.state->ve[BFMS_U]), 
	       (bfm.state->ve[BFMS_V]), 
	       (bfm.state->ve[BFMS_W]),
	       (bfm.state->ve[BFMS_X]), 
	       (bfm.state->ve[BFMS_Y]), 
	       (bfm.state->ve[BFMS_Z]));
    //    IvySendMsg("148 BOOZ_SIM_WIND %f %f %f",  
    //	       bwm.velocity->ve[AXIS_X], 
    //	       bwm.velocity->ve[AXIS_Y], 
    //	       bwm.velocity->ve[AXIS_Z]);
  }
}

int main ( int argc, char** argv) {

  booz2_sim_parse_options(argc, argv);

  booz2_sim_init();

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  g_timeout_add(TIMEOUT_PERIOD, booz2_sim_periodic, NULL);

  g_main_loop_run(ml);

  return 0;
}


static void ivy_transport_init(void) {
  IvyInit ("BoozSim", "BoozSim READY", NULL, NULL, NULL, NULL);
  //  IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");
}

#if 0
#include "std.h"
//#include "settings.h"
//#include "booz_controller_telemetry.h"
static void on_DL_SETTING(IvyClientPtr app __attribute__ ((unused)), 
			  void *user_data __attribute__ ((unused)), 
			  int argc __attribute__ ((unused)), char *argv[]){
  uint8_t index = atoi(argv[2]);
  float value = atof(argv[3]);
  //  DlSetting(index, value);
  //  DOWNLINK_SEND_DL_VALUE(&index, &value);
  printf("setting %d %f\n", index, value);
}
#endif



static void booz2_sim_parse_options(int argc, char** argv) {

  static const char* usage =
"Usage: %s [options]\n"
" Options :\n"
"   -j --js_dev joystick device\n"
"   --fg_host flight gear host\n"
"   --fg_port flight gear port\n";


  while (1) {

    static struct option long_options[] = {
      {"fg_host", 1, NULL, 0},
      {"fg_port", 1, NULL, 0},
      {"js_dev", 1, NULL, 0},
      {0, 0, 0, 0}
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "j:",
			long_options, &option_index);
    if (c == -1)
      break;
    
    switch (c) {
    case 0:
      switch (option_index) {
      case 0:
	fg_host = strdup(optarg); break;
      case 1:
	fg_port = atoi(optarg); break;
      case 2:
	joystick_dev = strdup(optarg); break;
      }
      break;

    case 'j':
      joystick_dev = strdup(optarg);
      break;
    
    default: /* ’?’ */
      printf("?? getopt returned character code 0%o ??\n", c);
      fprintf(stderr, usage, argv[0]);
      exit(EXIT_FAILURE);
    }
  }
}
