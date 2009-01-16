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
#include "booz_rc_sim.h"

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
static void     booz_sim_read_actuators(void);

static void ivy_transport_init(void);
static void on_DL_SETTING(IvyClientPtr app __attribute__ ((unused)), 
			  void *user_data __attribute__ ((unused)), 
			  int argc __attribute__ ((unused)), char *argv[]);


static void booz2_sim_init(void) {

  sim_time = 0.;
  disp_time = 0.;
  
  booz_flight_model_init();

  booz_sensors_model_init(sim_time);

  booz_wind_model_init();

  ivy_transport_init();

  booz2_main_init();

}

#include "booz2_analog_baro.h"
#include "booz2_imu.h"

static gboolean booz2_sim_periodic(gpointer data __attribute__ ((unused))) {
  /* read actuators positions */
  booz_sim_read_actuators();

  /* run our models */
  if (sim_time > 10.)
    bfm.on_ground = FALSE;

  booz_wind_model_run(DT);

  booz_flight_model_run(DT, booz_sim_actuators_values);

  booz_sensors_model_run(sim_time);
  
  sim_time += DT;

  /* outputs models state */
  booz2_sim_display();

  /* run the airborne code */
  
  // feed a rc frame and signal event
  BoozRcSimFeed(sim_time);
  // process it
  booz2_main_event();

  if (booz_sensors_model_baro_available()) {
    Booz2BaroISRHandler(bsm.baro);
    booz2_main_event();
  }
  if (booz_sensors_model_gyro_available()) {
    booz2_imu_b2_feed_data();
    booz2_main_event();
  }

  //  printf("throttle %d\n", rc_values[RADIO_THROTTLE]);
  //  printf("yaw %d\n", rc_values[RADIO_YAW]);

  booz2_main_periodic();
  

 
  return TRUE;
}

#define RPM_OF_RAD_S(a) ((a)*60./M_PI)
static void booz2_sim_display(void) {
  if (sim_time >= disp_time) {
    disp_time+= DT_DISPLAY;
    //    booz_flightgear_send();
    IvySendMsg("%d BOOZ_SIM_RPMS %f %f %f %f",  
	       AC_ID,
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_F]), 
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_B]), 
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_L]),
	       RPM_OF_RAD_S(bfm.state->ve[BFMS_OM_R]) );
    IvySendMsg("%d BOOZ_SIM_RATE_ATTITUDE %f %f %f %f %f %f",  
	       AC_ID,
	       DegOfRad(bfm.state->ve[BFMS_P]), 
	       DegOfRad(bfm.state->ve[BFMS_Q]), 
	       DegOfRad(bfm.state->ve[BFMS_R]),
	       DegOfRad(bfm.state->ve[BFMS_PHI]), 
	       DegOfRad(bfm.state->ve[BFMS_THETA]), 
	       DegOfRad(bfm.state->ve[BFMS_PSI]));
    IvySendMsg("%d BOOZ_SIM_SPEED_POS %f %f %f %f %f %f",  
	       AC_ID,
	       (bfm.state->ve[BFMS_U]), 
	       (bfm.state->ve[BFMS_V]), 
	       (bfm.state->ve[BFMS_W]),
	       (bfm.state->ve[BFMS_X]), 
	       (bfm.state->ve[BFMS_Y]), 
	       (bfm.state->ve[BFMS_Z]));
    IvySendMsg("%d BOOZ_SIM_WIND %f %f %f",  
    	       AC_ID,
	       bwm.velocity->ve[AXIS_X], 
    	       bwm.velocity->ve[AXIS_Y], 
    	       bwm.velocity->ve[AXIS_Z]);
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
  IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");
}

#if 1
#include "std.h"
#include "settings.h"
#include "dl_protocol.h"
#include "downlink.h"
static void on_DL_SETTING(IvyClientPtr app __attribute__ ((unused)), 
			  void *user_data __attribute__ ((unused)), 
			  int argc __attribute__ ((unused)), char *argv[]){
  uint8_t index = atoi(argv[2]);
  float value = atof(argv[3]);
  DlSetting(index, value);
  DOWNLINK_SEND_DL_VALUE(&index, &value);
  //  printf("setting %d %f\n", index, value);
}
#endif


#include "actuators.h"
static void booz_sim_read_actuators(void) {

  
  printf("actatuors %d %d %d %d\n",  
	 Actuator(SERVO_FRONT), Actuator(SERVO_BACK), Actuator(SERVO_RIGHT), Actuator(SERVO_LEFT));
  printf("actatuors %d %d %d %d\n",  
	 Actuator(SERVO_FRONT), Actuator(SERVO_BACK), Actuator(SERVO_RIGHT), Actuator(SERVO_LEFT));

  booz_sim_actuators_values[0] = (double)Actuator(SERVO_FRONT) / SUPERVISION_MAX_MOTOR;
  booz_sim_actuators_values[1] = (double)Actuator(SERVO_BACK)  / SUPERVISION_MAX_MOTOR;
  booz_sim_actuators_values[2] = (double)Actuator(SERVO_RIGHT) / SUPERVISION_MAX_MOTOR;
  booz_sim_actuators_values[3] = (double)Actuator(SERVO_LEFT)  / SUPERVISION_MAX_MOTOR;

  //  printf("%f %f %f %f\n",  booz_sim_actuators_values[0], booz_sim_actuators_values[1], booz_sim_actuators_values[2], booz_sim_actuators_values[3]);

}

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
