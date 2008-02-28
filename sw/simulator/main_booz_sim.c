/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Drouin
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

#include "booz_flightgear.h"
#include "booz_joystick.h"

#include "booz_controller_main.h"
#include "booz_filter_main.h"


//char* fg_host = "127.0.0.1";
char* fg_host = "10.31.4.107";
//char* fg_host = "192.168.1.191";
unsigned int fg_port = 5501;
char* joystick_dev = "/dev/input/js0";

/* 250Hz <-> 4ms */
#define TIMEOUT_PERIOD 4
#define DT (TIMEOUT_PERIOD*1e-3)
double sim_time;

#define DT_DISPLAY 0.04
double disp_time;

double booz_sim_actuators_values[] = {0., 0., 0., 0.};

static void booz_sim_parse_options(int argc, char** argv);
static gboolean booz_sim_periodic(gpointer data);
static inline void booz_sim_display(void);

static void booz_sim_set_ppm_from_joystick( void );
static void booz_sim_read_actuators( void );

#ifdef SIM_UART
static void sim_uart_init(void);
#endif
#if defined DOWNLINK_TRANSPORT && DOWNLINK_TRANSPORT == IvyTransport
static void ivy_transport_init(void);
static void on_DL_SETTING(IvyClientPtr app, void *user_data, int argc, char *argv[]);
#endif


#include "booz_imu.h"
#include "booz_estimator.h"
#include "radio_control.h"
#include "actuators.h"
#include "booz_inter_mcu.h"

volatile bool_t ppm_valid;

static gboolean booz_sim_periodic(gpointer data __attribute__ ((unused))) {
  /* read actuators positions */
  booz_sim_read_actuators();

  /* run our models */
  if (sim_time > 3.)
    bfm.on_ground = FALSE;
    /* no fdm at start to allow for filter initialisation */
    /* it sucks, I know */
  booz_flight_model_run(DT, booz_sim_actuators_values);

  booz_sensors_model_run(DT);
  
  booz_wind_model_run(DT);

  sim_time += DT;

  /* call the filter periodic task to run telemetry function            */
  booz_filter_main_periodic_task();
  
  /* feed sensors */
  if (booz_sensor_model_gyro_available()) {
    max1167_hw_feed_value(sim_time, bsm.gyro, bsm.accel);
    booz_filter_main_event_task();
  }

  if (booz_sensor_model_mag_available()) {
    micromag_hw_feed_value(sim_time, bsm.mag);
    booz_filter_main_event_task();
  }

  if (booz_sensor_model_baro_available()) {
    scp1000_hw_feed_value(sim_time, bsm.baro);
    booz_filter_main_event_task();
  }

  if (booz_sensor_model_gps_available()) {
    //    scp1000_hw_feed_value(sim_time, bsm.baro);
    booz_filter_main_event_task();
  }

  /* process sensors events */
  /* it will run the filter and the inter-process communication which   */
  /* will post a BoozLinkMcuEvent in the Controller process             */

  /* process the BoozLinkMcuEvent                                       */
  /* this will update the controller estimator                          */
  booz_controller_main_event_task();
#if 0
  /* cheat in simulation : psi not available from filter yet */
  //  booz_estimator_set_psi(bfm.state->ve[BFMS_PSI]);
  /* in simulation compute dcm as a helper for for nav */
  booz_estimator_compute_dcm();
  /* in simulation feed speed and pos estimations ( with a pos sensor :( ) */
  booz_estimator_set_speed_and_pos(bsm.speed_sensor->ve[AXIS_X], 
				   bsm.speed_sensor->ve[AXIS_Y], 
				   bsm.speed_sensor->ve[AXIS_Z],
				   bsm.pos_sensor->ve[AXIS_X], 
				   bsm.pos_sensor->ve[AXIS_Y], 
				   bsm.pos_sensor->ve[AXIS_Z] );
#endif

  /* post a radio control event */
  booz_sim_set_ppm_from_joystick();
  ppm_valid = TRUE;
  /* and let the controller process it */
  booz_controller_main_event_task();

  /* call the controller periodic task to run control loops            */
  booz_controller_main_periodic_task();


  booz_sim_display();

  return TRUE;
}

int main ( int argc, char** argv) {

  booz_sim_parse_options(argc, argv);

  sim_time = 0.;
  disp_time = 0.;
  
  booz_flight_model_init();

  booz_sensors_model_init();

  booz_wind_model_init();

  booz_flightgear_init(fg_host, fg_port);

#ifdef SIM_UART
  sim_uart_init();
#endif 

#if defined DOWNLINK_TRANSPORT && DOWNLINK_TRANSPORT == IvyTransport
  ivy_transport_init();
#endif

  booz_joystick_init(joystick_dev);

  booz_controller_main_init();

  booz_filter_main_init();
  

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  g_timeout_add(TIMEOUT_PERIOD, booz_sim_periodic, NULL);

  g_main_loop_run(ml);
 
  return 0;
}


/////////////////////
// Helpers
////////////////////

#define RPM_OF_RAD_S(a) ((a)*60./M_PI)
static inline void booz_sim_display(void) {
  if (sim_time >= disp_time) {
    disp_time+= DT_DISPLAY;
    booz_flightgear_send();
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
    IvySendMsg("148 BOOZ_SIM_GYRO_BIAS %f %f %f",  
	       DegOfRad(bsm.gyro_bias_random_walk_value->ve[AXIS_P]),
	       DegOfRad(bsm.gyro_bias_random_walk_value->ve[AXIS_Q]), 
	       DegOfRad(bsm.gyro_bias_random_walk_value->ve[AXIS_R]));
    IvySendMsg("148 BOOZ_SIM_RANGE_METER %f",  
	       bsm.range_meter);
    //    IvySendMsg("148 BOOZ_SIM_WIND %f %f %f",  
    //	       bwm.velocity->ve[AXIS_X], 
    //	       bwm.velocity->ve[AXIS_Y], 
    //	       bwm.velocity->ve[AXIS_Z]);
  }
}



#define FAKE_JOYSTICK
#include "booz_joystick_fake.h"
static void booz_sim_set_ppm_from_joystick( void ) {
#ifndef FAKE_JOYSTICK
  ppm_pulses[RADIO_THROTTLE] = 1223 + booz_joystick_value[JS_THROTTLE] * (2050-1223);
  ppm_pulses[RADIO_PITCH]    = 1498 + booz_joystick_value[JS_PITCH]    * (2050-950);
  ppm_pulses[RADIO_ROLL]     = 1500 + booz_joystick_value[JS_ROLL]     * (2050-950);
  ppm_pulses[RADIO_YAW]      = 1493 + booz_joystick_value[JS_YAW]      * (2050-950);
  ppm_pulses[RADIO_MODE]     = 1500 + booz_joystick_value[JS_MODE]     * (2050-950);
  //  printf("joystick mode     %f %d\n", booz_joystick_value[JS_MODE], ppm_pulses[RADIO_MODE]);
  //  printf("joystick throttle %f %d\n", booz_joystick_value[JS_THROTTLE], ppm_pulses[RADIO_THROTTLE]);
  //  printf("joystick yaw %f %d\n", booz_joystick_value[JS_YAW], ppm_pulses[RADIO_YAW]);
  //  printf("joystick pitch %f %d\n", booz_joystick_value[JS_PITCH], ppm_pulses[RADIO_PITCH]);
  //  printf("joystick roll %f %d\n", booz_joystick_value[JS_ROLL], ppm_pulses[RADIO_ROLL]);
#else
  ppm_pulses[RADIO_THROTTLE] = 1223 + 0.4 * (2050-1223);
  //BREAK_MTT();
  //  WALK_OVAL();
  TWO_POINTS();
  // CIRCLE();
  //HOVER();
  // TOUPIE();
  //ATTITUDE_ROLL_STEPS();
  //ATTITUDE_PITCH_STEPS();
  //  ATTITUDE_YAW_STEPS();
#endif
}


static void booz_sim_read_actuators( void ) {
#if 0
  booz_sim_actuators_values[SERVO_MOTOR_BACK] = (actuators[SERVO_MOTOR_BACK] - 1200)/(double)(1850-1200);
  booz_sim_actuators_values[SERVO_MOTOR_FRONT] = (actuators[SERVO_MOTOR_FRONT] - 1200)/(double)(1850-1200);
  booz_sim_actuators_values[SERVO_MOTOR_RIGHT] = (actuators[SERVO_MOTOR_RIGHT] - 1200)/(double)(1850-1200);
  booz_sim_actuators_values[SERVO_MOTOR_LEFT] = (actuators[SERVO_MOTOR_LEFT] - 1200)/(double)(1850-1200);
#else
  booz_sim_actuators_values[SERVO_MOTOR_BACK] = (actuators[SERVO_MOTOR_BACK]   - 0)/(double)(255);
  booz_sim_actuators_values[SERVO_MOTOR_FRONT] = (actuators[SERVO_MOTOR_FRONT] - 0)/(double)(255);
  booz_sim_actuators_values[SERVO_MOTOR_RIGHT] = (actuators[SERVO_MOTOR_RIGHT] - 0)/(double)(255);
  booz_sim_actuators_values[SERVO_MOTOR_LEFT] = (actuators[SERVO_MOTOR_LEFT]   - 0)/(double)(255);
#endif

}

static void booz_sim_parse_options(int argc, char** argv) {

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


#if defined DOWNLINK_TRANSPORT && DOWNLINK_TRANSPORT == IvyTransport

static void ivy_transport_init(void) {
  IvyInit ("BoozSim", "BoozSim READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");
}

#include "std.h"
#include "settings.h"
#include "booz_controller_telemetry.h"
static void on_DL_SETTING(IvyClientPtr app __attribute__ ((unused)), 
			  void *user_data __attribute__ ((unused)), 
			  int argc __attribute__ ((unused)), char *argv[]){
  uint8_t index = atoi(argv[2]);
  float value = atof(argv[3]);
  DlSetting(index, value);
  DOWNLINK_SEND_DL_VALUE(&index, &value);
  printf("setting %d %f\n", index, value);
}

#endif



#ifdef SIM_UART
#define AC_ID 148
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "sim_uart.h"

static void sim_uart_init(void) {
  char link_pipe_name[128];
  sprintf(link_pipe_name, "/tmp/pprz_link_%d", AC_ID);
  struct stat st;
  if (stat(link_pipe_name, &st)) {
    if (mkfifo(link_pipe_name, 0644) == -1) {
      perror("make pipe");
      exit (10);
    }
  }     
  if ( !(pipe_stream = fopen(link_pipe_name, "w")) ) {
    perror("open pipe");
    exit (10);
  }
}
#endif /* SIM_UART */













