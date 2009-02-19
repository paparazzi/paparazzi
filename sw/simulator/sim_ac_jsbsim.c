/*
 * $Id$
 *  
 * Copyright (C) 2008 Gautier Hattenberger
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

#include <JSBSim/FGFDMExec.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "aircraft.h"

#include "main_ap.h"
#include "main_fbw.h"


/* 60Hz <-> 17ms */
#define TIMEOUT_PERIOD 17
#define DT (TIMEOUT_PERIOD*1e-3)
double sim_time;

#define DT_DISPLAY 0.04
double disp_time;

static void sim_gps_feed_data(void);
static void sim_ir_feed_data(void);

static void     sim_parse_options(int argc, char** argv);
static void     sim_init(void);
static gboolean sim_periodic(gpointer data);
static void     sim_display(void);

static void ivy_transport_init(void);
static void on_DL_SETTING(IvyClientPtr app __attribute__ ((unused)), 
    void *user_data __attribute__ ((unused)), 
    int argc __attribute__ ((unused)), char *argv[]);


static void sim_init(void) {

  sim_time = 0.;
  disp_time = 0.;
  
  // init flight model

  // init sensors

  // init environment

  ivy_transport_init();

  // main init
  init_fbw();
  init_ap();

}


static gboolean sim_periodic(gpointer data __attribute__ ((unused))) {
  /* read actuators positions */

  // wind model

  // flight model

  // sensors
  
  sim_time += DT;

  /* outputs models state */
  sim_display();

  /* run the airborne code */
  
  // feed a rc frame and signal event

  // process it
  event_task_ap();
  event_task_fbw();

  //if (booz_sensors_model_baro_available()) {
  //  Booz2BaroISRHandler(bsm.baro);
  //  booz2_main_event();
  //}

  //if (booz_sensors_model_gyro_available()) {
  //  booz2_imu_b2_feed_data();
  //  booz2_main_event();
  //}

  //if (booz_sensors_model_gps_available()) {
  //  sim_gps_feed_data();
  //  booz2_main_event();
  //}

  //if (booz_sensors_model_mag_available()) {
  //  sim_mag_feed_data();
  //  booz2_main_event();
  //}

  periodic_task_ap();
  periodic_task_fbw();

  return TRUE;
}

#include "gps.h"
static void sim_gps_feed_data(void) {
}


#define RPM_OF_RAD_S(a) ((a)*60./M_PI)
static void sim_display(void) {
}

int main ( int argc, char** argv) {

  sim_parse_options(argc, argv);

  sim_init();

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  g_timeout_add(TIMEOUT_PERIOD, sim_periodic, NULL);

  g_main_loop_run(ml);

  return 0;
}


static void ivy_transport_init(void) {
  IvyInit ("Paparazzi sim " + AC_ID, "READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");
}


static void sim_parse_options(int argc, char** argv) {

  static const char* usage =
"Usage: %s [options]\n"
" Options :\n";


//  while (1) {
//
//    static struct option long_options[] = {
//      {"fg_host", 1, NULL, 0},
//      {"fg_port", 1, NULL, 0},
//      {"js_dev", 1, NULL, 0},
//      {0, 0, 0, 0}
//    };
//    int option_index = 0;
//    int c = getopt_long(argc, argv, "j:",
//			long_options, &option_index);
//    if (c == -1)
//      break;
//    
//    switch (c) {
//    case 0:
//      switch (option_index) {
//      case 0:
//	fg_host = strdup(optarg); break;
//      case 1:
//	fg_port = atoi(optarg); break;
//      case 2:
//	joystick_dev = strdup(optarg); break;
//      }
//      break;
//
//    case 'j':
//      joystick_dev = strdup(optarg);
//      break;
//    
//    default: /* ’?’ */
//      printf("?? getopt returned character code 0%o ??\n", c);
//      fprintf(stderr, usage, argv[0]);
//      exit(EXIT_FAILURE);
//    }
//  }
}
