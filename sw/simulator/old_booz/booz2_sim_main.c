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
#include <sys/time.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "booz_flight_model.h"
#include "booz_sensors_model.h"
#include "booz_wind_model.h"
#include "booz_rc_sim.h"
#include "firmwares/rotorcraft/battery.h"

#include "main.h"


char* fg_host = "10.31.4.107";
unsigned int fg_port = 5501;
char* joystick_dev = "/dev/input/js0";

/* rate of the host mainloop */
#define HOST_TIMEOUT_PERIOD 4
struct timeval host_time_start;
double host_time_elapsed;
double host_time_factor = 1.;

/* 250Hz <-> 4ms */
#define SIM_DT (1./512.)
double sim_time;

#define DT_DISPLAY 0.04
double disp_time;

double booz_sim_actuators_values[] = {0., 0., 0., 0.};

static void sim_run_one_step(void);
#ifdef BYPASS_AHRS
static void sim_overwrite_ahrs(void);
#endif
#ifdef BYPASS_INS
static void sim_overwrite_ins(void);
#endif
static void sim_gps_feed_data(void);
static void sim_mag_feed_data(void);

static void     booz2_sim_parse_options(int argc, char** argv);
static void     booz2_sim_init(void);
static gboolean booz2_sim_periodic(gpointer data);
static void     booz2_sim_display(void);
static void     booz_sim_read_actuators(void);

static void ivy_transport_init(void);
static void on_DL_SETTING(IvyClientPtr app __attribute__ ((unused)),
			  void *user_data __attribute__ ((unused)),
			  int argc __attribute__ ((unused)), char *argv[]);
static void on_DL_BLOCK(IvyClientPtr app __attribute__ ((unused)),
			  void *user_data __attribute__ ((unused)),
			  int argc __attribute__ ((unused)), char *argv[]);
static void on_DL_MOVE_WP(IvyClientPtr app __attribute__ ((unused)),
			  void *user_data __attribute__ ((unused)),
			  int argc __attribute__ ((unused)), char *argv[]);


static void booz2_sim_init(void) {

  gettimeofday (&host_time_start, NULL);
  sim_time = 0.;
  disp_time = 0.;

  booz_flight_model_init();

  booz_sensors_model_init(sim_time);

  booz_wind_model_init();

  ivy_transport_init();

  main_init();

}

#include "booz2_analog_baro.h"
#include "imu.h"

static gboolean booz2_sim_periodic(gpointer data __attribute__ ((unused))) {

  struct timeval host_time_now;
  gettimeofday (&host_time_now, NULL);
  host_time_elapsed = host_time_factor *
    ((host_time_now.tv_sec  - host_time_start.tv_sec) +
     (host_time_now.tv_usec - host_time_start.tv_usec)*1e-6);

  while (sim_time <= host_time_elapsed) {
    sim_run_one_step();
    sim_time += SIM_DT;
  }



  return TRUE;
}


#include "subsystems/ahrs.h"

static void sim_run_one_step(void) {

   /* read actuators positions */
    booz_sim_read_actuators();

    /* run our models */
    if (sim_time > 13.)
      bfm.on_ground = FALSE;

    booz_wind_model_run(SIM_DT);

    booz_flight_model_run(SIM_DT, booz_sim_actuators_values);

    booz_sensors_model_run(sim_time);
    battery_voltage = bfm.bat_voltage * 10;

    /* outputs models state */
    booz2_sim_display();

    /* run the airborne code */

    // feed a rc frame and signal event
    BoozRcSimFeed(sim_time);
    // process it
    main_event();

    if (booz_sensors_model_baro_available()) {
      Booz2BaroISRHandler(bsm.baro);
      main_event();
#ifdef BYPASS_INS
      sim_overwrite_ins();
#endif /* BYPASS_INS */
    }
    if (booz_sensors_model_gyro_available()) {
      imu_feed_data();
      main_event();
#ifdef BYPASS_AHRS
      sim_overwrite_ahrs();
#endif /* BYPASS_AHRS */
#ifdef BYPASS_INS
      sim_overwrite_ins();
#endif /* BYPASS_INS */
    }

    if (booz_sensors_model_gps_available()) {
      sim_gps_feed_data();
      main_event();
    }

    if (booz_sensors_model_mag_available()) {
      sim_mag_feed_data();
      main_event();
#ifdef BYPASS_AHRS
      sim_overwrite_ahrs();
#endif /* BYPASS_AHRS */
    }

    main_periodic();


}





#ifdef BYPASS_AHRS
#include "booz_geometry_mixed.h"
#include "subsystems/ahrs.h"
static void sim_overwrite_ahrs(void) {
  ahrs.ltp_to_body_euler.phi   = BOOZ_ANGLE_I_OF_F(bfm.eulers->ve[AXIS_X]);
  ahrs.ltp_to_body_euler.theta = BOOZ_ANGLE_I_OF_F(bfm.eulers->ve[AXIS_Y]);
  ahrs.ltp_to_body_euler.psi   = BOOZ_ANGLE_I_OF_F(bfm.eulers->ve[AXIS_Z]);

  ahrs.ltp_to_body_quat.qi = BOOZ_INT_OF_FLOAT(bfm.quat->ve[QUAT_QI], IQUAT_RES);
  ahrs.ltp_to_body_quat.qx = BOOZ_INT_OF_FLOAT(bfm.quat->ve[QUAT_QX], IQUAT_RES);
  ahrs.ltp_to_body_quat.qy = BOOZ_INT_OF_FLOAT(bfm.quat->ve[QUAT_QY], IQUAT_RES);
  ahrs.ltp_to_body_quat.qz = BOOZ_INT_OF_FLOAT(bfm.quat->ve[QUAT_QZ], IQUAT_RES);

  ahrs.body_rate.p = BOOZ_RATE_I_OF_F(bfm.ang_rate_body->ve[AXIS_X]);
  ahrs.body_rate.q = BOOZ_RATE_I_OF_F(bfm.ang_rate_body->ve[AXIS_Y]);
  ahrs.body_rate.r = BOOZ_RATE_I_OF_F(bfm.ang_rate_body->ve[AXIS_Z]);

}
#endif /* BYPASS_AHRS */


#ifdef BYPASS_INS
#include "subsystems/ins.h"
static void sim_overwrite_ins(void) {
  ins_position.z    = BOOZ_POS_I_OF_F(bfm.pos_ltp->ve[AXIS_Z]);
  ins_speed_earth.z = BOOZ_SPEED_I_OF_F(bfm.speed_ltp->ve[AXIS_Z]);
  ins_accel_earth.z = BOOZ_ACCEL_I_OF_F(bfm.accel_ltp->ve[AXIS_Z]);
}
#endif /* BYPASS_INS */




#include "booz2_gps.h"
static void sim_gps_feed_data(void) {
  //  booz2_gps_lat = bsm.gps_pos_lla.lat;
  //  booz2_gps_lon = bsm.gps_pos_lla.lon;
  // speed?
  //  booz2_gps_vel_n = rint(bsm.gps_speed->ve[AXIS_X] * 100.);
  //  booz2_gps_vel_e = rint(bsm.gps_speed->ve[AXIS_Y] * 100.);


  booz_gps_state.ecef_pos.x = rint(bsm.gps_pos_ecef.x);
  booz_gps_state.ecef_pos.y = rint(bsm.gps_pos_ecef.y);
  booz_gps_state.ecef_pos.z = rint(bsm.gps_pos_ecef.z);
  //  VECT3_COPY(booz_gps_state.ecef_pos,   bsm.gps_pos_ecef);
  VECT3_COPY(booz_gps_state.ecef_speed, bsm.gps_speed_ecef);
  booz_gps_state.fix = BOOZ2_GPS_FIX_3D;


}

#include "AMI601.h"
static void sim_mag_feed_data(void) {
  ami601_val[IMU_MAG_X_CHAN] = bsm.mag->ve[AXIS_X];
  ami601_val[IMU_MAG_Y_CHAN] = bsm.mag->ve[AXIS_Y];
  ami601_val[IMU_MAG_Z_CHAN] = bsm.mag->ve[AXIS_Z];
  ami601_status = AMI601_DATA_AVAILABLE;
}



#define RPM_OF_RAD_S(a) ((a)*60./M_PI)
static void booz2_sim_display(void) {
  if (sim_time >= disp_time) {
    disp_time+= DT_DISPLAY;
    //    booz_flightgear_send();
    IvySendMsg("%d BOOZ_SIM_RPMS %f %f %f %f",
	       AC_ID,
	       RPM_OF_RAD_S(bfm.omega->ve[SERVO_FRONT]),
	       RPM_OF_RAD_S(bfm.omega->ve[SERVO_BACK]),
	       RPM_OF_RAD_S(bfm.omega->ve[SERVO_RIGHT]),
	       RPM_OF_RAD_S(bfm.omega->ve[SERVO_LEFT]) );
    IvySendMsg("%d BOOZ_SIM_RATE_ATTITUDE %f %f %f %f %f %f",
	       AC_ID,
	       DegOfRad(bfm.ang_rate_body->ve[AXIS_X]),
	       DegOfRad(bfm.ang_rate_body->ve[AXIS_Y]),
	       DegOfRad(bfm.ang_rate_body->ve[AXIS_Z]),
	       DegOfRad(bfm.eulers->ve[AXIS_X]),
	       DegOfRad(bfm.eulers->ve[AXIS_Y]),
	       DegOfRad(bfm.eulers->ve[AXIS_Z]));
    IvySendMsg("%d BOOZ_SIM_SPEED_POS %f %f %f %f %f %f",
	       AC_ID,
	       (bfm.speed_ltp->ve[AXIS_X]),
	       (bfm.speed_ltp->ve[AXIS_Y]),
	       (bfm.speed_ltp->ve[AXIS_Z]),
	       (bfm.pos_ltp->ve[AXIS_X]),
	       (bfm.pos_ltp->ve[AXIS_Y]),
	       (bfm.pos_ltp->ve[AXIS_Z]));
#if 0
    IvySendMsg("%d BOOZ_SIM_WIND %f %f %f",
    	       AC_ID,
	       bwm.velocity->ve[AXIS_X],
    	       bwm.velocity->ve[AXIS_Y],
    	       bwm.velocity->ve[AXIS_Z]);
#endif
    IvySendMsg("%d BOOZ_SIM_ACCEL_LTP %f %f %f",
    	       AC_ID,
	       bfm.accel_ltp->ve[AXIS_X],
    	       bfm.accel_ltp->ve[AXIS_Y],
    	       bfm.accel_ltp->ve[AXIS_Z]);

  }
}

int main ( int argc, char** argv) {

  booz2_sim_parse_options(argc, argv);

  booz2_sim_init();

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  g_timeout_add(HOST_TIMEOUT_PERIOD, booz2_sim_periodic, NULL);

  g_main_loop_run(ml);

  return 0;
}


static void ivy_transport_init(void) {
  IvyInit ("BoozSim", "BoozSim READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_BLOCK, NULL, "^(\\S*) BLOCK (\\S*) (\\S*)");
  IvyBindMsg(on_DL_MOVE_WP, NULL, "^(\\S*) MOVE_WP (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");
}

#include "std.h"
#include "generated/settings.h"
#include "dl_protocol.h"
#include "subsystems/datalink/downlink.h"
static void on_DL_SETTING(IvyClientPtr app __attribute__ ((unused)),
			  void *user_data __attribute__ ((unused)),
			  int argc __attribute__ ((unused)), char *argv[]){
  uint8_t index = atoi(argv[2]);
  float value = atof(argv[3]);
  DlSetting(index, value);
  DOWNLINK_SEND_DL_VALUE(&index, &value);
  //  printf("setting %d %f\n", index, value);
}

static void on_DL_BLOCK(IvyClientPtr app __attribute__ ((unused)),
			  void *user_data __attribute__ ((unused)),
			  int argc __attribute__ ((unused)), char *argv[]){
  int block = atoi(argv[1]);
  nav_goto_block(block);
}

#include "pprz_geodetic_int.h"
#include "stdio.h"
static void on_DL_MOVE_WP(IvyClientPtr app __attribute__ ((unused)),
			  void *user_data __attribute__ ((unused)),
			  int argc __attribute__ ((unused)), char *argv[]){
  int wp_id = atoi(argv[1]);
  //int ac_id = atoi(argv[1]);
  struct LlaCoor_i lla;
  struct EnuCoor_i enu;
  //printf("move deg %d %d %d\n",atoi(argv[3]),atoi(argv[4]),atoi(argv[5]));
  int lat = atoi(argv[3]);
  int lon = atoi(argv[4]);
  int alt = atoi(argv[5]);
  lla.lat = INT32_RAD_OF_DEG(lat);
  lla.lon = INT32_RAD_OF_DEG(lon);
  lla.alt = alt+ins_ltp_def.lla.alt;
  //printf("move rad %d %d %d (%d)\n",lla.lat,lla.lon,lla.alt,ins_ltp_def.lla.alt);
  enu_of_lla_point_i(&enu,&ins_ltp_def,&lla);
  enu.x = POS_BFP_OF_REAL(enu.x)/100;
  enu.y = POS_BFP_OF_REAL(enu.y)/100;
  enu.z = POS_BFP_OF_REAL(enu.z)/100;
  //printf("enu_of_lla %d %d %d -> %d %d %d (%f %f %f)\n",waypoints[wp_id].x,waypoints[wp_id].y,waypoints[wp_id].z,enu.x,enu.y,enu.z,
  //    POS_FLOAT_OF_BFP(enu.x),POS_FLOAT_OF_BFP(enu.y),POS_FLOAT_OF_BFP(enu.z));
  VECT3_ASSIGN(waypoints[wp_id],enu.x,enu.y,enu.z);
  DOWNLINK_SEND_WP_MOVED_LTP(&wp_id, &enu.x, &enu.y, &enu.z);
  //DOWNLINK_SEND_WP_MOVED_LLA(&wp_id, &lat, &lon, &alt);
  //printf("WP_MOVED\n");
  //fflush(stdout);
}

#include "actuators.h"
static void booz_sim_read_actuators(void) {


//  printf("actatuors %d %d %d %d\n",
//	 Actuator(SERVO_FRONT), Actuator(SERVO_BACK), Actuator(SERVO_RIGHT), Actuator(SERVO_LEFT));
  int32_t ut_front = Actuator(SERVO_FRONT) - TRIM_FRONT;
  int32_t ut_back  = Actuator(SERVO_BACK)  - TRIM_BACK;
  int32_t ut_right = Actuator(SERVO_RIGHT) - TRIM_RIGHT;
  int32_t ut_left  = Actuator(SERVO_LEFT)  - TRIM_LEFT;
#if 1
  booz_sim_actuators_values[0] = (double)ut_front / SUPERVISION_MAX_MOTOR;
  booz_sim_actuators_values[1] = (double)ut_back  / SUPERVISION_MAX_MOTOR;
  booz_sim_actuators_values[2] = (double)ut_right / SUPERVISION_MAX_MOTOR;
  booz_sim_actuators_values[3] = (double)ut_left  / SUPERVISION_MAX_MOTOR;
#else
  //  double foo = 0.33;
  double foo = 0.35;
  booz_sim_actuators_values[0] = foo;
  booz_sim_actuators_values[1] = foo;
  booz_sim_actuators_values[2] = foo;
  booz_sim_actuators_values[3] = foo;
#endif


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
