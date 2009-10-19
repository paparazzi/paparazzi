/*
 * Attack3 Logitech
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
#define UPDATE_FREQ     (1./TIMEOUT_PERIOD)

#define DEFAULT_AC_ID   1
#define DEFAULT_MODE    0

#define CAM_TILT_MIN    1000
#define CAM_TILT_MAX    2000

/* Global vars */
int fp_received_once = 0;
float cam_tilt_sp = 1500.;
int last_button = 0;

/* Options */
char * device_name    = NULL;
int aircraft_id       = DEFAULT_AC_ID;
int mode              = DEFAULT_MODE;

void parse_args(int argc, char * argv[])
{
  int i;

  for (i = 1; i < argc; i++) {
    if      (!strcmp(argv[i],"-d") && i<argc-1) device_name = argv[++i];
    else if (!strcmp(argv[i],"-a") && i<argc-1) aircraft_id = atoi(argv[++i]);
    else if (!strcmp(argv[i],"-m") && i<argc-1) mode = atoi(argv[++i]);
    else if (!strcmp(argv[i],"-h")) goto l_help;
  }
  return;

l_help:
  printf("Usage:\n");
  printf("  %s <option> [<option>...]\n",argv[0]);
  printf("Options:\n");
  printf("  -d <string>  device name\n");
  printf("  -a <int>     aircraft id (default: %d)\n",DEFAULT_AC_ID);
  printf("  -m <int>     mode (0 attitude, 1 speed, default: %d)\n",DEFAULT_MODE);
  printf("  -h           display this help\n");
  exit(1);
}


#define STICK_DEADBAND 15
#define STICK_APPLY_DEADBAND(_v) (abs(_v) >= STICK_DEADBAND ? _v : 0)
#define STICK_LARGE_DEADBAND 20
#define STICK_APPLY_LARGE_DEADBAND(_v) (abs(_v) >= STICK_LARGE_DEADBAND ? _v : 0)

#define ATTITUDE_COEF 2144  // RadOfDeg(15 / 128) << 20
#define CLIMB_COEF 0.5      // vertical speed (m/s)
#define YAW_RATE_COEF (6./512) // 6 deg/s

#define SPEED_RES 19
#define ANGLE_REF_RES 20

#define bit_is_set(x, b) ((x >> b) & 0x1)

// ATTITUDE
static gboolean joystick_attitude_periodic(gpointer data __attribute__ ((unused))) {

	stick_read();

	int roll     = STICK_APPLY_DEADBAND(stick_axis_values[0]);
	int pitch    = STICK_APPLY_DEADBAND(stick_axis_values[1]);
  int tilt     = stick_axis_values[2];

  int right    = bit_is_set(stick_button_values,4);
  int left     = bit_is_set(stick_button_values,3);
  int up       = bit_is_set(stick_button_values,2);
  int down     = bit_is_set(stick_button_values,1);

  float yaw_rate = 0.;
  if (left != right) {
    if (right) yaw_rate = 1.;
    else yaw_rate = -1.;
  }

  float climb = 0.;
  if (up != down) {
    if (up) climb = - CLIMB_COEF; // Z DOWN !!!
    else climb = CLIMB_COEF;
  }

  static int last_tilt = 0;
  cam_tilt_sp = CAM_TILT_MIN + (CAM_TILT_MAX - CAM_TILT_MIN) * (tilt + 127.)/254.;
  if (cam_tilt_sp < CAM_TILT_MIN) cam_tilt_sp = CAM_TILT_MIN;
  if (cam_tilt_sp > CAM_TILT_MAX) cam_tilt_sp = CAM_TILT_MAX;

  IvySendMsg("dl BOOZ2_FMS_COMMAND %d %d %d %d %d %d %d",
      2, 3,
      (int)(climb * (1 << SPEED_RES)),
      (ATTITUDE_COEF * roll),
      (ATTITUDE_COEF * pitch),
      (int)(yaw_rate * YAW_RATE_COEF * (1 << ANGLE_REF_RES)),
      aircraft_id);
//  printf("dl BOOZ2_FMS_COMMAND %d %d %d %d %d %d %d\n",
//      2, 3,
//      (int)(climb * (1 << SPEED_RES)),
//      (ATTITUDE_COEF * roll),
//      (ATTITUDE_COEF * pitch),
//      (int)(yaw_rate * YAW_RATE_COEF * (1 << ANGLE_REF_RES)),
//      aircraft_id);
  if (tilt != last_tilt) {
    IvySendMsg("dl DL_SETTING %d %d %f",
        aircraft_id,
        1, //CAM_TILT FIXME
        cam_tilt_sp);
//    printf("dl DL_SETTING %d %d %f\n",
//        aircraft_id,
//        1, //CAM_TILT FIXME
//        cam_tilt_sp);
  }
  last_tilt = tilt;

	return 1;
}

#define SPEED_MAX_VAL 127
// SPEED
static gboolean joystick_speed_periodic(gpointer data __attribute__ ((unused))) {

	stick_read();

	int vx = -STICK_APPLY_DEADBAND(stick_axis_values[1]); // vx > 0 front
	int vy = STICK_APPLY_DEADBAND(stick_axis_values[0]); // vy > 0 right
  int tilt = stick_axis_values[2];

  int right = bit_is_set(stick_button_values,4);
  int left  = bit_is_set(stick_button_values,3);
  int up    = bit_is_set(stick_button_values,2);
  int down  = bit_is_set(stick_button_values,1);

  int yaw_rate = 0; // rate > 0 clockwise
  if (left != right) {
    if (right) yaw_rate = SPEED_MAX_VAL;
    else yaw_rate = -SPEED_MAX_VAL;
  }

  int climb = 0; // climb > 0 up
  if (up != down) {
    if (up) climb = SPEED_MAX_VAL;
    else climb = -SPEED_MAX_VAL;
  }

  static int last_tilt = 0;
  cam_tilt_sp = CAM_TILT_MIN + (CAM_TILT_MAX - CAM_TILT_MIN) * (tilt + 127.)/254.;
  if (cam_tilt_sp < CAM_TILT_MIN) cam_tilt_sp = CAM_TILT_MIN;
  if (cam_tilt_sp > CAM_TILT_MAX) cam_tilt_sp = CAM_TILT_MAX;

  // NAV h = 4, v = 5
  IvySendMsg("dl BOOZ2_FMS_COMMAND %d %d %d %d %d %d %d", 4, 5, climb, vx, vy, yaw_rate, aircraft_id);
  //printf("dl BOOZ2_FMS_COMMAND %d %d %d %d %d %d %d\n", 4, 5, climb, vx, vy, yaw_rate, aircraft_id);
  if (tilt != last_tilt) {
    IvySendMsg("dl DL_SETTING %d %d %f",
        aircraft_id,
        1, //CAM_TILT FIXME
        cam_tilt_sp);
//    printf("dl DL_SETTING %d %d %f\n",
//        aircraft_id,
//        1, //CAM_TILT FIXME
//        cam_tilt_sp);
  }
  last_tilt = tilt;

	return 1;
}

int main ( int argc, char** argv) {

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  parse_args(argc, argv);
  
  IvyInit ("IvyCtrlJoystick", "IvyCtrlJoystick READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  if (stick_init(device_name) != 0) return 0;

  if (mode == 0) 
    g_timeout_add(TIMEOUT_PERIOD, joystick_attitude_periodic, NULL);
  else if (mode == 1)
    g_timeout_add(TIMEOUT_PERIOD, joystick_speed_periodic, NULL);
  else {
    fprintf(stderr,"Unknown mode : %d\n",mode);
    exit(1);
  }


  g_main_loop_run(ml);

  return 0;
}
