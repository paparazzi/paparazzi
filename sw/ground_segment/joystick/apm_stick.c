/*
 * apm_stick
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

#define CAM_TILT_MIN    1000
#define CAM_TILT_MAX    2000

/* Global vars */
int fp_received_once = 0;
long int fp_east_sp,fp_north_sp,fp_up_sp,fp_psi_sp;
float east_sp,north_sp,up_sp,psi_sp;
float cam_tilt_sp = 1500.;
int last_button = 0;

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


#define STICK_DEADBAND 10
#define STICK_APPLY_DEADBAND(_v) (abs(_v) >= STICK_DEADBAND ? _v : 0)

#define ATTITUDE_COEF 2144  // RadOfDeg(15 / 128) << 20
#define CLIMB_COEF 0.5      // vertical speed (m/s)
#define YAW_RATE_COEF (2./128) // 2 deg/s
#define CAM_TILT_RATE 50    // cam rate

#define SPEED_RES 19
#define ANGLE_REF_RES 20

#define bit_is_set(x, b) ((x >> b) & 0x1)

// ATTITUDE
static gboolean joystick_attitude_periodic(gpointer data __attribute__ ((unused))) {

	stick_read();

	int roll     = STICK_APPLY_DEADBAND(stick_axis_values[0]);
	int pitch    = STICK_APPLY_DEADBAND(stick_axis_values[1]);

  int yaw_d_l  = bit_is_set(stick_button_values,6);
  int yaw_d_r  = bit_is_set(stick_button_values,7);
  int up       = bit_is_set(stick_button_values,0);
  int down     = bit_is_set(stick_button_values,3);
  int cam_up   = bit_is_set(stick_button_values,2);
  int cam_down = bit_is_set(stick_button_values,5);
  /*printf("button %d %d %d %d %d %d\n", yaw_d_l, yaw_d_r, up, down, cam_up, cam_down);*/

  float yaw_rate = 0.;
  if (yaw_d_l != yaw_d_r) {
    if (yaw_d_r) yaw_rate = YAW_RATE_COEF * (2^ANGLE_REF_RES);
    else yaw_rate = - YAW_RATE_COEF * (2^ANGLE_REF_RES);
  }

  float climb   = 0.;
  if (up != down) {
    if (up) climb = - CLIMB_COEF; // Z DOWN !!!
    else climb = CLIMB_COEF;
  }

  if (cam_up != cam_down) {
    if (cam_up) cam_tilt_sp += CAM_TILT_RATE;
    else cam_tilt_sp -= CAM_TILT_RATE;
  }
  if (cam_tilt_sp < CAM_TILT_MIN) cam_tilt_sp = CAM_TILT_MIN;
  if (cam_tilt_sp > CAM_TILT_MAX) cam_tilt_sp = CAM_TILT_MAX;

  IvySendMsg("dl BOOZ2_FMS_COMMAND %d %d %d %d %d %d %d",
      2, 3,
      (int)(climb * (1 << SPEED_RES)),
      (ATTITUDE_COEF * roll),
      (ATTITUDE_COEF * pitch),
      (int)(yaw_rate * YAW_RATE_COEF * (1 << ANGLE_REF_RES)),
      aircraft_id);
  /*printf("dl BOOZ2_FMS_COMMAND %d %d %d %d %d %d %d\n",
      2, 3,
      (int)(climb * (1 << SPEED_RES)),
      (ATTITUDE_COEF * roll),
      (ATTITUDE_COEF * pitch),
      (int)(yaw_rate * YAW_RATE_COEF * (1 << ANGLE_REF_RES)),
      aircraft_id);*/
  if (cam_up != cam_down) {
    IvySendMsg("dl DL_SETTING %d %d %f",
        aircraft_id,
        1, //CAM_TILT FIXME
        cam_tilt_sp);
    /*printf("dl DL_SETTING %d %d %f\n",
        aircraft_id,
        1, //CAM_TILT FIXME
        cam_tilt_sp);*/
  }

	return 1;
}

/*
void readBOOZ2_FPIvyBus(IvyClientPtr app, void *data, int argc, char **argv) {

  //printf("%s\n",argv[0]);
  if (argc > 0) {
    sscanf(argv[0],"%*d %*s
        %*d %*d %*d
        %*d %*d %*d
        %*d %*d %*d
        %ld %ld %ld %ld
        %*d",
        &fp_east_sp, &fp_north_sp, &fp_up_sp, &fp_psi_sp);

    fp_received_once = 1;
  }
}
*/

int main ( int argc, char** argv) {

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  parse_args(argc, argv);
  
  IvyInit ("IvyCtrlJoystick", "IvyCtrlJoystick READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  if (stick_init(device_name) != 0) return 0;

  /*
  char bindMsgBOOZ2_FP[32];
  snprintf(bindMsgBOOZ2_FP,32,"%s%d%s","(",aircraft_id," BOOZ2_FP .*)");
  IvyBindMsg(readBOOZ2_FPIvyBus,0,bindMsgBOOZ2_FP);
  */
 
  g_timeout_add(TIMEOUT_PERIOD, joystick_attitude_periodic, NULL);

  g_main_loop_run(ml);

  return 0;
}
