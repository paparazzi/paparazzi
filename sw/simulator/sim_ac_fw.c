/*
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

#include <string.h>
#include "sim_ac_jsbsim.h"
#include "firmwares/fixedwing/main_ap.h"
#include "firmwares/fixedwing/main_fbw.h"
#include "jsbsim_hw.h"

#include <iostream>
using namespace std;

using namespace JSBSim;

/* Datalink Ivy function */
static void on_DL_PING(IvyClientPtr app __attribute__ ((unused)),
			  void *user_data __attribute__ ((unused)),
			  int argc __attribute__ ((unused)), char *argv[]){
  parse_dl_ping(argv);
}
static void on_DL_ACINFO(IvyClientPtr app __attribute__ ((unused)),
			  void *user_data __attribute__ ((unused)),
			  int argc __attribute__ ((unused)), char *argv[]){
  parse_dl_acinfo(argv);
}
static void on_DL_SETTING(IvyClientPtr app __attribute__ ((unused)),
			  void *user_data __attribute__ ((unused)),
			  int argc __attribute__ ((unused)), char *argv[]){
  parse_dl_setting(argv);
}
static void on_DL_GET_SETTING(IvyClientPtr app __attribute__ ((unused)),
			  void *user_data __attribute__ ((unused)),
			  int argc __attribute__ ((unused)), char *argv[]){
  parse_dl_get_setting(argv);
}
static void on_DL_BLOCK(IvyClientPtr app __attribute__ ((unused)),
			  void *user_data __attribute__ ((unused)),
			  int argc __attribute__ ((unused)), char *argv[]){
  parse_dl_block(argv);
}
static void on_DL_MOVE_WP(IvyClientPtr app __attribute__ ((unused)),
			  void *user_data __attribute__ ((unused)),
			  int argc __attribute__ ((unused)), char *argv[]){
  parse_dl_move_wp(argv);
}

void sim_autopilot_init(void) {
  IvyBindMsg(on_DL_PING, NULL, "^(\\S*) DL_PING");
  IvyBindMsg(on_DL_ACINFO, NULL, "^(\\S*) DL_ACINFO (\\S*) (\\S*) (\\S* (\\S*) (\\S*) (\\S*)) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_GET_SETTING, NULL, "^(\\S*) GET_DL_SETTING (\\S*) (\\S*)");
  IvyBindMsg(on_DL_BLOCK, NULL, "^(\\S*) BLOCK (\\S*) (\\S*)");
  IvyBindMsg(on_DL_MOVE_WP, NULL, "^(\\S*) MOVE_WP (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");

  init_fbw();
  init_ap();
}

void autopilot_periodic_task(void) {
  handle_periodic_tasks_ap();
  handle_periodic_tasks_fbw();
}

void autopilot_event_task(void) {
  event_task_ap();
  event_task_fbw();
}

void print(FGFDMExec* FDMExec) {
  FGPropertyManager* cur_node;
  double cur_value, factor=1;
  const char* state[] = {"simulation/sim-time-sec",
			 /*"position/lat-gc-deg","position/long-gc-deg","position/h-sl-meters",
			 "ic/lat-gc-deg","ic/long-gc-deg","ic/h-sl-ft",
			 "velocities/v-north-fps","velocities/v-east-fps","velocities/v-down-fps","velocities/vg-fps",*/
			 /*"attitude/roll-rad","attitude/pitch-rad","attitude/heading-true-rad",
			 "velocities/p-rad_sec","velocities/q-rad_sec","velocities/r-rad_sec",*/
			 "fcs/elevator-pos-deg","fcs/elevator-pos-rad","fcs/elevator-cmd-norm"};
			 //"fcs/throttle-cmd-norm","fcs/aileron-cmd-norm","fcs/elevator-cmd-norm"};
  int i=0;

  cur_node = FDMExec->GetPropertyManager()->GetNode("simulation/sim-time-sec");
  cur_value = cur_node->getDoubleValue();
  cout << state[i] << " " << cur_value << endl;

  for (i=1; i<3+1; i++) {
    if (strstr(state[i],"rad_")!=NULL) factor=RAD2DEG;
    if (strstr(state[i],"fps")!=NULL || strstr(state[i],"ft")!=NULL) factor=FT2M;
    cur_node = FDMExec->GetPropertyManager()->GetNode(state[i]);
    cur_value = factor*(cur_node->getDoubleValue());
    cout << string(state[i]) << " " << cur_value << endl;
    factor = 1;
  }
}

static inline void set_value(FGFDMExec* FDMExec, string name, double value) {
  FDMExec->GetPropertyManager()->GetNode(name)->setDoubleValue(value);
}

static inline double normalize_from_pprz(int command) {
  double cmd_norm = (double)command / MAX_PPRZ;
  BoundAbs(cmd_norm, 1.0);
  return cmd_norm;
}

void copy_inputs_to_jsbsim(FGFDMExec* FDMExec) {
  static double throttle_slewed = 0.;
  static double th = 0.;
#ifndef JSBSIM_LAUNCHSPEED
#define JSBSIM_LAUNCHSPEED 20.0 //launch speed in m/s aligned with airframe body forward
#endif
  if (run_model) th += 0.01;
  if (th >= 1) th = 1;
  // detect launch
  if (!run_model && launch && !kill_throttle) {
    run_model = true;
    //set_value(FDMExec, "propulsion/set-running", 1);
    // set initial speed
    //FDMExec->GetIC()->SetAltitudeAGLFtIC(5.0 / FT2M);
    //FDMExec->GetIC()->SetVgroundFpsIC(20./FT2M);
    FDMExec->GetIC()->SetUBodyFpsIC( JSBSIM_LAUNCHSPEED / FT2M);
    FDMExec->RunIC();
    th = 0.;
  }

  double diff_throttle = normalize_from_pprz(commands[COMMAND_THROTTLE]) - throttle_slewed;
  BoundAbs(diff_throttle, 0.01);
  throttle_slewed += diff_throttle;

  set_value(FDMExec, "fcs/throttle-cmd-norm", throttle_slewed);
  //set_value(FDMExec, "fcs/throttle-cmd-norm", normalize_from_pprz(commands[COMMAND_THROTTLE]));
  //set_value(FDMExec, "fcs/throttle-cmd-norm", th);
  set_value(FDMExec, "fcs/aileron-cmd-norm",  normalize_from_pprz(commands[COMMAND_ROLL]));
  set_value(FDMExec, "fcs/elevator-cmd-norm", -normalize_from_pprz(commands[COMMAND_PITCH]));
  //set_value(FDMExec, "fcs/elevator-cmd-norm", -5);
  //set_value(FDMExec, "fcs/elevator-pos-rad", 0.4);
#ifdef COMMAND_YAW
  set_value(FDMExec, "fcs/rudder-cmd-norm",   normalize_from_pprz(commands[COMMAND_YAW]));
#endif

}

static inline double get_value(FGFDMExec* FDMExec, string name) {
  return FDMExec->GetPropertyManager()->GetNode(name)->getDoubleValue();
}

#define GPS_PERIOD (1./4.)

void copy_outputs_from_jsbsim(FGFDMExec* FDMExec) {
  static double gps_period = 0.;

  // copy GPS pos
  gps_period += DT;
  if (gps_period > GPS_PERIOD) {
    double lat    = get_value(FDMExec, "position/lat-gc-rad");
    double lon    = get_value(FDMExec, "position/long-gc-rad");
    double alt    = get_value(FDMExec, "position/h-sl-meters");
    double course = get_value(FDMExec, "attitude/heading-true-rad");
    double gspeed = get_value(FDMExec, "velocities/vg-fps") * FT2M;
    double climb  = get_value(FDMExec, "velocities/v-down-fps") * (-FT2M);
    double time   = get_value(FDMExec, "simulation/sim-time-sec");
    sim_use_gps_pos(lat, lon, alt, course, gspeed, climb, time);
    sim_update_sv();
    gps_period = 0.;
  }

  //print(FDMExec);

  double roll   = get_value(FDMExec, "attitude/roll-rad");
  double pitch  = get_value(FDMExec, "attitude/pitch-rad");
  double yaw    = get_value(FDMExec, "attitude/heading-true-rad");
  double p      = get_value(FDMExec, "velocities/p-rad_sec");
  double q      = get_value(FDMExec, "velocities/q-rad_sec");
  double r      = get_value(FDMExec, "velocities/r-rad_sec");

  // copy to AHRS
  provide_attitude_and_rates(roll, pitch, yaw, p, q, r);

  // copy IR
  set_ir(roll, pitch);

  // copy Bat level
  double bat    = 12.; // get_value(FDMExec, "propulsion/total-fuel-lbs");
  update_bat(bat);

}

