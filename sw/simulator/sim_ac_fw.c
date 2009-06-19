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

#include <string.h>
#include "sim_ac_jsbsim.h"
#include "main_ap.h"
#include "main_fbw.h"
#include "jsbsim_hw.h"

using namespace JSBSim;

void autopilot_init(void) {
  init_fbw();
  init_ap();
}

void autopilot_periodic_task(void) {
  periodic_task_ap();
  periodic_task_fbw();
}

void autopilot_event_task(void) {
  event_task_ap();
  event_task_fbw();
}

void print(FGFDMExec* FDMExec) {
  FGPropertyManager* cur_node;
  double cur_value, factor=1;
  const char* state[] = {"sim-time-sec",
			 "position/lat-gc-deg","position/long-gc-deg","position/h-sl-meters",/*
			 "ic/lat-gc-deg","ic/long-gc-deg","ic/h-sl-ft",
			 "velocities/v-north-fps","velocities/v-east-fps","velocities/v-down-fps","velocities/vg-fps",
			 "attitude/roll-rad","attitude/pitch-rad","attitude/heading-true-rad",
			 "velocities/p-rad_sec","velocities/q-rad_sec","velocities/r-rad_sec",*/
			 "forces/fbx-gear-lbs","forces/fby-gear-lbs","forces/fbz-gear-lbs"};
  int i=0;

  cur_node = FDMExec->GetPropertyManager()->GetNode("sim-time-sec");
  cur_value = cur_node->getDoubleValue();
  cout << state[i] << " " << cur_value << endl;
  
  for (i=1; i<6+1; i++) {
    if (strstr(state[i],"rad_")!=NULL) factor=RAD2DEG;
    if (strstr(state[i],"fps")!=NULL || strstr(state[i],"ft")!=NULL) factor=FT2M;
    cur_node = FDMExec->GetPropertyManager()->GetNode(state[i]);
    cur_value = factor*(cur_node->getDoubleValue());
    cout << string(state[i]) << " " << cur_value << endl;
    factor = 1;
  }
}

void copy_inputs_to_jsbsim(FGFDMExec* FDMExec) {


}

double get_value(FGFDMExec* FDMExec, string name) {
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
    double time   = get_value(FDMExec, "sim-time-sec");
    sim_use_gps_pos(lat, lon, alt, course, gspeed, climb, time);
    sim_update_sv();
    gps_period = 0.;
  }

    print(FDMExec);
  // copy IR
  double roll   = get_value(FDMExec, "attitude/roll-rad");
  double pitch  = get_value(FDMExec, "attitude/pitch-rad");
  set_ir(roll, pitch);

  // copy Bat level
  double bat    = 12.; // get_value(FDMExec, "propulsion/total-fuel-lbs");
  update_bat(bat);

}

