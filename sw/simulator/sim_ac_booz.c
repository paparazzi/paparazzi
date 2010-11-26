/*
 * $Id: sim_ac_fw.c 3499 2009-06-16 17:38:56Z gov $
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
#include "firmwares/fixedwing/main_ap.h"
#include "firmwares/fixedwing/main_fbw.h"

using namespace JSBSim;

//static void sim_gps_feed_data(void);
//static void sim_ir_feed_data(void);


void airborne_run_one_step(void) {
  // SEE  sim_run_one_step
}

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

void copy_inputs_to_jsbsim(FGFDMExec* FDMExec) {

  FGPropertyManager* cur_node;
  double cur_value;
  char buf[64];
  const char* state[] = {"front_motor",
			 "back_motor",
			 "right_motor",
			 "left_motor"};

  for (int i=0; i<4; i++) {
    sprintf(buf,"fcs/%s",state[i]);
    FDMExec->GetPropertyManager()->SetDouble(buf,0.5);
    cur_node = FDMExec->GetPropertyManager()->GetNode(buf);
    cur_value = cur_node->getDoubleValue();
    cout << state[i] << " " << cur_value << endl;
  }
}

void copy_outputs_from_jsbsim(FGFDMExec* FDMExec) {

  FGPropertyManager* cur_node;
  double cur_value, factor=1;
  char buf[64];
  const char* state[] = {"sim-time-sec",
			 "lat-gc-deg","long-gc-deg","h-sl-ft",
			 "u-fps","v-fps","w-fps",
			 "theta-deg","phi-deg","psi-true-deg",
			 "p-rad_sec","q-rad_sec","r-rad_sec"};
  int i=0;

  cur_node = FDMExec->GetPropertyManager()->GetNode("sim-time-sec");
  cur_value = cur_node->getDoubleValue();
  cout << state[i] << cur_value << endl;

  for (i=1; i<12+1; i++) {
    sprintf(buf,"ic/%s",state[i]);
    if (strstr(state[i],"rad_")!=NULL) factor=RAD2DEG;
    if (strstr(state[i],"fps")!=NULL || strstr(state[i],"ft")!=NULL) factor=FT2M;
    cur_node = FDMExec->GetPropertyManager()->GetNode(buf);
    cur_value = factor*(cur_node->getDoubleValue());
    cout << state[i] << " " << cur_value << endl;
    factor = 1;
  }

}
