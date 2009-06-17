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

#include "sim_ac_jsbsim.h"

#include <glib.h>
#include <getopt.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
GLOBAL DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

string RootDir = "";
string ICName;
string AircraftName;
string LogOutputName;
JSBSim::FGFDMExec* FDMExec;

static void     sim_parse_options(int argc, char** argv);
static void     sim_init(void);
static gboolean sim_periodic(gpointer data);

static void ivy_transport_init(void);
//static void on_DL_SETTING(IvyClientPtr app __attribute__ ((unused)), 
//    void *user_data __attribute__ ((unused)), 
//    int argc __attribute__ ((unused)), char *argv[]);


static void sim_init(void) {

  jsbsim_init();

  // init sensors ? or discribe them in jSBSim

  ivy_transport_init();

  // main AP init (feed the sensors once before ?)
  autopilot_init();

}

static gboolean sim_periodic(gpointer data __attribute__ ((unused))) {

  /* read actuators positions and feed JSBSim inputs */
  copy_inputs_to_jsbsim(FDMExec);

  /* run JSBSim flight model */
  bool result = FDMExec->Run();

  //sim_time += DT;

  /* check if still flying */
  result = check_crash_jsbsim(FDMExec);

  /* read outputs from model state (and display ?) */
  copy_outputs_from_jsbsim(FDMExec);

  /* run the airborne code */
  
  airborne_run_one_step();

  return result;
}


int main ( int argc, char** argv) {

  ICName = "reset00";
  AircraftName = "quad";
  LogOutputName = "out.csv";

  //sim_parse_options(argc, argv);

  sim_init();

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  g_timeout_add(JSBSIM_PERIOD, sim_periodic, NULL);

  g_main_loop_run(ml);

  //delete FDMExec;
  return 0;
}


static void ivy_transport_init(void) {
  IvyInit ("Paparazzi sim " + AC_ID, "READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");
}

void print_help() {
  //cout << "Usage: " << argv[0] << " [options]" << endl;
  cout << "Usage: simsitl [options]" << endl;
  cout << " Options :" << endl;
  cout << "   -a <aircraft name>" << endl;
  cout << "   -b <Ivy bus>\tdefault is 127.255.255.255:2010" << endl;
  cout << "   -fg <flight gear client address>" << endl;
  cout << "   -h --help show this help" << endl;
}

static void sim_parse_options(int argc, char** argv) {

  if (argc == 1) {
    print_help();
    exit(0);
  }

  int i;

  for (i = 1; i < argc; ++i) {
    string argument = string(argv[i]);

    if (argument == "--help" || argument == "-h") {
      print_help();
      exit(0);
    } else if (argument == "-a") {

    } else if (argument == "-b") {

    } else if (argument == "-fg") {

    } else {
      cerr << "Unknown argument" << endl;
      print_help();
      exit(0);
    }
  }

}

void jsbsim_init(void) {

  JSBSim::FGState* State;
  bool result;

  FDMExec = new JSBSim::FGFDMExec();

  State = FDMExec->GetState();
  State->Setsim_time(0.);
  State->Setdt(DT);

  cout << "Simulation elapsed time: " << FDMExec->GetSimTime() << endl;
  cout << "Simulation delta " << FDMExec->GetDeltaT() << endl;

#ifdef JSBSIM_ROOT_DIR
  RootDir = JSBSIM_ROOT_DIR;
#endif

  if (!AircraftName.empty()) {
    
    FDMExec->DisableOutput();
    FDMExec->SetDebugLevel(0); // No DEBUG messages
    
    if ( ! FDMExec->LoadModel( RootDir + "aircraft",
                               RootDir + "engine",
                               RootDir + "systems",
                               AircraftName)){
      cerr << "  JSBSim could not be started" << endl << endl;
      delete FDMExec;
      exit(-1);
    }

    JSBSim::FGInitialCondition *IC = FDMExec->GetIC();
    if ( ! IC->Load(ICName)) {
      delete FDMExec;
      cerr << "Initialization unsuccessful" << endl;
      exit(-1);
    }

  } else {
    cerr << "  No Aircraft given" << endl << endl;
    delete FDMExec;
    exit(-1);
  }

  result = FDMExec->Run();
  if (result) cout << "Made Initial Run" << endl;

}

bool check_crash_jsbsim(JSBSim::FGFDMExec* FDMExec) {

  JSBSim::FGPropertyManager* cur_node;
  double cur_value;
  
  cur_node = FDMExec->GetPropertyManager()->GetNode("ic/h-agl-ft");
  cur_value = cur_node->getDoubleValue();

  if (cur_value>0) return true;
  else return false;
}
