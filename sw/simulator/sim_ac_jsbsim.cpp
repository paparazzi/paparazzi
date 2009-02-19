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

#include "sim_ac_jsbsim.hpp"

#include <glib.h>
#include <getopt.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
GLOBAL DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

string RootDir = "";
string ScriptName;
string AircraftName;
string LogOutputName;
JSBSim::FGFDMExec* FDMExec;

/* 60Hz <-> 17ms */
#define TIMEOUT_PERIOD 17
#define DT (TIMEOUT_PERIOD*1e-3)
double sim_time;

#define DT_DISPLAY 0.04
double disp_time;

static void     sim_parse_options(int argc, char** argv);
static void     sim_init(void);
static gboolean sim_periodic(gpointer data);
//static void     sim_display(void);

static void ivy_transport_init(void);
static void on_DL_SETTING(IvyClientPtr app __attribute__ ((unused)), 
    void *user_data __attribute__ ((unused)), 
    int argc __attribute__ ((unused)), char *argv[]);


static void sim_init(void) {

  sim_time = 0.;
  disp_time = 0.;

  // *** SET UP JSBSIM *** //
  FDMExec = new JSBSim::FGFDMExec();
  FDMExec->SetAircraftPath(RootDir + "aircraft");
  FDMExec->SetEnginePath(RootDir + "engine");
  FDMExec->SetSystemsPath(RootDir + "systems");
  //FDMExec->GetPropertyManager()->Tie("simulation/frame_start_time", &actual_elapsed_time);
  //FDMExec->GetPropertyManager()->Tie("simulation/cycle_duration", &cycle_duration);

  if (!AircraftName.empty()) {

    FDMExec->SetDebugLevel(0); // No DEBUG messages

    if ( ! FDMExec->LoadModel( RootDir + "aircraft",
                               RootDir + "engine",
                               RootDir + "systems",
                               AircraftName)) {
      cerr << "  JSBSim could not be started" << endl << endl;
      delete FDMExec;
      exit(-1);
    }

    // Initial conditions (from flight_plan.h and aircraft.h ???)
    JSBSim::FGInitialCondition *IC = FDMExec->GetIC();
    //if ( ! IC->Load(ResetName)) {
    //  delete FDMExec;
    //  cerr << "Initialization unsuccessful" << endl;
    //  exit(-1);
    //}

  } else {
    cerr << "  No Aircraft given" << endl << endl;
    delete FDMExec;
    exit(-1);
  }

  // init sensors ? or discribe them in jSBSim

  ivy_transport_init();

  // main AP init (feed the sensors once before ?)
  init_autopilot();

}


static gboolean sim_periodic(gpointer data __attribute__ ((unused))) {

  /* read actuators positions and feed JSBSim inputs */
  copy_inputs_to_jsbsim(FDMExec);

  /* run JSBSim flight model */
  FDMExec->Run();

  sim_time += DT;

  /* read outputs from model state (and display ?) */
  copy_outputs_from_jsbsim(FDMExec);
  sim_display();

  /* run the airborne code */
  
  // feed a rc frame and signal event

  // process it
  autopilot_event_task();

  autopilot_periodic_task();

  return TRUE;
}


//static void sim_display(void) {
//}

int main ( int argc, char** argv) {

  ScriptName = "";
  AircraftName = "";
  LogOutputName = "";
  bool result = false;

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

void print_help(char** argv) {
  cout << "Usage: " << argv[0] << " [options]" << endl;
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
      PrintHelp();
      exit(0);
    } else if (argument == "-a") {

    } else if (argument == "-b") {

    } else if (argument == "-fg") {

    } else {
      cerr << "Unknown argument" << endl;
      PrintHelp();
      exit(0);
    }
  }

}
