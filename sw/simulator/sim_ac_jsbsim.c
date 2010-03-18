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

#include <stdlib.h>
#include <stdio.h>
#include <glib.h>
#include <getopt.h>

#include <iostream>
using namespace std;

//#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
GLOBAL DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

bool run_model;

string ICName;
string AircraftName;
JSBSim::FGFDMExec* FDMExec;

static void     sim_parse_options(int argc, char** argv);
static void     sim_init(void);
static gboolean sim_periodic(gpointer data);

string ivyBus = "127.255.255.255";
static void ivy_transport_init(void);


static void sim_init(void) {

  run_model = false;

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
  bool result = true;
  if (run_model) {
    result = FDMExec->Run();
  }
  /* check if still flying */
  //result = check_crash_jsbsim(FDMExec);

  /* read outputs from model state (and display ?) */
  copy_outputs_from_jsbsim(FDMExec);

  /* run the airborne code */
  
//  airborne_run_one_step();
  autopilot_event_task();                                                       
  autopilot_periodic_task();                                                    

  return result;
}


int main ( int argc, char** argv) {

  sim_parse_options(argc, argv);

  sim_init();

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  g_timeout_add(JSBSIM_PERIOD, sim_periodic, NULL);

  g_main_loop_run(ml);

  delete FDMExec;
  return 0;
}


static void ivy_transport_init(void) {
  IvyInit ("Paparazzi sim " + AC_ID, "READY", NULL, NULL, NULL, NULL);
  IvyStart(ivyBus.c_str());
}

void print_help() {
  cout << "Usage: simsitl [options]" << endl;
  cout << " Options :" << endl;
  cout << "   -a <aircraft name>\tUnused, only for compatibility" << endl;
  cout << "   -b <Ivy bus>\tdefault is 127.255.255.255:2010" << endl;
  cout << "   -fg <flight gear client address>" << endl;
  cout << "   -h --help show this help" << endl;
}

static void sim_parse_options(int argc, char** argv) {

  if (argc == 1) return;

  int i;
  for (i = 1; i < argc; ++i) {
    string argument = string(argv[i]);

    if (argument == "--help" || argument == "-h") {
      print_help();
      exit(0);
    }
    else if (argument == "-a") {
      // Compatibility with ocaml
      i++;
    }
    else if (argument == "-boot") {
      // Compatibility with ocaml
    }
    else if (argument == "-norc") {
      // Compatibility with ocaml
    }
    else if (argument == "-b") {
      ivyBus = string(argv[++i]);
    }
    else if (argument == "-fg") {
      // TODO
      i++;
    }
    else {
      cerr << "Unknown argument" << endl;
      print_help();
      exit(0);
    }
  }

}

void jsbsim_init(void) {

  // *** SET UP JSBSIM *** //

  char* root = getenv("PAPARAZZI_HOME");
  if (root == NULL) {
    cerr << "PAPARAZZI_HOME is not defined" << endl;
    exit(0);
  }
  string pprzRoot = string(root);
  
#ifdef JSBSIM_MODEL
  AircraftName = JSBSIM_MODEL;
#endif
#ifdef JSBSIM_INIT
  ICName = JSBSIM_INIT;
#endif

  FDMExec = new JSBSim::FGFDMExec();

  /* Set simulation time step */
  FDMExec->Setsim_time(0.);
  FDMExec->Setdt(DT);
  cout << "Simulation delta " << FDMExec->GetDeltaT() << endl;

  FDMExec->DisableOutput();
  FDMExec->SetDebugLevel(0); // No DEBUG messages

  if (!AircraftName.empty()) {

    if ( ! FDMExec->LoadModel( pprzRoot + "/conf/simulator",
                               pprzRoot + "/conf/simulator",
                               pprzRoot + "/conf/simulator",
                               AircraftName)){
      cerr << "  JSBSim could not be started" << endl << endl;
      delete FDMExec;
      exit(-1);
    }

    JSBSim::FGInitialCondition *IC = FDMExec->GetIC();
    if(!ICName.empty()) {
      if (!IC->Load(ICName)) {
        delete FDMExec;
        cerr << "Initialization from file unsuccessful" << endl;
        exit(-1);
      }
    }
    else {
      // Use flight plan initial conditions
      IC->SetLatitudeDegIC(NAV_LAT0 / 1e7);
      IC->SetLongitudeDegIC(NAV_LON0 / 1e7);
      IC->SetAltitudeAGLFtIC(0.0 / FT2M);
      IC->SetTerrainElevationFtIC(GROUND_ALT / FT2M);
      IC->SetPsiDegIC(QFU);
      IC->SetVgroundFpsIC(0.);
      if (!FDMExec->RunIC()) {
        cerr << "Initialization from flight plan unsuccessful" << endl;
        exit(-1);
      }
    }
    //FDMExec->GetGroundReactions()->InitModel();

  } else {
    cerr << "  No Aircraft given" << endl << endl;
    delete FDMExec;
    exit(-1);
  }

  //if (FDMExec->Run()) cout << "Made Initial Run" << endl;
  //else {
  //  cerr << "Initial run failed " << endl;
  //  exit(-1);
  //}

}

bool check_crash_jsbsim(JSBSim::FGFDMExec* FDMExec) {
  double agl = FDMExec->GetPropertyManager()->GetNode("position/h-agl-ft")->getDoubleValue();
  if (agl>=0) return true;
  else {
    cerr << "Crash detected" << endl;
    return false;
  }
}
