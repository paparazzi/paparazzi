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



#include <stdlib.h>
#include <stdio.h>
#include <glib.h>
#include <getopt.h>

#include <iostream>

// ignore stupid warnings in JSBSim
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <FGFDMExec.h>
//#include <SGGeod.hxx>
#include <math/FGLocation.h>
#pragma GCC diagnostic pop

#include "sim_ac_flightgear.h"

using namespace std;

//#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include "sim_ac_jsbsim.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
GLOBAL DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

bool run_model;
bool run_fg = false;

string ICName;
string AircraftName;
JSBSim::FGFDMExec* FDMExec;

static void     sim_parse_options(int argc, char** argv);
static void     sim_init(void);
static gboolean sim_periodic(gpointer data);

#ifdef __APPLE__
string ivyBus = "224.255.255.255";
#else
string ivyBus = "127.255.255.255";
#endif
string fgAddress = "127.0.0.1";

static void ivy_transport_init(void);


static void sim_init(void) {

  run_model = false;

  jsbsim_init();

  // init sensors ? or discribe them in jSBSim

  ivy_transport_init();

  // main AP init (feed the sensors once before ?)
  sim_autopilot_init();

  printf("sys_time frequency: %f\n", (float)SYS_TIME_FREQUENCY);

}

static gboolean sim_periodic(gpointer data __attribute__ ((unused))) {
  static uint8_t ncalls = 0;

  /* read actuators positions and feed JSBSim inputs */
  copy_inputs_to_jsbsim(FDMExec);

  /* run JSBSim flight model */
  bool result = true;
  if (run_model) {
    result = FDMExec->Run();
  }
  /* check if still flying */
  result = check_crash_jsbsim(FDMExec);

  /* read outputs from model state */
  copy_outputs_from_jsbsim(FDMExec);

  /* send outputs to flightgear for visualisation */
  if (run_fg == true)
    sim_ac_flightgear_send(FDMExec);

  /* run the airborne code
     with 60 Hz, even if JSBSim runs with a multiple of this */
  if (ncalls == 0) {
    //  airborne_run_one_step();
    autopilot_event_task();
    autopilot_periodic_task();
  }
  ++ncalls;
  if (ncalls == JSBSIM_SPEEDUP) ncalls = 0;

  return result;
}

static gboolean systime_periodic(gpointer data __attribute__ ((unused))) {
  sys_tick_handler();
  return true;
}


int main ( int argc, char** argv) {

  sim_parse_options(argc, argv);

  sim_init();

  if (run_fg == true)
    sim_ac_flightgear_init(fgAddress.c_str(), 5501);

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  guint systime_dt_ms = SYSTIME_PERIOD;
  printf("sys_time period in msec: %d\n", systime_dt_ms);
  guint jsbsim_dt_ms = JSBSIM_PERIOD;
  printf("jsbsim period in msec: %d\n", jsbsim_dt_ms);

  g_timeout_add(jsbsim_dt_ms, sim_periodic, NULL);
  g_timeout_add(systime_dt_ms, systime_periodic, NULL);

  g_main_loop_run(ml);

  delete FDMExec;
  return 0;
}


static void ivy_transport_init(void) {
  const char* agent_name = AIRFRAME_NAME"_JSBSIM";
  IvyInit(agent_name, "READY", NULL, NULL, NULL, NULL);
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
    else if (argument == "-jsbsim") {
      // Compatibility with ocaml
    }
    else if (argument == "-b") {
      ivyBus = string(argv[++i]);
    }
    else if (argument == "-fg") {
      run_fg = true;
      fgAddress = string(argv[++i]);
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

      // FGInitialCondition::SetAltitudeASLFtIC
      // requires this function to be called
      // before itself
      IC->SetVgroundFpsIC(0.);

      // Use flight plan initial conditions
      IC->SetLatitudeDegIC(NAV_LAT0 / 1e7);
      IC->SetLongitudeDegIC(NAV_LON0 / 1e7);

      IC->SetAltitudeASLFtIC((GROUND_ALT + 2.0) / FT2M);
      IC->SetTerrainElevationFtIC(GROUND_ALT / FT2M);
      IC->SetPsiDegIC(QFU);
      IC->SetVgroundFpsIC(0.);

      //initRunning for all engines
      FDMExec->GetPropulsion()->InitRunning(-1);
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

  double agl = FDMExec->GetPropagate()->GetDistanceAGL(), // in ft
  lat = FDMExec->GetPropagate()->GetLatitude(), // in rad
  lon = FDMExec->GetPropagate()->GetLongitude(); // in rad

  if (agl< -1e-5) {
    cerr << "Crash detected: agl < 0" << endl << endl;
    return false;
  }
  if (agl > 1e5 || abs(lat) > M_PI_2 || abs(lon) > M_PI) {
    cerr << "Simulation divergence: Lat=" << lat
         << " rad, lon=" << lon << " rad, agl=" << agl << " ft" << endl
         << endl;
    return false;
  }

  if (isnan(agl) || isnan(lat) || isnan(lon)) {
    cerr << "JSBSim is producing NaNs. Exiting." << endl << endl;
    return false;
  }
  return true;
}
