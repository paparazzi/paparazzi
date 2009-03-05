#include <glib.h>
#include <getopt.h>
#include <sys/time.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <FGFDMExec.h>
#include <FGState.h>

#define HOST_TIMEOUT_PERIOD 4
//#include "nps_fdm.h"
//#include "nps_jsbsim.h"

string RootDir = "/home/violato/JSBSim/";
string AircraftName = "quad";
string ResetName = "reset00";

JSBSim::FGFDMExec* fdmex;

//static struct NpsFdmState fdm_state;

static gboolean sim_periodic(gpointer data __attribute__ ((unused))) {
    // run autopilot
    //nps_jsbsim_feed_inputs(fdmex, &fdm_state);
    bool result = fdmex->Run();
    //nps_jsbsim_fetch_state(fdmex, &fdm_state);
    cerr << "time " << fdmex->GetSimTime() << " s" << endl;
    if (fdmex->GetSimTime() > 100)
      result = false;
    return result;
}

int main(int argc, char** argv) {

  bool result = false;

  fdmex = new JSBSim::FGFDMExec();

  fdmex->DisableOutput();
  fdmex->SetDebugLevel(0);

  JSBSim::FGState State(fdmex);
  State.Setdt(1./10.);

  if ( ! fdmex->LoadModel( RootDir + "aircraft",
			   RootDir + "engine",
			   RootDir + "systems",
			   AircraftName,
			   1)) {
    cerr << "  JSBSim could not be started" << endl << endl;
    delete fdmex;
    exit(-1);
  }

  JSBSim::FGInitialCondition* IC = fdmex->GetIC();
  if ( ! IC->Load(ResetName)) {
    delete fdmex;
    cerr << "Initialization unsuccessful" << endl;
    exit(-1);
  }

  result = fdmex->RunIC();

  IvyInit ("nps_test2", "nps_test2 READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  g_timeout_add(HOST_TIMEOUT_PERIOD, sim_periodic, NULL);

  g_main_loop_run(ml);
  
  
  delete fdmex;
  return 0;


}

