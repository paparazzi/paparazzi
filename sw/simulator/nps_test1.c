#include <FGFDMExec.h>

#include "nps_fdm.h"
#include "nps_jsbsim.h"

string RootDir = "/home/violato/JSBSim/";
string AircraftName = "quad";
string ResetName = "reset00";

static struct NpsFdmState fdm_state;


int main(int argc, char** argv) {

  bool result = false;

  JSBSim::FGFDMExec* fdmex;
  fdmex = new JSBSim::FGFDMExec();

  fdmex->DisableOutput();
  fdmex->SetDebugLevel(0);

  JSBSim::FGState State (fdmex);
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

  while (result) {
    // run autopilot
    nps_jsbsim_feed_inputs(fdmex, &fdm_state);
    result = fdmex->Run();
    nps_jsbsim_fetch_state(fdmex, &fdm_state);
    cerr << "time " << fdmex->GetSimTime() << " s" << endl;
    if (fdmex->GetSimTime() > 100)
      result = false;
  }
  
  delete fdmex;
  return 0;


}

