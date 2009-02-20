#include "nps_fdm.h"
#include <FGFDMExec.h>
#include <math/FGColumnVector3.h>

string RootDir = "/home/violato/JSBSim/";
string AircraftName = "ball";
string ResetName = "reset00";

static struct NpsFdmState fdm_state;



static void feed_inputs(JSBSim::FGFDMExec* fdmex) {

  //double hover_force = 0.;

  fdmex->SetPropertyValue("/fdm/jsbsim/fcs/parachute_reef_pos_norm", fdm_state.dummy.f_input);
  

}

static void fetch_state(JSBSim::FGFDMExec* fdmex) {

  //  double foo = fdmex->GetPropagate()->GetInertialVelocityMagnitude();
  //  double foo = fdmex->GetPropagate()->GetGeodLatitudeRad();
  //  cerr << "lat " << foo << endl ;
  const JSBSim::FGColumnVector3 vel = fdmex->GetPropagate()->GetVel();
  //  cerr << "vel " << vel << endl ;

  //  eulers->ve[EULER_PHI] = vel

  
  //  fdm_state.ecef_pos = 

}

int main(int argc, char** argv) {

  bool result = false;

  JSBSim::FGFDMExec* fdmex;
  fdmex = new JSBSim::FGFDMExec();

  fdmex->DisableOutput();
  fdmex->SetDebugLevel(0);

  if ( ! fdmex->LoadModel( RootDir + "aircraft",
			   RootDir + "engine",
			   RootDir + "systems",
			   AircraftName,
			   1)) {
    cerr << "  JSBSim could not be started" << endl << endl;
    delete fdmex;
    exit(-1);
  }

  //fdmex->PrintPropertyCatalog();

  JSBSim::FGInitialCondition* IC = fdmex->GetIC();
  if ( ! IC->Load(ResetName)) {
    delete fdmex;
    cerr << "Initialization unsuccessful" << endl;
    exit(-1);
  }

  result = fdmex->RunIC();

  while (result) {
    // run autopilot
    feed_inputs(fdmex);
    result = fdmex->Run();
    fetch_state(fdmex);
    //cerr << "time " << fdmex->GetSimTime() << " h " << fdmex->GetPropagate()->Geth() << endl ;
    //cerr << "reef " << fdmex->GetPropertyValue("/fdm/jsbsim/fcs/parachute_reef_pos_norm") << endl ;
    if (fdmex->GetPropagate()->Geth() < 10)
      result = false;
  }
  
  delete fdmex;
  return 0;


}

