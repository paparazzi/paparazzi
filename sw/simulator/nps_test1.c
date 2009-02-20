#include "nps_fdm.h"


static struct NpsFdmState fdm_state;



static void feed_inputs(JSBSim::FGFDMExec* fdmex) {

  double hover_force = 0.;

  fdmex->SetPropertyValue("/fdm/jsbsim/fcs/force_front", fdm_state.dummy.f_input);
  

}

static void fetch_state(JSBSim::FGFDMExec* fdmex) {

  //  double foo = fdmex->GetPropagate()->GetInertialVelocityMagnitude();
  //  double foo = fdmex->GetPropagate()->GetGeodLatitudeRad();
  // cerr << "lat " << foo << endl ;
  const JSBSim::FGColumnVector3 vel = fdmex->GetPropagate()->GetVel();
  cerr << "vel " << vel << endl ;

  //  eulers->ve[EULER_PHI] = vel

  
  //  fdm_state.ecef_pos = 

}




int main(int argc, char** argv) {

  


}

