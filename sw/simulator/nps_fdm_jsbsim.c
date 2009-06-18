#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <FGState.h>
#include "nps_fdm.h"
#include "airframe.h"

using namespace JSBSim;

static void feed_jsbsim(double* commands);
static void fetch_state(void);

FGFDMExec* FDMExec;

void nps_fdm_init(double dt) {

#if 0

  JSBSim::FGState* State;

  FDMExec = new FGFDMExec();

  State = FDMExec->GetState();
  State->Setsim_time(0.);
  State->Setdt(dt);

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
  
  FDMExec->Run();

#endif
  
}

void nps_fdm_run_step(double* commands) {

  feed_jsbsim(commands);

  FDMExec->Run();

  fetch_state();

}

static void feed_jsbsim(double* commands) {

#if 0
  char buf[64];
  
  for (int i=0; i<SERVOS_NB; i++) {
    sprintf(buf,"fcs/%s",ACTUATOR_COMMAND_NAMES[i]);
    FDMExec->GetPropertyManager()->SetDouble(buf,commands[i]);
  }
#endif

}

static void fetch_state(void) {

#if 0
  FGGroundReactions* ground_reactions;
  FGPropertyManager* cur_node;
  double cur_value
  
  ground_reactions = FGGroundReactions(FDMExec);

  bool_t fdm.on_ground /* with */ bool ground_reactions->GetWOW();
  VEC* ecef_pos /* with */ bool ground_reactions->GetWOW();

  VEC* ecef_vel;
  VEC* ecef_accel;

  VEC* ltp_to_body_quat;
  VEC* ltp_body_rate;
  VEC* ltp_body_accel;
#endif
  /* JSBSim to fdm */

}
