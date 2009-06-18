#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <FGState.h>
#include "nps_fdm.h"
#include "airframe.h"

using namespace JSBSim;

static void feed_jsbsim(double* commands);
static void fetch_state(void);
static void jsbsimvec_to_vec(VEC* vector, const FGColumnVector3* jsb_vector);
static void jsbsimloc_to_vec(VEC* vector, FGLocation* location);
static void jsbsimquat_to_vec(VEC* ltp_to_body_quat, FGQuaternion* quat);

struct NpsFdm fdm;
FGFDMExec* FDMExec;

void nps_fdm_init(double dt) {
  
  char rootdir[1024];
  JSBSim::FGState* State;
  
  sprintf(rootdir,"%s/conf/simulator",getenv("PAPARAZZI_HOME"));
  FDMExec = new FGFDMExec();
  
  State = FDMExec->GetState();
  State->Setsim_time(0.);
  State->Setdt(dt);
  
  FDMExec->DisableOutput();
  FDMExec->SetDebugLevel(0); // No DEBUG messages
  
  if ( ! FDMExec->LoadModel( strcat(rootdir,"aircraft"),
			     strcat(rootdir,"engine"),
			     strcat(rootdir,"systems"),
			     AIRFRAME_NAME)){
#ifdef DEBUG
    cerr << "  JSBSim could not be started" << endl << endl;
#endif
    delete FDMExec;
    exit(-1);
  }
  
  JSBSim::FGInitialCondition *IC = FDMExec->GetIC();
  if ( ! IC->Load(NPS_INITIAL_CONDITITONS)) {
#ifdef DEBUG
    cerr << "Initialization unsuccessful" << endl;
#endif
    delete FDMExec;
    exit(-1);
  }
  
}

void nps_fdm_run_step(double* commands) {

  feed_jsbsim(commands);

  FDMExec->Run();

  fetch_state();

}

static void feed_jsbsim(double* commands) {

  char buf[64];
  const char* names[] = NPS_ACTUATOR_NAMES;
  
  for (int i=0; i<SERVOS_NB; i++) {
    sprintf(buf,"fcs/%s",names[i]);
    FDMExec->GetPropertyManager()->SetDouble(buf,commands[i]);
  }

}

static void fetch_state(void) {

  FGGroundReactions* ground_reactions;
  FGPropagate* propagate;
  FGPropagate::VehicleState* VState;

  ground_reactions = FDMExec->GetGroundReactions();
  propagate = FDMExec->GetPropagate();
  VState = propagate->GetVState();

  fdm.on_ground = ground_reactions->GetWOW();

  jsbsimloc_to_vec(fdm.ecef_pos,&VState->vLocation);
  jsbsimvec_to_vec(fdm.ecef_vel,&VState->vUVW);

  jsbsimvec_to_vec(fdm.body_accel,&propagate->GetUVWdot());

  jsbsimquat_to_vec(fdm.ltp_to_body_quat,&VState->vQtrn);
  jsbsimvec_to_vec(fdm.body_rate,&VState->vPQR);
  jsbsimvec_to_vec(fdm.body_rate_dot,&propagate->GetPQRdot());

}

static void jsbsimloc_to_vec(VEC* vector, FGLocation* location) {
  int i;
  for (i=0; i<3; i++) {
    vector->ve[i] = location->Entry(i+1);
  }
}

static void jsbsimquat_to_vec(VEC* vector, FGQuaternion* quat) {
  int i;
  for (i=0; i<3; i++) {
    vector->ve[i] = quat->Entry(i+1);
  }
}

static void jsbsimvec_to_vec(VEC* vector, const FGColumnVector3* jsb_vector) {
  int i;
  for (i=0; i<3; i++) {
    vector->ve[i] = jsb_vector->Entry(i+1);
  }  
}

