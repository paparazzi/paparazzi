#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <FGState.h>
#include <stdlib.h>
#include "nps_fdm.h"
#include "6dof.h"
#include "airframe.h"
#include "pprz_geodetic.h"
#include "pprz_geodetic_double.h"
#include "pprz_geodetic_float.h"
#include "pprz_algebra.h"
#include "pprz_algebra_float.h"

#define MetersOfFeet(_f) ((_f)/3.2808399)

using namespace JSBSim;

static void feed_jsbsim(double* commands);
static void fetch_state(void);

static void jsbsimvec_to_vec(DoubleVect3* fdm_vector, const FGColumnVector3* jsb_vector);
static void jsbsimloc_to_loc(EcefCoor_d* fdm_location, FGLocation* jsb_location);
static void jsbsimquat_to_quat(DoubleQuat* fdm_quat, FGQuaternion* jsb_quat);
static void jsbsimvec_to_rate(DoubleRates* fdm_rate, const FGColumnVector3* jsb_vector);
static void jsbsimloc_to_lla(LlaCoor_d* fdm_lla, FGLocation* jsb_location);
//static void rate_to_vec(DoubleVect3* vector, DoubleRates* rate);
static void ltpdef_copy(struct LtpDef_f* ltpdef_f, struct LtpDef_d* ltpdef_d);

static void init_jsbsim(double dt);
static void init_ltp(void);

struct NpsFdm fdm;
FGFDMExec* FDMExec;
struct LtpDef_f ltpdef;

void nps_fdm_init(double dt) {

  init_jsbsim(dt);

  FDMExec->RunIC();

  init_ltp();

  fetch_state();
 
}

void nps_fdm_run_step(double* commands) {

  feed_jsbsim(commands);

  FDMExec->Run();

  fetch_state();

}

static void feed_jsbsim(double* commands) {

  char buf[64];
  const char* names[] = NPS_ACTUATOR_NAMES;
  string property;

  int i;
  for (i=0; i<SERVOS_NB; i++) {
    sprintf(buf,"fcs/%s",names[i]);
    property = string(buf);
    FDMExec->GetPropertyManager()->SetDouble(property,commands[i]);
  }
}

static void fetch_state(void) {

  FGPropertyManager* node = FDMExec->GetPropertyManager()->GetNode("sim-time-sec");
  fdm.time = node->getDoubleValue();

  //  printf("%f\n", fdm.time); 

  /* Commented are the calculus of acceleration in the case where
     jsbsim does not consider body rotational velocity */

  FGPropagate* propagate;
  FGPropagate::VehicleState* VState;
  // DoubleVect3 noninertial_accel;
  // DoubleVect3 dummy_vector;
  
  propagate = FDMExec->GetPropagate();
  VState = propagate->GetVState();
  
  fdm.on_ground = FDMExec->GetGroundReactions()->GetWOW();
  
  jsbsimloc_to_loc(&fdm.ecef_pos,&VState->vLocation);
  jsbsimvec_to_vec(&fdm.body_ecef_vel,&VState->vUVW);
  jsbsimvec_to_vec(&fdm.body_ecef_accel,&propagate->GetUVWdot());
  //jsbsimvec_to_vec(&noninertial_accel,&propagate->GetUVWdot());

  jsbsimquat_to_quat(&fdm.ltp_to_body_quat,&VState->vQtrn);
  jsbsimvec_to_rate(&fdm.body_ecef_rotvel,&VState->vPQR);
  jsbsimvec_to_rate(&fdm.body_ecef_rotaccel,&propagate->GetPQRdot());
  // rate_to_vec(&dummy_vector,&fdm.body_ecef_rotvel);
  // DOUBLE_VECT3_CROSS_PRODUCT(fdm.body_ecef_accel,dummy_vector,fdm.body_ecef_vel);
  // DOUBLE_VECT3_SUM(fdm.body_ecef_accel,fdm.body_ecef_accel,noninertial_accel)
  
  struct EcefCoor_f ecefpos_f;
  VECT3_COPY(ecefpos_f, fdm.ecef_pos);
  struct NedCoor_f ned;
  ned_of_ecef_point_f(&ned, &ltpdef, &ecefpos_f);
  VECT3_COPY(fdm.ltpprz_pos,ned);
  DOUBLE_EULERS_OF_QUAT(fdm.ltp_to_body_eulers, fdm.ltp_to_body_quat);
  jsbsimloc_to_lla(&fdm.lla_pos, &VState->vLocation);
  
  
}


static void init_jsbsim(double dt) {

  char buf[1024];
  string rootdir;
  JSBSim::FGState* State;
  
  sprintf(buf,"%s/conf/simulator/jsbsim/",getenv("PAPARAZZI_HOME"));
  rootdir = string(buf);
  FDMExec = new FGFDMExec();
  
  State = FDMExec->GetState();
  State->Setsim_time(0.);
  State->Setdt(dt);
  
  FDMExec->DisableOutput();
  FDMExec->SetDebugLevel(0); // No DEBUG messages
  
  if ( ! FDMExec->LoadModel( rootdir + "aircraft",
			     rootdir + "engine",
			     rootdir + "systems",
			     AIRFRAME_NAME,
			     false)){
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

static void init_ltp(void) {

  FGPropagate* propagate;
  FGPropagate::VehicleState* VState;
  struct LtpDef_d ltpdef_d;
  
  propagate = FDMExec->GetPropagate();
  VState = propagate->GetVState();
  
  jsbsimloc_to_loc(&fdm.ecef_pos,&VState->vLocation);
  ltp_def_from_ecef_d(&ltpdef_d,&fdm.ecef_pos);
  ltpdef_copy(&ltpdef, &ltpdef_d);

}

static void jsbsimloc_to_loc(EcefCoor_d* fdm_location, FGLocation* jsb_location){

  fdm_location->x = jsb_location->Entry(1);
  fdm_location->y = jsb_location->Entry(2);
  fdm_location->z = jsb_location->Entry(3);
  
}
 
static void jsbsimvec_to_vec(DoubleVect3* fdm_vector, const FGColumnVector3* jsb_vector) {

  fdm_vector->x = jsb_vector->Entry(1);
  fdm_vector->y = jsb_vector->Entry(2);
  fdm_vector->z = jsb_vector->Entry(3);

}

static void jsbsimquat_to_quat(DoubleQuat* fdm_quat, FGQuaternion* jsb_quat){
  
  fdm_quat->qi = jsb_quat->Entry(1);
  fdm_quat->qx = jsb_quat->Entry(2);
  fdm_quat->qy = jsb_quat->Entry(3);
  fdm_quat->qz = jsb_quat->Entry(4);

}

static void jsbsimvec_to_rate(DoubleRates* fdm_rate, const FGColumnVector3* jsb_vector) {

  fdm_rate->p = jsb_vector->Entry(1);
  fdm_rate->q = jsb_vector->Entry(2);
  fdm_rate->r = jsb_vector->Entry(3);

}

#if 0
static void rate_to_vec(DoubleVect3* vector, DoubleRates* rate) {

  vector->x = rate->p;
  vector->y = rate->q;
  vector->z = rate->r;

}
#endif

void jsbsimloc_to_lla(LlaCoor_d* fdm_lla, FGLocation* jsb_location) {

  fdm_lla->lat = jsb_location->GetLatitude();
  fdm_lla->lon = jsb_location->GetLongitude();
  fdm_lla->alt = MetersOfFeet(jsb_location->GetGeodAltitude());

}

void ltpdef_copy(struct LtpDef_f* ltpdef_f, struct LtpDef_d* ltpdef_d) {

  VECT3_COPY(ltpdef_f->ecef, ltpdef_d->ecef);
  LLA_COPY(ltpdef_f->lla, ltpdef_d->lla);
  MAT33_COPY(ltpdef_f->ltp_of_ecef, ltpdef_d->ltp_of_ecef);
  
}
