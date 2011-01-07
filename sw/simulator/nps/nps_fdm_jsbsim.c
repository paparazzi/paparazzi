#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <models/FGPropulsion.h>
#include <models/FGGroundReactions.h>
#include <stdlib.h>
#include "nps_fdm.h"
#include "generated/airframe.h"
#include "math/pprz_geodetic.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"

#define MetersOfFeet(_f) ((_f)/3.2808399)

using namespace JSBSim;

static void feed_jsbsim(double* commands);
static void fetch_state(void);

static void jsbsimvec_to_vec(DoubleVect3* fdm_vector, const FGColumnVector3* jsb_vector);
static void jsbsimloc_to_loc(EcefCoor_d* fdm_location, const FGLocation* jsb_location);
static void jsbsimquat_to_quat(DoubleQuat* fdm_quat, const FGQuaternion* jsb_quat);
static void jsbsimvec_to_rate(DoubleRates* fdm_rate, const FGColumnVector3* jsb_vector);
static void llh_from_jsbsim(LlaCoor_d* fdm_lla, FGPropagate* propagate);
static void lla_from_jsbsim_geodetic(LlaCoor_d* fdm_lla, FGPropagate* propagate);
static void lla_from_jsbsim_geocentric(LlaCoor_d* fdm_lla, FGPropagate* propagate);
//static void rate_to_vec(DoubleVect3* vector, DoubleRates* rate);

static void init_jsbsim(double dt);
static void init_ltp(void);

struct NpsFdm fdm;
static FGFDMExec* FDMExec;
static struct LtpDef_d ltpdef;

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
  for (i=0; i<COMMANDS_NB; i++) {
    sprintf(buf,"fcs/%s",names[i]);
    property = string(buf);
    FDMExec->GetPropertyManager()->SetDouble(property,commands[i]);
  }
}

static void fetch_state(void) {

  FGPropertyManager* node = FDMExec->GetPropertyManager()->GetNode("simulation/sim-time-sec");
  fdm.time = node->getDoubleValue();

  FGPropagate* propagate = FDMExec->GetPropagate();

  fdm.on_ground = FDMExec->GetGroundReactions()->GetWOW();

  /*
   * position
   */
  jsbsimloc_to_loc(&fdm.ecef_pos,&propagate->GetLocation());
  fdm.hmsl = propagate->GetAltitudeASLmeters();

  /*
   * linear speed and accelerations
   */

  /* in body frame */
  const FGColumnVector3& fg_body_ecef_vel = propagate->GetUVW();
  jsbsimvec_to_vec(&fdm.body_ecef_vel, &fg_body_ecef_vel);
  const FGColumnVector3& fg_body_ecef_accel = propagate->GetUVWdot();
  jsbsimvec_to_vec(&fdm.body_ecef_accel,&fg_body_ecef_accel);

  /* in LTP frame */
  const FGMatrix33& body_to_ltp = propagate->GetTb2l();
  const FGColumnVector3& fg_ltp_ecef_vel = body_to_ltp * fg_body_ecef_vel;
  jsbsimvec_to_vec((DoubleVect3*)&fdm.ltp_ecef_vel, &fg_ltp_ecef_vel);
  const FGColumnVector3& fg_ltp_ecef_accel = body_to_ltp * fg_body_ecef_accel;
  jsbsimvec_to_vec((DoubleVect3*)&fdm.ltp_ecef_accel, &fg_ltp_ecef_accel);

  /* in ECEF frame */
  const FGMatrix33& body_to_ecef = propagate->GetTb2ec();
  const FGColumnVector3& fg_ecef_ecef_vel = body_to_ecef * fg_body_ecef_vel;
  jsbsimvec_to_vec((DoubleVect3*)&fdm.ecef_ecef_vel, &fg_ecef_ecef_vel);
  const FGColumnVector3& fg_ecef_ecef_accel = body_to_ecef * fg_body_ecef_accel;
  jsbsimvec_to_vec((DoubleVect3*)&fdm.ecef_ecef_accel, &fg_ecef_ecef_accel);

  /* in LTP pprz */
  ned_of_ecef_point_d(&fdm.ltpprz_pos, &ltpdef, &fdm.ecef_pos);
  ned_of_ecef_vect_d(&fdm.ltpprz_ecef_vel, &ltpdef, &fdm.ecef_ecef_vel);
  ned_of_ecef_vect_d(&fdm.ltpprz_ecef_accel, &ltpdef, &fdm.ecef_ecef_accel);

  /* llh */
  llh_from_jsbsim(&fdm.lla_pos, propagate);
  //test123(&fdm.lla_pos, propagate);

  //for debug
  lla_from_jsbsim_geodetic(&fdm.lla_pos_geod, propagate);
  lla_from_jsbsim_geocentric(&fdm.lla_pos_geoc, propagate);
  lla_of_ecef_d(&fdm.lla_pos_pprz, &fdm.ecef_pos);
  fdm.agl = MetersOfFeet(propagate->GetDistanceAGL());


  /*
   * attitude
   */
  const FGQuaternion jsb_quat = propagate->GetQuaternion();
  jsbsimquat_to_quat(&fdm.ltp_to_body_quat, &jsb_quat);
  /* convert to eulers */
  DOUBLE_EULERS_OF_QUAT(fdm.ltp_to_body_eulers, fdm.ltp_to_body_quat);
  /* the "false" pprz lpt */
  /* FIXME: use jsbsim ltp for now */
  EULERS_COPY(fdm.ltpprz_to_body_eulers, fdm.ltp_to_body_eulers);
  QUAT_COPY(fdm.ltpprz_to_body_quat, fdm.ltp_to_body_quat);

  /*
   * rotational speed and accelerations
   */
  jsbsimvec_to_rate(&fdm.body_ecef_rotvel,&propagate->GetPQR());
  jsbsimvec_to_rate(&fdm.body_ecef_rotaccel,&propagate->GetPQRdot());

}

static void init_jsbsim(double dt) {

  char buf[1024];
  string rootdir;

  sprintf(buf,"%s/conf/simulator/jsbsim/",getenv("PAPARAZZI_HOME"));
  rootdir = string(buf);
  FDMExec = new FGFDMExec();

  FDMExec->Setsim_time(0.);
  FDMExec->Setdt(dt);

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

  //initRunning for all engines
  FDMExec->GetPropulsion()->InitRunning(-1);

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

  FGPropagate* propagate = FDMExec->GetPropagate();

  jsbsimloc_to_loc(&fdm.ecef_pos,&propagate->GetLocation());
  ltp_def_from_ecef_d(&ltpdef,&fdm.ecef_pos);

  fdm.ltp_g.x = 0.;
  fdm.ltp_g.y = 0.;
  fdm.ltp_g.z = 9.81;

  fdm.ltp_h.x = 0.4912;
  fdm.ltp_h.y = 0.1225;
  fdm.ltp_h.z = 0.8624;
}

static void jsbsimloc_to_loc(EcefCoor_d* fdm_location, const FGLocation* jsb_location){

  fdm_location->x = MetersOfFeet(jsb_location->Entry(1));
  fdm_location->y = MetersOfFeet(jsb_location->Entry(2));
  fdm_location->z = MetersOfFeet(jsb_location->Entry(3));

}

static void jsbsimvec_to_vec(DoubleVect3* fdm_vector, const FGColumnVector3* jsb_vector) {

  fdm_vector->x = MetersOfFeet(jsb_vector->Entry(1));
  fdm_vector->y = MetersOfFeet(jsb_vector->Entry(2));
  fdm_vector->z = MetersOfFeet(jsb_vector->Entry(3));

}

static void jsbsimquat_to_quat(DoubleQuat* fdm_quat, const FGQuaternion* jsb_quat){

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

//longitude and geodetic latitude in rad and height above sea level in m
void llh_from_jsbsim(LlaCoor_d* fdm_lla, FGPropagate* propagate) {
  fdm_lla->lat = propagate->GetGeodLatitudeRad();
  fdm_lla->lon = propagate->GetLongitude();
  fdm_lla->alt = MetersOfFeet(propagate->GetAltitudeASLmeters());
  //printf("geodetic alt: %f\n", MetersOfFeet(propagate->GetGeodeticAltitude()));
  //printf("ground alt: %f\n", MetersOfFeet(propagate->GetDistanceAGL()));
  //printf("ASL alt: %f\n", MetersOfFeet(propagate->GetAltitudeASLmeters()));
}

void lla_from_jsbsim_geocentric(LlaCoor_d* fdm_lla, FGPropagate* propagate) {
  fdm_lla->lat = propagate->GetLatitude();
  fdm_lla->lon = propagate->GetLongitude();
  fdm_lla->alt = MetersOfFeet(propagate->GetRadius());
}

void lla_from_jsbsim_geodetic(LlaCoor_d* fdm_lla, FGPropagate* propagate) {
  fdm_lla->lat = propagate->GetGeodLatitudeRad();
  fdm_lla->lon = propagate->GetLongitude();
  fdm_lla->alt = MetersOfFeet(propagate->GetGeodeticAltitude());
}


#if 0
static void rate_to_vec(DoubleVect3* vector, DoubleRates* rate) {

  vector->x = rate->p;
  vector->y = rate->q;
  vector->z = rate->r;

}
#endif
