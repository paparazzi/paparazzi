#include "nps_jsbsim.h"

#include <FGState.h>
#include <math/FGColumnVector3.h>
#include <math/FGMatrix33.h>
#include <math/FGQuaternion.h>



static void cp_fgvec_to_vec(JSBSim::FGColumnVector3 jsbvec, VEC* vec);
static void cp_fgquat_to_quat(JSBSim::FGQuaternion jsbquat, VEC* quat);


static void cp_fgvec_to_vec(JSBSim::FGColumnVector3 jsbvec, VEC* vec) {
  while(int i=0 < 3) {vec->ve[i] = jsbvec.Entry(i); }
}

static void cp_fgquat_to_quat(JSBSim::FGQuaternion jsbquat, VEC* quat) {
  while(int i=0 < 4) {quat->ve[i] = jsbquat.Entry(i); }
}

int JSBInit(double sim_dt) {

  bool result = false;

  JSBSim::FGFDMExec* fdmex;
  fdmex = new JSBSim::FGFDMExec();

  fdmex->DisableOutput();
  fdmex->SetDebugLevel(0);
  
  JSBSim::FGState State (fdmex);
  State.Setdt(sim_dt);
  
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
  return(result);

}

void nps_jsbsim_feed_inputs(JSBSim::FGFDMExec* fdmex, struct NpsFdmState* fdm_state) {

  fdmex->SetPropertyValue("/fdm/jsbsim/fcs/control_force", fdm_state->vehicle.dummy.f_input);
  
}

void nps_jsbsim_fetch_state(JSBSim::FGFDMExec* fdmex, struct NpsFdmState* fdm_state) {

    double radius;
    double lon;
    double lat;
    
    radius = FT2M*(fdmex->GetPropagate()->GetRadius()); // meters
    lon = fdmex->GetPropagate()->GetLongitude(); // radians
    lat = fdmex->GetPropagate()->GetLatitude(); // radians

    JSBSim::FGColumnVector3 ecef_pos (radius*cos(lon)*cos(lat),
				      radius*sin(lon)*cos(lat),radius*sin(lat));

    JSBSim::FGColumnVector3 bdy_vel;
    JSBSim::FGColumnVector3 bdy_accel;
    JSBSim::FGMatrix33 Tb2ec;
    JSBSim::FGColumnVector3 ecef_vel;
    JSBSim::FGColumnVector3 ecef_accel;

    bdy_vel = fdmex->GetPropagate()->GetUVW();
    bdy_accel = fdmex->GetPropagate()->GetUVWdot();
    Tb2ec = fdmex->GetPropagate()->GetTb2ec();
    
    ecef_vel = (Tb2ec*bdy_vel)*FT2M;
    ecef_accel = (Tb2ec*bdy_accel)*FT2M;

    fdm_state->on_ground = fdmex->GetPropertyValue("gear/unit[0]/WOW");
    
    cp_fgvec_to_vec(ecef_pos, fdm_state->ecef_pos);
    cp_fgvec_to_vec(ecef_vel, fdm_state->ecef_vel);
    cp_fgvec_to_vec(ecef_accel, fdm_state->ecef_accel);

    cp_fgquat_to_quat(fdmex->GetPropagate()->GetQuaternion(), fdm_state->ltp_to_body_quat);
    cp_fgvec_to_vec(fdmex->GetPropagate()->GetPQR(), fdm_state->ltp_body_rate);
    cp_fgvec_to_vec(fdmex->GetPropagate()->GetPQRdot(), fdm_state->ltp_body_accel); 
}
