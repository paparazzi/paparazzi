/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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
 */

/**
 * @file nps_fdm_jsbsim.c
 * Flight Dynamics Model (FDM) for NPS using JSBSim.
 *
 * This is an FDM for NPS that uses JSBSim as the simulation engine.
 */

#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <models/FGPropulsion.h>
#include <models/FGGroundReactions.h>
#include <models/FGAccelerations.h>
#include <stdlib.h>
#include "nps_fdm.h"
#include "generated/airframe.h"
#include "math/pprz_geodetic.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"

/// Macro to convert from feet to metres
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

/// Holds all necessary NPS FDM state information
struct NpsFdm fdm;

/// The JSBSim executive object
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

/**
 * Feed JSBSim with the latest actuator commands.
 * 
 * @param commands   Pointer to array of doubles holding actuator commands
 */
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

/**
 * Populates the NPS fdm struct after a simulation step.
 */
static void fetch_state(void) {

  FGPropertyManager* node = FDMExec->GetPropertyManager()->GetNode("simulation/sim-time-sec");
  fdm.time = node->getDoubleValue();

  FGPropagate* propagate = FDMExec->GetPropagate();
  FGAccelerations* accelerations = FDMExec->GetAccelerations();

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
  const FGColumnVector3& fg_body_ecef_accel = accelerations->GetUVWdot();
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
  jsbsimvec_to_rate(&fdm.body_ecef_rotaccel,&accelerations->GetPQRdot());

}

/**
 * Initializes JSBSim.
 *
 * Sets up the JSBSim executive and loads initial conditions
 * Exits NPS with -1 if models or ICs fail to load
 *
 * @param dt   The desired simulation timestep
 *
 * @warning Needs PAPARAZZI_HOME defined to find the config files
 */
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

/**
 * Initialize the ltp from the JSBSim location.
 *
 * @todo The magnetic field is hardcoded, make location dependent
 * (might be able to use JSBSim sensors)
 */
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

/**
 * Convert JSBSim location format and struct to NPS location format and struct.
 *
 * JSBSim is in feet by default, NPS in metres
 *
 * @param fdm_location Pointer to EcefCoor_d struct
 * @param jsb_location Pointer to FGLocation struct
 */
static void jsbsimloc_to_loc(EcefCoor_d* fdm_location, const FGLocation* jsb_location){

  fdm_location->x = MetersOfFeet(jsb_location->Entry(1));
  fdm_location->y = MetersOfFeet(jsb_location->Entry(2));
  fdm_location->z = MetersOfFeet(jsb_location->Entry(3));

}

/**
 * Convert JSBSim vector format and struct to NPS vector format and struct.
 *
 * JSBSim is in feet by default, NPS in metres
 *
 * @param fdm_vector    Pointer to DoubleVect3 struct
 * @param jsb_vector    Pointer to FGColumnVector3 struct
 */
static void jsbsimvec_to_vec(DoubleVect3* fdm_vector, const FGColumnVector3* jsb_vector) {

  fdm_vector->x = MetersOfFeet(jsb_vector->Entry(1));
  fdm_vector->y = MetersOfFeet(jsb_vector->Entry(2));
  fdm_vector->z = MetersOfFeet(jsb_vector->Entry(3));

}

/**
 * Convert JSBSim quaternion struct to NPS quaternion struct.
 *
 * @param fdm_quat    Pointer to DoubleQuat struct
 * @param jsb_quat    Pointer to FGQuaternion struct
 */
static void jsbsimquat_to_quat(DoubleQuat* fdm_quat, const FGQuaternion* jsb_quat){

  fdm_quat->qi = jsb_quat->Entry(1);
  fdm_quat->qx = jsb_quat->Entry(2);
  fdm_quat->qy = jsb_quat->Entry(3);
  fdm_quat->qz = jsb_quat->Entry(4);

}

/**
 * Convert JSBSim rates vector struct to NPS rates struct.
 *
 * @param fdm_rate    Pointer to DoubleRates struct
 * @param jsb_vector  Pointer to FGColumnVector3 struct
 */
static void jsbsimvec_to_rate(DoubleRates* fdm_rate, const FGColumnVector3* jsb_vector) {

  fdm_rate->p = jsb_vector->Entry(1);
  fdm_rate->q = jsb_vector->Entry(2);
  fdm_rate->r = jsb_vector->Entry(3);

}

/**
 * Convert JSBSim location to NPS LLH.
 *
 * Gets geodetic latitude, longitude and height above sea level in metres
 *
 * @param fdm_lla   Pointer to LlaCoor_d struct
 * @param propagate Pointer to JSBSim FGPropagate object
 */
void llh_from_jsbsim(LlaCoor_d* fdm_lla, FGPropagate* propagate) {

  fdm_lla->lat = propagate->GetGeodLatitudeRad();
  fdm_lla->lon = propagate->GetLongitude();
  fdm_lla->alt = MetersOfFeet(propagate->GetAltitudeASLmeters());
  //printf("geodetic alt: %f\n", MetersOfFeet(propagate->GetGeodeticAltitude()));
  //printf("ground alt: %f\n", MetersOfFeet(propagate->GetDistanceAGL()));
  //printf("ASL alt: %f\n", MetersOfFeet(propagate->GetAltitudeASLmeters()));

}

/**
 * Convert JSBSim location to NPS LLA.
 *
 * Gets geocentric latitude, longitude and geocentric radius
 *
 * @param fdm_lla   Pointer to LlaCoor_d struct
 * @param propagate Pointer to JSBSim FGPropagate object
 */
void lla_from_jsbsim_geocentric(LlaCoor_d* fdm_lla, FGPropagate* propagate) {

  fdm_lla->lat = propagate->GetLatitude();
  fdm_lla->lon = propagate->GetLongitude();
  fdm_lla->alt = MetersOfFeet(propagate->GetRadius());

}

/**
 * Convert JSBSim location to NPS LLA.
 *
 * Gets geodetic latitude, longitude and geodetic altitude in metres
 *
 * @param fdm_lla   Pointer to LlaCoor_d struct
 * @param propagate Pointer to JSBSim FGPropagate object
 */
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
