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

#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <models/FGPropulsion.h>
#include <models/FGGroundReactions.h>
#include <models/FGAccelerations.h>

#include "nps_fdm.h"
#include "math/pprz_geodetic.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"

/// Macro to convert from feet to metres
#define MetersOfFeet(_f) ((_f)/3.2808399)
#define FeetOfMeters(_m) ((_m)*3.2808399)

/** Name of the JSBSim model.
 *  Defaults to the AIRFRAME_NAME
 */
#ifndef NPS_JSBSIM_MODEL
#define NPS_JSBSIM_MODEL AIRFRAME_NAME
#endif

#ifdef NPS_INITIAL_CONDITITONS
#warning NPS_INITIAL_CONDITITONS was replaced by NPS_JSBSIM_INIT!
#warning Defaulting to flight plan location.
#endif

/** Minimum JSBSim timestep
  * Around 1/10000 seems to be good for ground impacts
  */
#define MIN_DT (1.0/10240.0)

using namespace JSBSim;
using namespace std;

static void feed_jsbsim(double* commands);
static void fetch_state(void);
static int check_for_nan(void);

static void jsbsimvec_to_vec(DoubleVect3* fdm_vector, const FGColumnVector3* jsb_vector);
static void jsbsimloc_to_loc(EcefCoor_d* fdm_location, const FGLocation* jsb_location);
static void jsbsimquat_to_quat(DoubleQuat* fdm_quat, const FGQuaternion* jsb_quat);
static void jsbsimvec_to_rate(DoubleRates* fdm_rate, const FGColumnVector3* jsb_vector);
static void llh_from_jsbsim(LlaCoor_d* fdm_lla, FGPropagate* propagate);
static void lla_from_jsbsim_geodetic(LlaCoor_d* fdm_lla, FGPropagate* propagate);
static void lla_from_jsbsim_geocentric(LlaCoor_d* fdm_lla, FGPropagate* propagate);

static void init_jsbsim(double dt);
static void init_ltp(void);

/// Holds all necessary NPS FDM state information
struct NpsFdm fdm;

/// The JSBSim executive object
static FGFDMExec* FDMExec;

static struct LtpDef_d ltpdef;

/// The largest distance between vehicle CG and contact point
double vehicle_radius_max;

/// Timestep used for higher fidelity near the ground
double min_dt;

void nps_fdm_init(double dt) {

  fdm.init_dt = dt;
  fdm.curr_dt = dt;
  //Sets up the high fidelity timestep as a multiple of the normal timestep
  for (min_dt = (1.0/dt); min_dt < (1/MIN_DT); min_dt += (1/dt)){}
  min_dt = (1/min_dt);

  fdm.nan_count = 0;

  init_jsbsim(dt);

  FDMExec->RunIC();

  init_ltp();

#if DEBUG_NPS_JSBSIM
  printf("fdm.time,fg_body_ecef_accel1,fg_body_ecef_accel2,fg_body_ecef_accel3,fdm.body_ecef_accel.x,fdm.body_ecef_accel.y,fdm.body_ecef_accel.z,fg_ltp_ecef_accel1,fg_ltp_ecef_accel2,fg_ltp_ecef_accel3,fdm.ltp_ecef_accel.x,fdm.ltp_ecef_accel.y,fdm.ltp_ecef_accel.z,fg_ecef_ecef_accel1,fg_ecef_ecef_accel2,fg_ecef_ecef_accel3,fdm.ecef_ecef_accel.x,fdm.ecef_ecef_accel.y,fdm.ecef_ecef_accel.z,fdm.ltpprz_ecef_accel.z,fdm.ltpprz_ecef_accel.y,fdm.ltpprz_ecef_accel.z,fdm.agl\n");
#endif

  fetch_state();

}

void nps_fdm_run_step(double* commands) {

  feed_jsbsim(commands);

  /* To deal with ground interaction issues, we decrease the time
     step as the vehicle is close to the ground. This is done predictively
     to ensure no weird accelerations or oscillations. From tests with a bouncing
     ball model in JSBSim, it seems that 10k steps per second is reasonable to capture
     all the dynamics. Higher might be a bit more stable, but really starting to push
     the simulation CPU requirements, especially for more complex models.
      - at init: get the largest radius from CG to any contact point (landing gear)
      - if descending...
        - find current number of timesteps to impact
        - if impact imminent, calculate a new timestep to use (with limit)
      - if ascending...
        - change timestep back to init value
      - run sim for as many steps as needed to reach init_dt amount of time

     Of course, could probably be improved...
  */
  // If the vehicle has a downwards velocity
  if (fdm.ltp_ecef_vel.z > 0) {
    // Get the current number of timesteps until impact at current velocity
    double numDT_to_impact = (fdm.agl - vehicle_radius_max) / (fdm.curr_dt * fdm.ltp_ecef_vel.z);
    // If impact imminent within next timestep, use high sim rate
    if (numDT_to_impact <= 1.0) {
      fdm.curr_dt = min_dt;
    }
  }
  // If the vehicle is moving upwards and out of the ground, reset timestep
  else if ((fdm.ltp_ecef_vel.z <= 0) && ((fdm.agl + vehicle_radius_max) > 0)) {
    fdm.curr_dt = fdm.init_dt;
  }

  // Calculate the number of sim steps for correct amount of time elapsed
  int num_steps = int(fdm.init_dt / fdm.curr_dt);

  // Set the timestep then run sim
  FDMExec->Setdt(fdm.curr_dt);
  int i;
  for (i = 0; i < num_steps; i++) {
    FDMExec->Run();
  }

  fetch_state();

  /* Check the current state to make sure it is valid (no NaNs) */
  if (check_for_nan()) {
    printf("Error: FDM simulation encountered a total of %i NaN values at simulation time %f.\n", fdm.nan_count, fdm.time);
    printf("It is likely the simulation diverged and gave non-physical results. If you did\n");
    printf("not crash, check your model and/or initial conditions. Exiting with status 1.\n");
    exit(1);
  }

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

#if DEBUG_NPS_JSBSIM
  printf("%f,",fdm.time);
#endif

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

#if DEBUG_NPS_JSBSIM
  printf("%f,%f,%f,%f,%f,%f,",(&fg_body_ecef_accel)->Entry(1),(&fg_body_ecef_accel)->Entry(2),(&fg_body_ecef_accel)->Entry(3),fdm.body_ecef_accel.x,fdm.body_ecef_accel.y,fdm.body_ecef_accel.z);
#endif

  /* in LTP frame */
  const FGMatrix33& body_to_ltp = propagate->GetTb2l();
  const FGColumnVector3& fg_ltp_ecef_vel = body_to_ltp * fg_body_ecef_vel;
  jsbsimvec_to_vec((DoubleVect3*)&fdm.ltp_ecef_vel, &fg_ltp_ecef_vel);
  const FGColumnVector3& fg_ltp_ecef_accel = body_to_ltp * fg_body_ecef_accel;
  jsbsimvec_to_vec((DoubleVect3*)&fdm.ltp_ecef_accel, &fg_ltp_ecef_accel);

#if DEBUG_NPS_JSBSIM
  printf("%f,%f,%f,%f,%f,%f,",(&fg_ltp_ecef_accel)->Entry(1),(&fg_ltp_ecef_accel)->Entry(2),(&fg_ltp_ecef_accel)->Entry(3),fdm.ltp_ecef_accel.x,fdm.ltp_ecef_accel.y,fdm.ltp_ecef_accel.z);
#endif

  /* in ECEF frame */
  const FGMatrix33& body_to_ecef = propagate->GetTb2ec();
  const FGColumnVector3& fg_ecef_ecef_vel = body_to_ecef * fg_body_ecef_vel;
  jsbsimvec_to_vec((DoubleVect3*)&fdm.ecef_ecef_vel, &fg_ecef_ecef_vel);
  const FGColumnVector3& fg_ecef_ecef_accel = body_to_ecef * fg_body_ecef_accel;
  jsbsimvec_to_vec((DoubleVect3*)&fdm.ecef_ecef_accel, &fg_ecef_ecef_accel);

#if DEBUG_NPS_JSBSIM
  printf("%f,%f,%f,%f,%f,%f,",(&fg_ecef_ecef_accel)->Entry(1),(&fg_ecef_ecef_accel)->Entry(2),(&fg_ecef_ecef_accel)->Entry(3),fdm.ecef_ecef_accel.x,fdm.ecef_ecef_accel.y,fdm.ecef_ecef_accel.z);
#endif

  /* in LTP pprz */
  ned_of_ecef_point_d(&fdm.ltpprz_pos, &ltpdef, &fdm.ecef_pos);
  ned_of_ecef_vect_d(&fdm.ltpprz_ecef_vel, &ltpdef, &fdm.ecef_ecef_vel);
  ned_of_ecef_vect_d(&fdm.ltpprz_ecef_accel, &ltpdef, &fdm.ecef_ecef_accel);

#if DEBUG_NPS_JSBSIM
  printf("%f,%f,%f,",fdm.ltpprz_ecef_accel.z,fdm.ltpprz_ecef_accel.y,fdm.ltpprz_ecef_accel.z);
#endif

  /* llh */
  llh_from_jsbsim(&fdm.lla_pos, propagate);

  //for debug
  lla_from_jsbsim_geodetic(&fdm.lla_pos_geod, propagate);
  lla_from_jsbsim_geocentric(&fdm.lla_pos_geoc, propagate);
  lla_of_ecef_d(&fdm.lla_pos_pprz, &fdm.ecef_pos);
  fdm.agl = MetersOfFeet(propagate->GetDistanceAGL());

#if DEBUG_NPS_JSBSIM
  printf("%f\n",fdm.agl);
#endif

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
  string jsbsim_ic_name;

  sprintf(buf,"%s/conf/simulator/jsbsim/",getenv("PAPARAZZI_HOME"));
  rootdir = string(buf);

  /* if jsbsim initial conditions are defined, use them
   * otherwise use flightplan location
   */
#ifdef NPS_JSBSIM_INIT
  jsbsim_ic_name = NPS_JSBSIM_INIT;
#endif

  FDMExec = new FGFDMExec();

  FDMExec->Setsim_time(0.);
  FDMExec->Setdt(dt);

  FDMExec->DisableOutput();
  FDMExec->SetDebugLevel(0); // No DEBUG messages

  if ( ! FDMExec->LoadModel( rootdir + "aircraft",
                             rootdir + "engine",
                             rootdir + "systems",
                             NPS_JSBSIM_MODEL,
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
  if(!jsbsim_ic_name.empty()) {
    if ( ! IC->Load(jsbsim_ic_name)) {
#ifdef DEBUG
      cerr << "Initialization unsuccessful" << endl;
#endif
      delete FDMExec;
      exit(-1);
    }
  }
  else {
    // FGInitialCondition::SetAltitudeASLFtIC
    // requires this function to be called
    // before itself
    IC->SetVgroundFpsIC(0.);

    // Use flight plan initial conditions
    // convert geodetic lat from flight plan to geocentric
    double gd_lat = RadOfDeg(NAV_LAT0 / 1e7);
    double gc_lat = gc_of_gd_lat_d(gd_lat, GROUND_ALT);
    IC->SetLatitudeDegIC(DegOfRad(gc_lat));
    IC->SetLongitudeDegIC(NAV_LON0 / 1e7);

    IC->SetAltitudeASLFtIC(FeetOfMeters(GROUND_ALT + 2.0));
    IC->SetTerrainElevationFtIC(FeetOfMeters(GROUND_ALT));
    IC->SetPsiDegIC(QFU);
    IC->SetVgroundFpsIC(0.);

    //initRunning for all engines
    FDMExec->GetPropulsion()->InitRunning(-1);
    if (!FDMExec->RunIC()) {
      cerr << "Initialization from flight plan unsuccessful" << endl;
      exit(-1);
    }
  }

  // calculate vehicle max radius in m
  vehicle_radius_max = 0.01; // specify not 0.0 in case no gear
  int num_gear = FDMExec->GetGroundReactions()->GetNumGearUnits();
  int i;
  for(i = 0; i < num_gear; i++) {
    FGColumnVector3 gear_location = FDMExec->GetGroundReactions()->GetGearUnit(i)->GetBodyLocation();
    double radius = MetersOfFeet(gear_location.Magnitude());
    if (radius > vehicle_radius_max) vehicle_radius_max = radius;
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

#ifdef AHRS_H_X
#pragma message "Using magnetic field as defined in airframe file."
  fdm.ltp_h.x = AHRS_H_X;
  fdm.ltp_h.y = AHRS_H_Y;
  fdm.ltp_h.z = AHRS_H_Z;
#else
  fdm.ltp_h.x = 0.4912;
  fdm.ltp_h.y = 0.1225;
  fdm.ltp_h.z = 0.8624;
#endif

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
  fdm_lla->alt = propagate->GetAltitudeASLmeters();
  //printf("geodetic alt: %f\n", MetersOfFeet(propagate->GetGeodeticAltitude()));
  //printf("ground alt: %f\n", MetersOfFeet(propagate->GetDistanceAGL()));
  //printf("ASL alt: %f\n", propagate->GetAltitudeASLmeters());

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

#ifdef __APPLE__
/* Why isn't this there when we include math.h (on osx with clang)? */
/// Check if a double is NaN.
static int isnan(double f) { return (f != f); }
#endif

/**
 * Checks NpsFdm struct for NaNs.
 *
 * Increments the NaN count on each new NaN
 *
 * @return Count of new NaNs. 0 for no new NaNs.
 */
static int check_for_nan(void) {
  int orig_nan_count = fdm.nan_count;
  /* Check all elements for nans */
  if (isnan(fdm.ecef_pos.x)) fdm.nan_count++;
  if (isnan(fdm.ecef_pos.y)) fdm.nan_count++;
  if (isnan(fdm.ecef_pos.z)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_pos.x)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_pos.y)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_pos.z)) fdm.nan_count++;
  if (isnan(fdm.lla_pos.lon)) fdm.nan_count++;
  if (isnan(fdm.lla_pos.lat)) fdm.nan_count++;
  if (isnan(fdm.lla_pos.alt)) fdm.nan_count++;
  if (isnan(fdm.hmsl)) fdm.nan_count++;
  // Skip debugging elements
  if (isnan(fdm.ecef_ecef_vel.x)) fdm.nan_count++;
  if (isnan(fdm.ecef_ecef_vel.y)) fdm.nan_count++;
  if (isnan(fdm.ecef_ecef_vel.z)) fdm.nan_count++;
  if (isnan(fdm.ecef_ecef_accel.x)) fdm.nan_count++;
  if (isnan(fdm.ecef_ecef_accel.y)) fdm.nan_count++;
  if (isnan(fdm.ecef_ecef_accel.z)) fdm.nan_count++;
  if (isnan(fdm.body_ecef_vel.x)) fdm.nan_count++;
  if (isnan(fdm.body_ecef_vel.y)) fdm.nan_count++;
  if (isnan(fdm.body_ecef_vel.z)) fdm.nan_count++;
  if (isnan(fdm.body_ecef_accel.x)) fdm.nan_count++;
  if (isnan(fdm.body_ecef_accel.y)) fdm.nan_count++;
  if (isnan(fdm.body_ecef_accel.z)) fdm.nan_count++;
  if (isnan(fdm.ltp_ecef_vel.x)) fdm.nan_count++;
  if (isnan(fdm.ltp_ecef_vel.y)) fdm.nan_count++;
  if (isnan(fdm.ltp_ecef_vel.z)) fdm.nan_count++;
  if (isnan(fdm.ltp_ecef_accel.x)) fdm.nan_count++;
  if (isnan(fdm.ltp_ecef_accel.y)) fdm.nan_count++;
  if (isnan(fdm.ltp_ecef_accel.z)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_ecef_vel.x)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_ecef_vel.y)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_ecef_vel.z)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_ecef_accel.x)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_ecef_accel.y)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_ecef_accel.z)) fdm.nan_count++;
  if (isnan(fdm.ecef_to_body_quat.qi)) fdm.nan_count++;
  if (isnan(fdm.ecef_to_body_quat.qx)) fdm.nan_count++;
  if (isnan(fdm.ecef_to_body_quat.qy)) fdm.nan_count++;
  if (isnan(fdm.ecef_to_body_quat.qz)) fdm.nan_count++;
  if (isnan(fdm.ltp_to_body_quat.qi)) fdm.nan_count++;
  if (isnan(fdm.ltp_to_body_quat.qx)) fdm.nan_count++;
  if (isnan(fdm.ltp_to_body_quat.qy)) fdm.nan_count++;
  if (isnan(fdm.ltp_to_body_quat.qz)) fdm.nan_count++;
  if (isnan(fdm.ltp_to_body_eulers.phi)) fdm.nan_count++;
  if (isnan(fdm.ltp_to_body_eulers.theta)) fdm.nan_count++;
  if (isnan(fdm.ltp_to_body_eulers.psi)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_to_body_quat.qi)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_to_body_quat.qx)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_to_body_quat.qy)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_to_body_quat.qz)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_to_body_eulers.phi)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_to_body_eulers.theta)) fdm.nan_count++;
  if (isnan(fdm.ltpprz_to_body_eulers.psi)) fdm.nan_count++;
  if (isnan(fdm.body_ecef_rotvel.p)) fdm.nan_count++;
  if (isnan(fdm.body_ecef_rotvel.q)) fdm.nan_count++;
  if (isnan(fdm.body_ecef_rotvel.r)) fdm.nan_count++;
  if (isnan(fdm.body_ecef_rotaccel.p)) fdm.nan_count++;
  if (isnan(fdm.body_ecef_rotaccel.q)) fdm.nan_count++;
  if (isnan(fdm.body_ecef_rotaccel.r)) fdm.nan_count++;
  if (isnan(fdm.ltp_g.x)) fdm.nan_count++;
  if (isnan(fdm.ltp_g.y)) fdm.nan_count++;
  if (isnan(fdm.ltp_g.z)) fdm.nan_count++;
  if (isnan(fdm.ltp_h.x)) fdm.nan_count++;
  if (isnan(fdm.ltp_h.y)) fdm.nan_count++;
  if (isnan(fdm.ltp_h.z)) fdm.nan_count++;

  return (fdm.nan_count - orig_nan_count);
}
