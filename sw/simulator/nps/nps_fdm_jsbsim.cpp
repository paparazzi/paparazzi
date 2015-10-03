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

// ignore stupid warnings in JSBSim
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGPropulsion.h>
#include <models/FGGroundReactions.h>
#include <models/FGAccelerations.h>
#include <models/FGFCS.h>
#include <models/atmosphere/FGWinds.h>

// Thrusters
#include <models/propulsion/FGThruster.h>
#include <models/propulsion/FGPropeller.h>

// end ignore unused param warnings in JSBSim
#pragma GCC diagnostic pop


#include "nps_fdm.h"
#include "math/pprz_geodetic.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"

#include "math/pprz_geodetic_wmm2015.h"

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

/**
 * Trim values for the airframe
 */
#ifndef NPS_JSBSIM_PITCH_TRIM
#define NPS_JSBSIM_PITCH_TRIM 0.0
#endif

#ifndef NPS_JSBSIM_ROLL_TRIM
#define NPS_JSBSIM_ROLL_TRIM 0.0
#endif

#ifndef NPS_JSBSIM_YAW_TRIM
#define NPS_JSBSIM_YAW_TRIM 0.0
#endif

/**
 * Control surface deflections for visualisation
 */
#define DEG2RAD 0.017

#ifndef NPS_JSBSIM_ELEVATOR_MAX_RAD
#define NPS_JSBSIM_ELEVATOR_MAX_RAD (20.0*DEG2RAD)
#endif

#ifndef NPS_JSBSIM_AILERON_MAX_RAD
#define NPS_JSBSIM_AILERON_MAX_RAD (20.0*DEG2RAD)
#endif

#ifndef NPS_JSBSIM_RUDDER_MAX_RAD
#define NPS_JSBSIM_RUDDER_MAX_RAD (20.0*DEG2RAD)
#endif

#ifndef NPS_JSBSIM_FLAP_MAX_RAD
#define NPS_JSBSIM_FLAP_MAX_RAD (20.0*DEG2RAD)
#endif

/** Minimum JSBSim timestep
 * Around 1/10000 seems to be good for ground impacts
 */
#define MIN_DT (1.0/10240.0)

using namespace JSBSim;
using namespace std;

static void feed_jsbsim(double *commands, int commands_nb);
static void feed_jsbsim(double throttle, double aileron, double elevator, double rudder);
static void fetch_state(void);
static int check_for_nan(void);

static void jsbsimvec_to_vec(DoubleVect3 *fdm_vector, const FGColumnVector3 *jsb_vector);
static void jsbsimloc_to_loc(EcefCoor_d *fdm_location, const FGLocation *jsb_location);
static void jsbsimquat_to_quat(DoubleQuat *fdm_quat, const FGQuaternion *jsb_quat);
static void jsbsimvec_to_rate(DoubleRates *fdm_rate, const FGColumnVector3 *jsb_vector);
static void llh_from_jsbsim(LlaCoor_d *fdm_lla, FGPropagate *propagate);
static void lla_from_jsbsim_geodetic(LlaCoor_d *fdm_lla, FGPropagate *propagate);
static void lla_from_jsbsim_geocentric(LlaCoor_d *fdm_lla, FGPropagate *propagate);

static void init_jsbsim(double dt);
static void init_ltp(void);

/// Holds all necessary NPS FDM state information
struct NpsFdm fdm;

/// The JSBSim executive object
static FGFDMExec *FDMExec;

static struct LtpDef_d ltpdef;

// Offset between ecef in geodetic and geocentric coordinates
static struct EcefCoor_d offset;

/// The largest distance between vehicle CG and contact point
double vehicle_radius_max;

/// Timestep used for higher fidelity near the ground
double min_dt;

void nps_fdm_init(double dt)
{

  fdm.init_dt = dt;
  fdm.curr_dt = dt;
  //Sets up the high fidelity timestep as a multiple of the normal timestep
  for (min_dt = (1.0 / dt); min_dt < (1 / MIN_DT); min_dt += (1 / dt)) {}
  min_dt = (1 / min_dt);

  fdm.nan_count = 0;

  VECT3_ASSIGN(offset, 0., 0., 0.);

  init_jsbsim(dt);

  FDMExec->RunIC();

  init_ltp();

#if DEBUG_NPS_JSBSIM
  printf("fdm.time,fdm.body_ecef_accel.x,fdm.body_ecef_accel.y,fdm.body_ecef_accel.z,fdm.ltp_ecef_accel.x,fdm.ltp_ecef_accel.y,fdm.ltp_ecef_accel.z,fdm.ecef_ecef_accel.x,fdm.ecef_ecef_accel.y,fdm.ecef_ecef_accel.z,fdm.ltpprz_ecef_accel.z,fdm.ltpprz_ecef_accel.y,fdm.ltpprz_ecef_accel.z,fdm.agl\n");
#endif

  fetch_state();

}

void nps_fdm_run_step(bool_t launch __attribute__((unused)), double *commands, int commands_nb)
{

#ifdef NPS_JSBSIM_LAUNCHSPEED
  static bool_t already_launched = FALSE;

  if (launch && !already_launched) {
    printf("Launching with speed of %.1f m/s!\n", (float)NPS_JSBSIM_LAUNCHSPEED);
    FDMExec->GetIC()->SetUBodyFpsIC(FeetOfMeters(NPS_JSBSIM_LAUNCHSPEED));
    FDMExec->RunIC();
    already_launched = TRUE;
  }
#endif

  feed_jsbsim(commands, commands_nb);

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

void nps_fdm_set_wind(double speed, double dir)
{
  FGWinds *Winds = FDMExec->GetWinds();
  Winds->SetWindspeed(FeetOfMeters(speed));
  Winds->SetWindPsi(dir);
}

void nps_fdm_set_wind_ned(double wind_north, double wind_east, double wind_down)
{
  FGWinds *Winds = FDMExec->GetWinds();
  Winds->SetWindNED(FeetOfMeters(wind_north), FeetOfMeters(wind_east),
                    FeetOfMeters(wind_down));
}

void nps_fdm_set_turbulence(double wind_speed, int turbulence_severity)
{
  FGWinds *Winds = FDMExec->GetWinds();
  /* wind speed used for turbulence */
  Winds->SetWindspeed20ft(FeetOfMeters(wind_speed) / 2);
  Winds->SetProbabilityOfExceedence(turbulence_severity);
}

/**
 * Feed JSBSim with the latest actuator commands.
 *
 * @param commands    Pointer to array of doubles holding actuator commands
 * @param commands_nb Number of commands (length of array)
 */
static void feed_jsbsim(double *commands, int commands_nb)
{
#ifdef NPS_ACTUATOR_NAMES
  char buf[64];
  const char *names[] = NPS_ACTUATOR_NAMES;
  string property;

  int i;
  for (i = 0; i < commands_nb; i++) {
    sprintf(buf, "fcs/%s", names[i]);
    property = string(buf);
    FDMExec->GetPropertyManager()->GetNode(property)->SetDouble("", commands[i]);
  }
#else
  if (commands_nb != 4) {
    cerr << "commands_nb must be 4!" << endl;
    exit(-1);
  }
  /* call version that directly feeds throttle, aileron, elevator, rudder */
  feed_jsbsim(commands[COMMAND_THROTTLE], commands[COMMAND_ROLL], commands[COMMAND_PITCH], commands[3]);
#endif
}

static void feed_jsbsim(double throttle, double aileron, double elevator, double rudder)
{
  FGFCS *FCS = FDMExec->GetFCS();
  FGPropulsion *FProp = FDMExec->GetPropulsion();

  // Set trims
  FCS->SetPitchTrimCmd(NPS_JSBSIM_PITCH_TRIM);
  FCS->SetRollTrimCmd(NPS_JSBSIM_ROLL_TRIM);
  FCS->SetYawTrimCmd(NPS_JSBSIM_YAW_TRIM);

  // Set commands
  FCS->SetDaCmd(aileron);
  FCS->SetDeCmd(elevator);
  FCS->SetDrCmd(rudder);

  for (unsigned int i = 0; i < FDMExec->GetPropulsion()->GetNumEngines(); i++) {
    FCS->SetThrottleCmd(i, throttle);

    if (throttle > 0.01) {
      FProp->SetStarter(1);
    } else {
      FProp->SetStarter(0);
    }
  }
}


/**
 * Populates the NPS fdm struct after a simulation step.
 */
static void fetch_state(void)
{

  fdm.time = FDMExec->GetPropertyManager()->GetNode("simulation/sim-time-sec")->getDoubleValue();

#if DEBUG_NPS_JSBSIM
  printf("%f,", fdm.time);
#endif

  FGPropagate *propagate = FDMExec->GetPropagate();
  FGAccelerations *accelerations = FDMExec->GetAccelerations();

  fdm.on_ground = FDMExec->GetGroundReactions()->GetWOW();

  /*
   * position
   */
  jsbsimloc_to_loc(&fdm.ecef_pos, &propagate->GetLocation());
  fdm.hmsl = propagate->GetAltitudeASLmeters();

  /*
   * linear speed and accelerations
   */

  /* in body frame */
  jsbsimvec_to_vec(&fdm.body_ecef_vel, &propagate->GetUVW());
  jsbsimvec_to_vec(&fdm.body_ecef_accel, &accelerations->GetUVWdot());
  jsbsimvec_to_vec(&fdm.body_inertial_accel, &accelerations->GetUVWidot());
  jsbsimvec_to_vec(&fdm.body_accel, &accelerations->GetBodyAccel());

#if DEBUG_NPS_JSBSIM
  printf("%f,%f,%f,", fdm.body_ecef_accel.x, fdm.body_ecef_accel.y, fdm.body_ecef_accel.z);
#endif

  /* in LTP frame */
  jsbsimvec_to_vec((DoubleVect3 *)&fdm.ltp_ecef_vel, &propagate->GetVel());
  const FGColumnVector3 &fg_ltp_ecef_accel = propagate->GetTb2l() * accelerations->GetUVWdot();
  jsbsimvec_to_vec((DoubleVect3 *)&fdm.ltp_ecef_accel, &fg_ltp_ecef_accel);

#if DEBUG_NPS_JSBSIM
  printf("%f,%f,%f,", fdm.ltp_ecef_accel.x, fdm.ltp_ecef_accel.y, fdm.ltp_ecef_accel.z);
#endif

  /* in ECEF frame */
  const FGColumnVector3 &fg_ecef_ecef_vel = propagate->GetECEFVelocity();
  jsbsimvec_to_vec((DoubleVect3 *)&fdm.ecef_ecef_vel, &fg_ecef_ecef_vel);

  const FGColumnVector3 &fg_ecef_ecef_accel = propagate->GetTb2ec() * accelerations->GetUVWdot();
  jsbsimvec_to_vec((DoubleVect3 *)&fdm.ecef_ecef_accel, &fg_ecef_ecef_accel);

#if DEBUG_NPS_JSBSIM
  printf("%f,%f,%f,", fdm.ecef_ecef_accel.x, fdm.ecef_ecef_accel.y, fdm.ecef_ecef_accel.z);
#endif

  /* in LTP pprz */
  ned_of_ecef_point_d(&fdm.ltpprz_pos, &ltpdef, &fdm.ecef_pos);
  ned_of_ecef_vect_d(&fdm.ltpprz_ecef_vel, &ltpdef, &fdm.ecef_ecef_vel);
  ned_of_ecef_vect_d(&fdm.ltpprz_ecef_accel, &ltpdef, &fdm.ecef_ecef_accel);

#if DEBUG_NPS_JSBSIM
  printf("%f,%f,%f,", fdm.ltpprz_ecef_accel.z, fdm.ltpprz_ecef_accel.y, fdm.ltpprz_ecef_accel.z);
#endif

  /* llh */
  llh_from_jsbsim(&fdm.lla_pos, propagate);

  //for debug
  lla_from_jsbsim_geodetic(&fdm.lla_pos_geod, propagate);
  lla_from_jsbsim_geocentric(&fdm.lla_pos_geoc, propagate);
  lla_of_ecef_d(&fdm.lla_pos_pprz, &fdm.ecef_pos);
  fdm.agl = MetersOfFeet(propagate->GetDistanceAGL());

#if DEBUG_NPS_JSBSIM
  printf("%f\n", fdm.agl);
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
  jsbsimvec_to_rate(&fdm.body_ecef_rotvel, &propagate->GetPQR());
  jsbsimvec_to_rate(&fdm.body_ecef_rotaccel, &accelerations->GetPQRdot());

  jsbsimvec_to_rate(&fdm.body_inertial_rotvel, &propagate->GetPQRi());
  jsbsimvec_to_rate(&fdm.body_inertial_rotaccel, &accelerations->GetPQRidot());


  /*
   * wind
   */
  const FGColumnVector3 &fg_wind_ned = FDMExec->GetWinds()->GetTotalWindNED();
  jsbsimvec_to_vec(&fdm.wind, &fg_wind_ned);

  /*
   * Control surface positions
   *
   */
  fdm.rudder = (FDMExec->GetPropertyManager()->GetNode("fcs/rudder-pos-rad")->getDoubleValue()) /
               NPS_JSBSIM_RUDDER_MAX_RAD;
  fdm.left_aileron = (-1 * FDMExec->GetPropertyManager()->GetNode("fcs/left-aileron-pos-rad")->getDoubleValue()) /
                     NPS_JSBSIM_AILERON_MAX_RAD;
  fdm.right_aileron = (FDMExec->GetPropertyManager()->GetNode("fcs/right-aileron-pos-rad")->getDoubleValue()) /
                      NPS_JSBSIM_AILERON_MAX_RAD;
  fdm.elevator = (FDMExec->GetPropertyManager()->GetNode("fcs/elevator-pos-rad")->getDoubleValue()) /
                 NPS_JSBSIM_ELEVATOR_MAX_RAD;
  fdm.flap = (FDMExec->GetPropertyManager()->GetNode("fcs/flap-pos-rad")->getDoubleValue()) / NPS_JSBSIM_FLAP_MAX_RAD;

  /*
   * Propulsion
   */
  FGPropulsion *FGProp =  FDMExec->GetPropulsion();
  fdm.num_engines = FGProp->GetNumEngines();

  /*
   * Note that JSBSim for some reason has very high momentum for the propeller
   * (even when the moment of inertia of the propeller has the right value)
   * As a result after switching the motor off
   */
  for (uint32_t k = 0; k < fdm.num_engines; k++) {
    FGEngine *FGEng = FGProp->GetEngine(k);
    FGThruster *FGThrst = FGEng->GetThruster();
    fdm.eng_state[k] = FGEng->GetStarter();
    fdm.rpm[k] = (float) FGThrst->GetRPM();
    //printf("RPM: %f\n", fdm.rpm[k]);
    //printf("STATE: %u\n", fdm.eng_state[k]);
  }
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
static void init_jsbsim(double dt)
{

  char buf[1024];
  string rootdir;
  string jsbsim_ic_name;

  sprintf(buf, "%s/conf/simulator/jsbsim/", getenv("PAPARAZZI_HOME"));
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

  if (! FDMExec->LoadModel(rootdir + "aircraft",
                           rootdir + "engine",
                           rootdir + "systems",
                           NPS_JSBSIM_MODEL,
                           false)) {
#ifdef DEBUG
    cerr << "  JSBSim could not be started" << endl << endl;
#endif
    delete FDMExec;
    exit(-1);
  }

#ifdef DEBUG
  cerr << "NumEngines: " << FDMExec->GetPropulsion()->GetNumEngines() << endl;
  cerr << "NumGearUnits: " << FDMExec->GetGroundReactions()->GetNumGearUnits() << endl;
#endif

  // LLA initial coordinates (geodetic lat, geoid alt)
  struct LlaCoor_d lla0;

  FGInitialCondition *IC = FDMExec->GetIC();
  if (!jsbsim_ic_name.empty()) {
    if (! IC->Load(jsbsim_ic_name)) {
#ifdef DEBUG
      cerr << "Initialization unsuccessful" << endl;
#endif
      delete FDMExec;
      exit(-1);
    }

    llh_from_jsbsim(&lla0, FDMExec->GetPropagate());
    cout << "JSBSim initial conditions loaded from " << jsbsim_ic_name << endl;
  } else {
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

    IC->SetWindNEDFpsIC(0.0, 0.0, 0.0);
    IC->SetAltitudeASLFtIC(FeetOfMeters(GROUND_ALT + 2.0));
    IC->SetTerrainElevationFtIC(FeetOfMeters(GROUND_ALT));
    IC->SetPsiDegIC(QFU);
    IC->SetVgroundFpsIC(0.);

    lla0.lon = RadOfDeg(NAV_LON0 / 1e7);
    lla0.lat = gd_lat;
    lla0.alt = (double)(NAV_ALT0 + NAV_MSL0) / 1000.0;
  }

  // initial commands to zero
  feed_jsbsim(0.0, 0.0, 0.0, 0.0);

  //loop JSBSim once w/o integrating
  if (!FDMExec->RunIC()) {
    cerr << "Initialization unsuccessful" << endl;
    exit(-1);
  }

  //initRunning for all engines
  FDMExec->GetPropulsion()->InitRunning(-1);


  // compute offset between geocentric and geodetic ecef
  ecef_of_lla_d(&offset, &lla0);
  struct EcefCoor_d ecef0 = {
    MetersOfFeet(FDMExec->GetPropagate()->GetLocation().Entry(1)),
    MetersOfFeet(FDMExec->GetPropagate()->GetLocation().Entry(2)),
    MetersOfFeet(FDMExec->GetPropagate()->GetLocation().Entry(3))
  };
  VECT3_DIFF(offset, offset, ecef0);

  // calculate vehicle max radius in m
  vehicle_radius_max = 0.01; // specify not 0.0 in case no gear
  int num_gear = FDMExec->GetGroundReactions()->GetNumGearUnits();
  int i;
  for (i = 0; i < num_gear; i++) {
    FGColumnVector3 gear_location = FDMExec->GetGroundReactions()->GetGearUnit(i)->GetBodyLocation();
    double radius = MetersOfFeet(gear_location.Magnitude());
    if (radius > vehicle_radius_max) { vehicle_radius_max = radius; }
  }

}

/**
 * Initialize the ltp from the JSBSim location.
 *
 */
static void init_ltp(void)
{

  FGPropagate *propagate = FDMExec->GetPropagate();

  jsbsimloc_to_loc(&fdm.ecef_pos, &propagate->GetLocation());
  ltp_def_from_ecef_d(&ltpdef, &fdm.ecef_pos);

  fdm.ltp_g.x = 0.;
  fdm.ltp_g.y = 0.;
  fdm.ltp_g.z = 9.81;


#if !NPS_CALC_GEO_MAG && defined(AHRS_H_X)
  PRINT_CONFIG_MSG("Using magnetic field as defined in airframe file (AHRS section).")
  fdm.ltp_h.x = AHRS_H_X;
  fdm.ltp_h.y = AHRS_H_Y;
  fdm.ltp_h.z = AHRS_H_Z;
#elif !NPS_CALC_GEO_MAG && defined(INS_H_X)
  PRINT_CONFIG_MSG("Using magnetic field as defined in airframe file (INS section).")
  fdm.ltp_h.x = INS_H_X;
  fdm.ltp_h.y = INS_H_Y;
  fdm.ltp_h.z = INS_H_Z;
#else
  PRINT_CONFIG_MSG("Using WMM2010 model to calculate magnetic field at simulated location.")
  /* calculation of magnetic field according to WMM2010 model */
  double gha[MAXCOEFF];

  /* Current date in decimal year, for example 2012.68 */
  /** @FIXME properly get current time */
  double sdate = 2014.5;

  llh_from_jsbsim(&fdm.lla_pos, propagate);
  /* LLA Position in decimal degrees and altitude in km */
  double latitude = DegOfRad(fdm.lla_pos.lat);
  double longitude = DegOfRad(fdm.lla_pos.lon);
  double alt = fdm.lla_pos.alt / 1e3;

  // Calculates additional coeffs
  int32_t nmax = extrapsh(sdate, GEO_EPOCH, NMAX_1, NMAX_2, gha);
  // Calculates absolute magnetic field
  mag_calc(1, latitude, longitude, alt, nmax, gha,
           &fdm.ltp_h.x, &fdm.ltp_h.y, &fdm.ltp_h.z,
           IEXT, EXT_COEFF1, EXT_COEFF2, EXT_COEFF3);
  double_vect3_normalize(&fdm.ltp_h);
  printf("normalized magnetic field: %.4f %.4f %.4f\n", fdm.ltp_h.x, fdm.ltp_h.y, fdm.ltp_h.z);
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
static void jsbsimloc_to_loc(EcefCoor_d *fdm_location, const FGLocation *jsb_location)
{

  fdm_location->x = MetersOfFeet(jsb_location->Entry(1));
  fdm_location->y = MetersOfFeet(jsb_location->Entry(2));
  fdm_location->z = MetersOfFeet(jsb_location->Entry(3));

  VECT3_ADD(*fdm_location, offset);
}

/**
 * Convert JSBSim vector format and struct to NPS vector format and struct.
 *
 * JSBSim is in feet by default, NPS in metres
 *
 * @param fdm_vector    Pointer to DoubleVect3 struct
 * @param jsb_vector    Pointer to FGColumnVector3 struct
 */
static void jsbsimvec_to_vec(DoubleVect3 *fdm_vector, const FGColumnVector3 *jsb_vector)
{

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
static void jsbsimquat_to_quat(DoubleQuat *fdm_quat, const FGQuaternion *jsb_quat)
{

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
static void jsbsimvec_to_rate(DoubleRates *fdm_rate, const FGColumnVector3 *jsb_vector)
{

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
void llh_from_jsbsim(LlaCoor_d *fdm_lla, FGPropagate *propagate)
{

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
void lla_from_jsbsim_geocentric(LlaCoor_d *fdm_lla, FGPropagate *propagate)
{

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
void lla_from_jsbsim_geodetic(LlaCoor_d *fdm_lla, FGPropagate *propagate)
{

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
static int check_for_nan(void)
{
  int orig_nan_count = fdm.nan_count;
  /* Check all elements for nans */
  if (isnan(fdm.ecef_pos.x)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_pos.y)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_pos.z)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_pos.x)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_pos.y)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_pos.z)) { fdm.nan_count++; }
  if (isnan(fdm.lla_pos.lon)) { fdm.nan_count++; }
  if (isnan(fdm.lla_pos.lat)) { fdm.nan_count++; }
  if (isnan(fdm.lla_pos.alt)) { fdm.nan_count++; }
  if (isnan(fdm.hmsl)) { fdm.nan_count++; }
  // Skip debugging elements
  if (isnan(fdm.ecef_ecef_vel.x)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_ecef_vel.y)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_ecef_vel.z)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_ecef_accel.x)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_ecef_accel.y)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_ecef_accel.z)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_vel.x)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_vel.y)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_vel.z)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_accel.x)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_accel.y)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_accel.z)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_ecef_vel.x)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_ecef_vel.y)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_ecef_vel.z)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_ecef_accel.x)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_ecef_accel.y)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_ecef_accel.z)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_ecef_vel.x)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_ecef_vel.y)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_ecef_vel.z)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_ecef_accel.x)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_ecef_accel.y)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_ecef_accel.z)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_to_body_quat.qi)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_to_body_quat.qx)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_to_body_quat.qy)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_to_body_quat.qz)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_to_body_quat.qi)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_to_body_quat.qx)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_to_body_quat.qy)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_to_body_quat.qz)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_to_body_eulers.phi)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_to_body_eulers.theta)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_to_body_eulers.psi)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_to_body_quat.qi)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_to_body_quat.qx)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_to_body_quat.qy)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_to_body_quat.qz)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_to_body_eulers.phi)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_to_body_eulers.theta)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_to_body_eulers.psi)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_rotvel.p)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_rotvel.q)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_rotvel.r)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_rotaccel.p)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_rotaccel.q)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_rotaccel.r)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_g.x)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_g.y)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_g.z)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_h.x)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_h.y)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_h.z)) { fdm.nan_count++; }

  return (fdm.nan_count - orig_nan_count);
}
