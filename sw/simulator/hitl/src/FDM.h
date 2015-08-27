/*
 * Copyright (C) 2015 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
 * Utah State University, http://aggieair.usu.edu/
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
 * @file FDM.h
 *
 * HITL demo version - Flight Dynamic Model class
 * Uses JSBSim for the simulation
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef INCLUDE_FDM_H_
#define INCLUDE_FDM_H_

#include <cmath>

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

// for debug
#include <iostream>
#include <boost/format.hpp>

// paparazzi math
#include "math/pprz_geodetic.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_wmm2015.h"

// nps fdm
#include "nps_fdm.h"

// from flightplan
// NOTE: Hardcoded demo values for now
#define NAV_LAT0 418141523/* 1e7deg */
#define NAV_LON0 -1119792296/* 1e7deg */
#define NAV_ALT0 1348000/* mm above msl */
#define NAV_MSL0 641390 /* mm, EGM96 geoid-height (msl) over ellipsoid */
#define GROUND_ALT 1348.
#define QFU 90.0
#define NPS_JSBSIM_LAUNCHSPEED 18 // m/s I assue

// from airframe.h
// NOTE: Hardcoded demo values for now
#define NPS_JSBSIM_ROLL_TRIM_CMD_NORM 0
#define NPS_JSBSIM_PITCH_TRIM_CMD_NORM 0
#define NPS_JSBSIM_YAW_TRIM_CMD_NORM 0

using namespace JSBSim;
using namespace std;

#define DEBUG 1

#ifdef DegOfRad
#undef DegOfRad
#endif

#ifdef RadOfDeg
#undef RadOfDeg
#endif

/** Minimum JSBSim timestep
 * Around 1/10000 seems to be good for ground impacts
 */
#define MIN_DT (1.0/10240.0)
#define DEBUG_NPS_JSBSIM 0

/*
 * Flight Dynamic Model for JSBSim
 */
class FDM
{
private:
  /// The JSBSim executive object
  FGFDMExec *FDMExec_;
  bool initialized_;
  string rootdir_ = "jsbsim/"; // TODO: add proper links
  string NPS_JSBSIM_MODEL_ = "Malolo1";
  string jsbsim_ic_name_;

  bool high_sim_rate_;


  constexpr static const double m_pi = 3.14159265358979323846;  /* pi */

public:
  /// Holds all necessary NPS FDM state information
  struct NpsFdm fdm;
  struct LtpDef_d ltpdef;
  // Offset between ecef in geodetic and geocentric coordinates
  struct EcefCoor_d offset;
  /// The largest distance between vehicle CG and contact point
  double vehicle_radius_max;
  /// Timestep used for higher fidelity near the ground
  double min_dt;
  /// Launch variable
  bool launch;
  bool already_launched;

  timeval initial_time_;

  FDM(double dt, timeval startTime) {
    initial_time_ = startTime;

    fdm.init_dt = dt;
    fdm.curr_dt = dt;
    //Sets up the high fidelity timestep as a multiple of the normal timestep
    for (min_dt = (1.0 / dt); min_dt < (1 / MIN_DT); min_dt += (1 / dt)) {}
    min_dt = (1 / min_dt);

    fdm.nan_count = 0;

    VECT3_ASSIGN(offset, 0., 0., 0.);

    initialized_ = false;
    FDMExec_ = new FGFDMExec();
    FDMExec_->Setsim_time(0.);
    FDMExec_->Setdt(dt);
    FDMExec_->DisableOutput();
    FDMExec_->SetDebugLevel(0); // No DEBUG messages

    set_wind(0, 0, 0);

    startFDM();

    init_ltp();

    launch = false;
    already_launched = false;
    high_sim_rate_ = false;
  }

  /**
   * Initialize FDM
   */
  void startFDM() {
    // Load model
    if (! FDMExec_->LoadModel(rootdir_ + "aircraft",
                              rootdir_ + "engine",
                              rootdir_ + "systems",
                              NPS_JSBSIM_MODEL_,
                              false)) {
      cerr << "  JSBSim could not be started" << endl;
      return;
    } else {
      cout << "JSBSim model started" << endl;
    }

    //initRunning for all engines
    FDMExec_->GetPropulsion()->InitRunning(-1);

    // LLA initial coordinates (geodetic lat, geoid alt)
    struct LlaCoor_d lla0;

    // get initial conditions
    FGInitialCondition *IC = FDMExec_->GetIC();
    if (!jsbsim_ic_name_.empty()) {
      if (! IC->Load(jsbsim_ic_name_)) {
        cerr << "Initialization unsuccessful" << endl;
        return;
      }

      // load lla from JSBSim
      llh_from_jsbsim(&lla0, FDMExec_->GetPropagate());
    } else {
      // feed initial values to JSBSim
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

      // initial commands to zero
      double zeros[] = {0.0, 0.0, 0.0, 0.0};
      feed_jsbsim(zeros, 4);

      //initRunning for all engines
      FDMExec_->GetPropulsion()->InitRunning(-1);
      if (!FDMExec_->RunIC()) {
        cerr << "Initialization from flight plan unsuccessful" << endl;
        return;
      }

      // Get init time for the simulation
      initial_time_ = LogTime::getStart();

      lla0.lon = RadOfDeg(NAV_LON0 / 1e7);
      lla0.lat = gd_lat;
      lla0.alt = (double)(NAV_ALT0 + NAV_MSL0) / 1000.0;
    }

    // compute offset between geocentric and geodetic ecef
    ecef_of_lla_d(&offset, &lla0);
    struct EcefCoor_d ecef0 = {
      MetersOfFeet(FDMExec_->GetPropagate()->GetLocation().Entry(1)),
      MetersOfFeet(FDMExec_->GetPropagate()->GetLocation().Entry(2)),
      MetersOfFeet(FDMExec_->GetPropagate()->GetLocation().Entry(3))
    };
    VECT3_DIFF(offset, offset, ecef0);

    // calculate vehicle max radius in m
    vehicle_radius_max = 0.01; // specify not 0.0 in case no gear
    int num_gear = FDMExec_->GetGroundReactions()->GetNumGearUnits();
    int i;
    for (i = 0; i < num_gear; i++) {
      FGColumnVector3 gear_location = FDMExec_->GetGroundReactions()->GetGearUnit(i)->GetBodyLocation();
      double radius = MetersOfFeet(gear_location.Magnitude());
      if (radius > vehicle_radius_max) { vehicle_radius_max = radius; }
    }

    // if all went well
    initialized_ = true;
  }

  /**
   * Are we good to use FDM?
   * @return
   */
  bool isInitialized() {
    return initialized_;
  }

  /**
   * Feed JSBSim with the latest actuator commands.
   *
   * @param commands    Pointer to array of doubles holding actuator commands
   * @param commands_nb Number of commands (length of array)
   */
  void feed_jsbsim(double *commands, int commands_nb) {
    char buf[64];
#ifdef NPS_ACTUATOR_NAMES
    const char *names[] = NPS_ACTUATOR_NAMES;
    string property;
    for (int i = 0; i < commands_nb; i++) {
      sprintf(buf, "fcs/%s", names[i]);
      property = string(buf);
      FDMExec_->GetPropertyManager()->GetNode(property)->SetDouble("", commands[i]);
    }
#else
    feed_jsbsim(commands[0], commands[1], commands[2], commands[3]);
#endif /* NPS_ACTUATOR_NAMES */
  }

  /**
   * Feed JSBSim with defined command values
   * @param throttle
   * @param aileron
   * @param elevator
   * @param rudder
   */
  void feed_jsbsim(double throttle, double aileron, double elevator, double rudder) {
    FGFCS *FCS = FDMExec_->GetFCS();
    FGPropulsion *FProp = FDMExec_->GetPropulsion();

    // Set trims
    FCS->SetPitchTrimCmd(NPS_JSBSIM_PITCH_TRIM);
    FCS->SetRollTrimCmd(NPS_JSBSIM_ROLL_TRIM);
    FCS->SetYawTrimCmd(NPS_JSBSIM_YAW_TRIM);

    // Set commands
    FCS->SetDaCmd(aileron);
    FCS->SetDeCmd(elevator);
    FCS->SetDrCmd(rudder);

    for (unsigned int i = 0; i < FDMExec_->GetPropulsion()->GetNumEngines(); i++) {
      FCS->SetThrottleCmd(i, throttle);

      if (throttle > 0.01) {
        FProp->SetStarter(1);
      } else {
        FProp->SetStarter(0);
      }
    }
  }

  /**
   * Right now just runs a step, extend to it accounts for delays
   */
  bool run_step() {
    if (launch && !already_launched) {
      printf("Launching with speed of %.1f m/s!\n", (float)NPS_JSBSIM_LAUNCHSPEED);
      FDMExec_->GetIC()->SetUBodyFpsIC(FeetOfMeters(NPS_JSBSIM_LAUNCHSPEED));
      FDMExec_->RunIC();
      // Get init time for the simulation (because the time resets after running RunIC)
      initial_time_ = LogTime::getStart();
      already_launched = true;
    }


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
    high_sim_rate_ = false;
    if (fdm.ltp_ecef_vel.z > 0) {
      // Get the current number of timesteps until impact at current velocity
      double numDT_to_impact = (fdm.agl - vehicle_radius_max) / (fdm.curr_dt * fdm.ltp_ecef_vel.z);
      // If impact imminent within next timestep, use high sim rate
      if (numDT_to_impact <= 1.0) {
        high_sim_rate_ = true;
        fdm.curr_dt = min_dt;
      }
    }
    // If the vehicle is moving upwards and out of the ground, reset timestep
    else if ((fdm.ltp_ecef_vel.z <= 0) && ((fdm.agl + vehicle_radius_max) > 0)) {
      fdm.curr_dt = fdm.init_dt;
    }

    // Calculate the number of sim steps for correct amount of time elapsed
    int num_steps = int(fdm.init_dt / fdm.curr_dt);

    double realTime = LogTime::getTimeSinceStartDouble(initial_time_);
    double simTime = FDMExec_->GetPropertyManager()->GetNode("simulation/sim-time-sec")->getDoubleValue();
    double timeDiff = realTime - simTime;

    if (timeDiff < 0) {
      num_steps = 0;
      // the idea is if we are too much ahead of the time, so we want to skip a step
      // once in a while
    }

    // Set the timestep then run sim
    FDMExec_->Setdt(fdm.curr_dt);
    int i;
    for (i = 0; i < num_steps; i++) {
      FDMExec_->Run();
    }

    fetch_state();

    /* Check the current state to make sure it is valid (no NaNs) */
    if (check_for_nan()) {
      printf("Error: FDM simulation encountered a total of %i NaN values at simulation time %f.\n", fdm.nan_count, fdm.time);
      printf("It is likely the simulation diverged and gave non-physical results. If you did\n");
      printf("not crash, check your model and/or initial conditions. Exiting with status 1.\n");
      return false;
    } else {
      return true;
    }
  }

  /**
   * Fetches state for internal fdm struct
   */
  void fetch_state() {
    fdm.time = FDMExec_->GetPropertyManager()->GetNode("simulation/sim-time-sec")->getDoubleValue();

#if DEBUG_NPS_JSBSIM
    printf("%f,", fdm.time);
#endif

    FGPropagate *propagate = FDMExec_->GetPropagate();
    FGAccelerations *accelerations = FDMExec_->GetAccelerations();

    fdm.on_ground = FDMExec_->GetGroundReactions()->GetWOW();

    /*
     * position
     */
    jsbsimloc_to_loc(&fdm.ecef_pos, &propagate->GetLocation());
    fdm.hmsl = propagate->GetAltitudeASLmeters();

    /*
     * linear speed and accelerations
     */

    /* in body frame */
    const FGColumnVector3 &fg_body_ecef_vel = propagate->GetUVW();
    jsbsimvec_to_vec(&fdm.body_ecef_vel, &fg_body_ecef_vel);
    const FGColumnVector3 &fg_body_ecef_accel = accelerations->GetUVWdot();
    jsbsimvec_to_vec(&fdm.body_ecef_accel, &fg_body_ecef_accel);

#if DEBUG_NPS_JSBSIM
    printf("%f,%f,%f,%f,%f,%f,", (&fg_body_ecef_accel)->Entry(1), (&fg_body_ecef_accel)->Entry(2),
           (&fg_body_ecef_accel)->Entry(3), fdm.body_ecef_accel.x, fdm.body_ecef_accel.y, fdm.body_ecef_accel.z);
#endif

    /* in LTP frame */
    const FGMatrix33 &body_to_ltp = propagate->GetTb2l();
    const FGColumnVector3 &fg_ltp_ecef_vel = body_to_ltp * fg_body_ecef_vel;
    jsbsimvec_to_vec((DoubleVect3 *)&fdm.ltp_ecef_vel, &fg_ltp_ecef_vel);
    const FGColumnVector3 &fg_ltp_ecef_accel = body_to_ltp * fg_body_ecef_accel;
    jsbsimvec_to_vec((DoubleVect3 *)&fdm.ltp_ecef_accel, &fg_ltp_ecef_accel);

#if DEBUG_NPS_JSBSIM
    printf("%f,%f,%f,%f,%f,%f,", (&fg_ltp_ecef_accel)->Entry(1), (&fg_ltp_ecef_accel)->Entry(2),
           (&fg_ltp_ecef_accel)->Entry(3), fdm.ltp_ecef_accel.x, fdm.ltp_ecef_accel.y, fdm.ltp_ecef_accel.z);
#endif

    /* in ECEF frame */
    const FGMatrix33 &body_to_ecef = propagate->GetTb2ec();
    const FGColumnVector3 &fg_ecef_ecef_vel = body_to_ecef * fg_body_ecef_vel;
    jsbsimvec_to_vec((DoubleVect3 *)&fdm.ecef_ecef_vel, &fg_ecef_ecef_vel);
    const FGColumnVector3 &fg_ecef_ecef_accel = body_to_ecef * fg_body_ecef_accel;
    jsbsimvec_to_vec((DoubleVect3 *)&fdm.ecef_ecef_accel, &fg_ecef_ecef_accel);

#if DEBUG_NPS_JSBSIM
    printf("%f,%f,%f,%f,%f,%f,", (&fg_ecef_ecef_accel)->Entry(1), (&fg_ecef_ecef_accel)->Entry(2),
           (&fg_ecef_ecef_accel)->Entry(3), fdm.ecef_ecef_accel.x, fdm.ecef_ecef_accel.y, fdm.ecef_ecef_accel.z);
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


    /*
     * wind
     */
    const FGColumnVector3 &fg_wind_ned = FDMExec_->GetWinds()->GetTotalWindNED();
    jsbsimvec_to_vec(&fdm.wind, &fg_wind_ned);
  }

  /**
   * Get VectorNavData for the packet
   * @return VN data
   */
  VectorNavData fetch_data() {
    /*
     * Vectornav part
     */
    VectorNavData data;
    FGPropagate *propagate = FDMExec_->GetPropagate();
    FGAccelerations *accelerations = FDMExec_->GetAccelerations();

    // Timer - in nanoseconds since startup
    double sim_time;
    sim_time = FDMExec_->GetPropertyManager()->GetNode("simulation/sim-time-sec")->getDoubleValue();
    data.TimeStartup = (uint64_t)(sim_time * 1000000000.0);

    //Attitude, float, [degrees], yaw, pitch, roll, NED frame
    const FGQuaternion jsb_quat = propagate->GetQuaternion();
    data.YawPitchRoll[0] = (float)jsb_quat.GetEulerDeg(3); // yaw
    data.YawPitchRoll[1] = (float)jsb_quat.GetEulerDeg(2); // pitch
    data.YawPitchRoll[2] = (float)jsb_quat.GetEulerDeg(1); // roll

    // Rates (imu frame), float, [rad/s]
    data.AngularRate[0] = (float)fdm.body_ecef_rotvel.p;
    data.AngularRate[1] = (float)fdm.body_ecef_rotvel.q;
    data.AngularRate[2] = (float)fdm.body_ecef_rotvel.r;

    //Pos LLA, double,[beg, deg, m]
    //The estimated position given as latitude, longitude, and altitude given in [deg, deg, m] respectfully.
    data.Position[0] = propagate->GetGeodLatitudeDeg();
    data.Position[1] = propagate->GetLongitudeDeg();
    data.Position[2] = propagate->GetAltitudeASLmeters();

    //VelNed, float [m/s]
    //The estimated velocity in the North East Down (NED) frame, given in m/s.
    data.Velocity[0] = fdm.ltp_ecef_vel.x;
    data.Velocity[1] = fdm.ltp_ecef_vel.y;
    data.Velocity[2] = fdm.ltp_ecef_vel.z;

    // Accel (imu-frame), float, [m/s^-2]
    data.Accel[0] = (float)fdm.body_ecef_accel.x;
    data.Accel[1] = (float)fdm.body_ecef_accel.y;
    data.Accel[2] = (float)fdm.body_ecef_accel.z;

    // tow (in nanoseconds), uint64
    data.Tow = LogTime::getTimeOfWeek();

    //num sats, uint8
    data.NumSats = 8; // random number

    //gps fix, uint8
    data.Fix = 3; // 3D fix

    //posU, float[3] (DUMMY)
    data.PosU[0] = 0.0;
    data.PosU[1] = 0.0;
    data.PosU[2] = 0.0;

    //velU, float (DUMMY)
    data.VelU = 0.0;

    //linear acceleration imu-body frame, float [m/s^2]
    data.LinearAccelBody[0] = (float)propagate->GetVel(1);
    data.LinearAccelBody[1] = (float)propagate->GetVel(2);
    data.LinearAccelBody[2] = (float)propagate->GetVel(3);

    //YprU, float[3] (DUMMY)
    data.YprU[0] = 0.0;
    data.YprU[1] = 0.0;
    data.YprU[2] = 0.0;

    //instatus, uint16
    /* Indicates the current mode of the INS filter.
     * 0 = Not tracking. Insufficient dynamic motion to estimate attitude.
     * 1 = Sufficient dynamic motion, but solution not within performance specs.
     * 2 = INS is tracking and operating within specifications.
     */
    data.InsStatus = 0x02;

    return data;
  }

  /**
   * geocentric latitude of geodetic latitude
   * @param gd_lat
   * @param hmsl
   * @return
   */
  static double gc_of_gd_lat_d(double gd_lat, double hmsl) {
    const double a = 6378137.0;           /* earth semimajor axis in meters */
    const double f = 1. / 298.257223563;  /* reciprocal flattening          */
    const double c2 = (1. - f) * (1. - f);
    /* geocentric latitude at the planet surface */
    double ls = atan(c2 * tan(gd_lat));
    return atan2(hmsl * sin(gd_lat) + a * sin(ls), hmsl * cos(gd_lat) + a * cos(ls));
  }


  static double DegOfRad(double x) {
    return x * 180.0 / FDM::m_pi;
  }

  static double RadOfDeg(double x) {
    return x * (FDM::m_pi / 180.0);
  }

  static double MetersOfFeet(double f) {
    return (f / 3.2808399);
  }

  static double FeetOfMeters(double m) {
    return (m * 3.2808399);
  }

  /**
   * Initialize the ltp from the JSBSim location.
   *
   */
  void init_ltp(void) {

    FGPropagate *propagate = FDMExec_->GetPropagate();

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
   * Set wind
   * @param speed
   * @param dir
   * @param turbulence_severity
   */
  void set_wind(double speed, double dir, int turbulence_severity) {
    FGWinds *Winds = FDMExec_->GetWinds();
    Winds->SetWindspeed(FeetOfMeters(speed));
    Winds->SetWindPsi(dir);

    /* wind speed used for turbulence */
    Winds->SetWindspeed20ft(FeetOfMeters(speed) / 2);
    Winds->SetProbabilityOfExceedence(turbulence_severity);
  }

  /**
   * Convert JSBSim location format and struct to NPS location format and struct.
   *
   * JSBSim is in feet by default, NPS in metres
   *
   * @param fdm_location Pointer to EcefCoor_d struct
   * @param jsb_location Pointer to FGLocation struct
   */
  void jsbsimloc_to_loc(EcefCoor_d *fdm_location, const FGLocation *jsb_location) {
    fdm_location->x = FDM::MetersOfFeet(jsb_location->Entry(1));
    fdm_location->y = FDM::MetersOfFeet(jsb_location->Entry(2));
    fdm_location->z = FDM::MetersOfFeet(jsb_location->Entry(3));

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
  void jsbsimvec_to_vec(DoubleVect3 *fdm_vector, const FGColumnVector3 *jsb_vector) {
    fdm_vector->x = FDM::MetersOfFeet(jsb_vector->Entry(1));
    fdm_vector->y = FDM::MetersOfFeet(jsb_vector->Entry(2));
    fdm_vector->z = FDM::MetersOfFeet(jsb_vector->Entry(3));
  }

  /**
   * Convert JSBSim quaternion struct to NPS quaternion struct.
   *
   * @param fdm_quat    Pointer to DoubleQuat struct
   * @param jsb_quat    Pointer to FGQuaternion struct
   */
  void jsbsimquat_to_quat(DoubleQuat *fdm_quat, const FGQuaternion *jsb_quat) {
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
  void jsbsimvec_to_rate(DoubleRates *fdm_rate, const FGColumnVector3 *jsb_vector) {
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
  void llh_from_jsbsim(LlaCoor_d *fdm_lla, FGPropagate *propagate) {
    fdm_lla->lat = propagate->GetGeodLatitudeRad();
    fdm_lla->lon = propagate->GetLongitude();
    fdm_lla->alt = propagate->GetAltitudeASLmeters();
  }

  /**
   * Convert JSBSim location to NPS LLA.
   *
   * Gets geocentric latitude, longitude and geocentric radius
   *
   * @param fdm_lla   Pointer to LlaCoor_d struct
   * @param propagate Pointer to JSBSim FGPropagate object
   */
  void lla_from_jsbsim_geocentric(LlaCoor_d *fdm_lla, FGPropagate *propagate) {
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
  void lla_from_jsbsim_geodetic(LlaCoor_d *fdm_lla, FGPropagate *propagate) {
    fdm_lla->lat = propagate->GetGeodLatitudeRad();
    fdm_lla->lon = propagate->GetLongitude();
    fdm_lla->alt = MetersOfFeet(propagate->GetGeodeticAltitude());
  }

  /**
   * Checks NpsFdm struct for NaNs.
   *
   * Increments the NaN count on each new NaN
   *
   * @return Count of new NaNs. 0 for no new NaNs.
   */
  int check_for_nan(void) {
    int orig_nan_count = fdm.nan_count;
    /* Check all elements for nans */
    if (std::isnan(fdm.ecef_pos.x)) { fdm.nan_count++; }
    if (std::isnan(fdm.ecef_pos.y)) { fdm.nan_count++; }
    if (std::isnan(fdm.ecef_pos.z)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_pos.x)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_pos.y)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_pos.z)) { fdm.nan_count++; }
    if (std::isnan(fdm.lla_pos.lon)) { fdm.nan_count++; }
    if (std::isnan(fdm.lla_pos.lat)) { fdm.nan_count++; }
    if (std::isnan(fdm.lla_pos.alt)) { fdm.nan_count++; }
    if (std::isnan(fdm.hmsl)) { fdm.nan_count++; }
    // Skip debugging elements
    if (std::isnan(fdm.ecef_ecef_vel.x)) { fdm.nan_count++; }
    if (std::isnan(fdm.ecef_ecef_vel.y)) { fdm.nan_count++; }
    if (std::isnan(fdm.ecef_ecef_vel.z)) { fdm.nan_count++; }
    if (std::isnan(fdm.ecef_ecef_accel.x)) { fdm.nan_count++; }
    if (std::isnan(fdm.ecef_ecef_accel.y)) { fdm.nan_count++; }
    if (std::isnan(fdm.ecef_ecef_accel.z)) { fdm.nan_count++; }
    if (std::isnan(fdm.body_ecef_vel.x)) { fdm.nan_count++; }
    if (std::isnan(fdm.body_ecef_vel.y)) { fdm.nan_count++; }
    if (std::isnan(fdm.body_ecef_vel.z)) { fdm.nan_count++; }
    if (std::isnan(fdm.body_ecef_accel.x)) { fdm.nan_count++; }
    if (std::isnan(fdm.body_ecef_accel.y)) { fdm.nan_count++; }
    if (std::isnan(fdm.body_ecef_accel.z)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_ecef_vel.x)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_ecef_vel.y)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_ecef_vel.z)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_ecef_accel.x)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_ecef_accel.y)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_ecef_accel.z)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_ecef_vel.x)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_ecef_vel.y)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_ecef_vel.z)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_ecef_accel.x)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_ecef_accel.y)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_ecef_accel.z)) { fdm.nan_count++; }
    if (std::isnan(fdm.ecef_to_body_quat.qi)) { fdm.nan_count++; }
    if (std::isnan(fdm.ecef_to_body_quat.qx)) { fdm.nan_count++; }
    if (std::isnan(fdm.ecef_to_body_quat.qy)) { fdm.nan_count++; }
    if (std::isnan(fdm.ecef_to_body_quat.qz)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_to_body_quat.qi)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_to_body_quat.qx)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_to_body_quat.qy)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_to_body_quat.qz)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_to_body_eulers.phi)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_to_body_eulers.theta)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_to_body_eulers.psi)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_to_body_quat.qi)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_to_body_quat.qx)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_to_body_quat.qy)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_to_body_quat.qz)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_to_body_eulers.phi)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_to_body_eulers.theta)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltpprz_to_body_eulers.psi)) { fdm.nan_count++; }
    if (std::isnan(fdm.body_ecef_rotvel.p)) { fdm.nan_count++; }
    if (std::isnan(fdm.body_ecef_rotvel.q)) { fdm.nan_count++; }
    if (std::isnan(fdm.body_ecef_rotvel.r)) { fdm.nan_count++; }
    if (std::isnan(fdm.body_ecef_rotaccel.p)) { fdm.nan_count++; }
    if (std::isnan(fdm.body_ecef_rotaccel.q)) { fdm.nan_count++; }
    if (std::isnan(fdm.body_ecef_rotaccel.r)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_g.x)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_g.y)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_g.z)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_h.x)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_h.y)) { fdm.nan_count++; }
    if (std::isnan(fdm.ltp_h.z)) { fdm.nan_count++; }

    return (fdm.nan_count - orig_nan_count);
  }
};



#endif /* INCLUDE_FDM_H_ */
