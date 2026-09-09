/*
 * Copyright (C) 2017 Tom van Dijk, Kirk Scheper
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
 * @file nps_fdm_gazebo.cpp
 * Flight Dynamics Model (FDM) for NPS using Gazebo.
 *
 * This is an FDM for NPS that uses Gazebo as the simulation engine.
 */

#include <cstdio>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sys/time.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>

#include <gazebo/gazebo_config.h>
#if GAZEBO_MAJOR_VERSION < 8
#error "Paparazzi only supports gazebo versions > 7, please upgrade to a more recent version of Gazebo"
#endif

extern "C" {
#include "nps_fdm.h"
#include "nps_autopilot.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "autopilot.h"
#include "modules/core/abi.h"

#include "math/pprz_isa.h"
#include "math/pprz_algebra_double.h"
#include "filters/low_pass_filter.h"
#include "filters/high_pass_filter.h"

#include "modules/actuators/motor_mixing_types.h"

#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_double.h"
#include "state.h"
}

#if defined(NPS_DEBUG_VIDEO)
// Opencv tools
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

using namespace std;

#ifndef NPS_GAZEBO_WORLD
#define NPS_GAZEBO_WORLD "empty.world"
#endif
#ifndef NPS_GAZEBO_AC_NAME
#define NPS_GAZEBO_AC_NAME "ardrone"
#endif

// Add video handling functions if req'd.
#if NPS_SIMULATE_VIDEO
extern "C" {
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/video_thread_nps.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "mcu_periph/sys_time.h"
#include "boards/bebop/mt9f002.h"
#include "boards/bebop/mt9v117.h"
struct mt9f002_t mt9f002 __attribute__((weak)); // Prevent undefined reference errors when Bebop code is not linked.
}

static void init_gazebo_video(void);
static void gazebo_read_video(void);
static void read_image(
  struct image_t *img,
  gazebo::sensors::CameraSensorPtr cam);
struct gazebocam_t {
  gazebo::sensors::CameraSensorPtr cam;
  gazebo::common::Time last_measurement_time;
};
static struct gazebocam_t gazebo_cams[VIDEO_THREAD_MAX_CAMERAS] =
{ { NULL, 0 } };

// Reduce resolution of the simulated MT9F002 sensor (Bebop) to improve runtime
// performance at the cost of image resolution.
// Recommended values: 1 (realistic), 2, 4 (fast but slightly blurry)
#ifndef NPS_MT9F002_SENSOR_RES_DIVIDER
#define NPS_MT9F002_SENSOR_RES_DIVIDER 1
#endif
#endif // NPS_SIMULATE_VIDEO

struct gazebo_actuators_t {
  string names[NPS_COMMANDS_NB];
  double thrusts[NPS_COMMANDS_NB];
  double torques[NPS_COMMANDS_NB];
  struct FirstOrderLowPass lowpass[NPS_COMMANDS_NB];
  struct FirstOrderHighPass highpass[NPS_COMMANDS_NB];
  double max_ang_momentum[NPS_COMMANDS_NB];
};

struct gazebo_actuators_t gazebo_actuators = { NPS_ACTUATOR_NAMES, NPS_ACTUATOR_THRUSTS, NPS_ACTUATOR_TORQUES, { }, { }, { }};

#if NPS_SIMULATE_LASER_RANGE_ARRAY
extern "C" {
#include "modules/core/abi.h"
}

static void gazebo_init_range_sensors(void);
static void gazebo_read_range_sensors(void);

#endif

std::shared_ptr<gazebo::sensors::SonarSensor> sonar = NULL;

/// Holds all necessary NPS FDM state information
struct NpsFdm fdm;

// Pointer to Gazebo data
static bool gazebo_initialized = false;
static gazebo::physics::ModelPtr model = NULL;

// Get contact sensor
static gazebo::sensors::ContactSensorPtr ct;

// Helper functions
static void init_gazebo(void);
static void gazebo_read(void);
static void gazebo_write(double act_commands[], int commands_nb);

// Conversion routines
inline struct EcefCoor_d to_pprz_ecef(ignition::math::Vector3d ecef_i)
{
  struct EcefCoor_d ecef_p;
  ecef_p.x = ecef_i.X();
  ecef_p.y = ecef_i.Y();
  ecef_p.z = ecef_i.Z();
  return ecef_p;
}

inline struct NedCoor_d to_pprz_ned(ignition::math::Vector3d global)
{
  struct NedCoor_d ned;
  ned.x = global.Y();
  ned.y = global.X();
  ned.z = -global.Z();
  return ned;
}

inline struct LlaCoor_d to_pprz_lla(ignition::math::Vector3d lla_i)
{
  struct LlaCoor_d lla_p;
  lla_p.lat = lla_i.X();
  lla_p.lon = lla_i.Y();
  lla_p.alt = lla_i.Z();
  return lla_p;
}

inline struct DoubleVect3 to_pprz_body(ignition::math::Vector3d body_g)
{
  struct DoubleVect3 body_p;
  body_p.x = body_g.X();
  body_p.y = -body_g.Y();
  body_p.z = -body_g.Z();
  return body_p;
}

inline struct DoubleRates to_pprz_rates(ignition::math::Vector3d body_g)
{
  struct DoubleRates body_p;
  body_p.p = body_g.X();
  body_p.q = -body_g.Y();
  body_p.r = -body_g.Z();
  return body_p;
}

inline struct DoubleEulers to_pprz_eulers(ignition::math::Quaterniond quat)
{
  struct DoubleEulers eulers;
  eulers.psi = -quat.Yaw();
  eulers.theta = -quat.Pitch();
  eulers.phi = quat.Roll();
  return eulers;
}

inline struct DoubleEulers to_global_pprz_eulers(ignition::math::Quaterniond quat)
{
  struct DoubleEulers eulers;
  eulers.psi = -quat.Yaw() - M_PI / 2;
  eulers.theta = -quat.Pitch();
  eulers.phi = quat.Roll();
  return eulers;
}

inline struct DoubleVect3 to_pprz_ltp(ignition::math::Vector3d xyz)
{
  struct DoubleVect3 ltp;
  ltp.x = xyz.Y();
  ltp.y = xyz.X();
  ltp.z = -xyz.Z();
  return ltp;
}

// External functions, interface with Paparazzi's NPS as declared in nps_fdm.h

/**
 * Initialize actuator dynamics, set unused fields in fdm
 * @param dt
 */
void nps_fdm_init(double dt)
{
  fdm.init_dt = dt; // JSBsim specific
  fdm.curr_dt = dt; // JSBsim specific
  fdm.nan_count = 0; // JSBsim specific

#ifdef NPS_ACTUATOR_TIME_CONSTANTS
  // Set up low-pass filter to simulate delayed actuator response
  const float tau[NPS_COMMANDS_NB] = NPS_ACTUATOR_TIME_CONSTANTS;
  for (int i = 0; i < NPS_COMMANDS_NB; i++) {
    init_first_order_low_pass(&gazebo_actuators.lowpass[i], tau[i], dt, 0.f);
  }
#ifdef NPS_ACTUATOR_MAX_ANGULAR_MOMENTUM
  // Set up high-pass filter to simulate spinup torque
  const float Iwmax[NPS_COMMANDS_NB] = NPS_ACTUATOR_MAX_ANGULAR_MOMENTUM;
  for (int i = 0; i < NPS_COMMANDS_NB; i++) {
    init_first_order_high_pass(&gazebo_actuators.highpass[i], tau[i], dt, 0.f);
    gazebo_actuators.max_ang_momentum[i] = Iwmax[i];
  }
#endif
#endif
}

/**
 * Update the simulation state.
 * @param launch
 * @param act_commands
 * @param commands_nb
 */
void nps_fdm_run_step(
  bool launch __attribute__((unused)),
  double *act_commands,
  int commands_nb)
{
  // Initialize Gazebo if req'd.
  // Initialization is peformed here instead of in nps_fdm_init because:
  // - Video devices need to added at this point. Video devices have not been
  //   added yet when nps_fdm_init is called.
  // - nps_fdm_init runs on a different thread then nps_fdm_run_step, which
  //   causes problems with Gazebo.
  if (!gazebo_initialized) {
    init_gazebo();
    gazebo_read();
#if NPS_SIMULATE_VIDEO
    init_gazebo_video();
#endif
#if NPS_SIMULATE_LASER_RANGE_ARRAY
    gazebo_init_range_sensors();
#endif
    gazebo_initialized = true;
  }

  // Update the simulation for a single timestep.
  gazebo::runWorld(model->GetWorld(), 1);
  gazebo::sensors::run_once();
  gazebo_write(act_commands, commands_nb);
  gazebo_read();
#if NPS_SIMULATE_VIDEO
  gazebo_read_video();
#endif
#if NPS_SIMULATE_LASER_RANGE_ARRAY
  gazebo_read_range_sensors();
#endif

}

// TODO Atmosphere functions have not been implemented yet.
// Starting at version 8, Gazebo has its own atmosphere and wind model.
void nps_fdm_set_wind(
  double speed __attribute__((unused)),
  double dir __attribute__((unused)))
{
}

void nps_fdm_set_wind_ned(
  double wind_north __attribute__((unused)),
  double wind_east __attribute__((unused)),
  double wind_down __attribute__((unused)))
{
}

void nps_fdm_set_turbulence(
  double wind_speed __attribute__((unused)),
  int turbulence_severity __attribute__((unused)))
{
}

/** Set temperature in degrees Celcius at given height h above MSL */
void nps_fdm_set_temperature(
  double temp __attribute__((unused)),
  double h __attribute__((unused)))
{
}

// Internal functions
/**
 * Set up a Gazebo server.
 *
 * Starts a Gazebo server, adds conf/simulator/gazebo/models to its model path
 * and loads the world specified by NPS_GAZEBO_WORLD.
 *
 * This function also obtaines a pointer to the aircraft model, named
 * NPS_GAZEBO_AC_NAME ('paparazzi_uav' by default). This pointer, 'model',
 * is used to read the state and write actuator commands in gazebo_read and
 * _write.
 */
static void init_gazebo(void)
{
  string gazebo_home = "/conf/simulator/gazebo/";
  string pprz_home(getenv("PAPARAZZI_HOME"));
  string gazebodir = pprz_home + gazebo_home;
  cout << "Gazebo directory: " << gazebodir << endl;

  if (getenv("ROS_MASTER_URI")) {
    // Launch with ROS support
    cout << "Add ROS plugins... ";
    gazebo::addPlugin("libgazebo_ros_paths_plugin.so");
    gazebo::addPlugin("libgazebo_ros_api_plugin.so");
    cout << "ok" << endl;
  }

  if (!gazebo::setupServer(0, NULL)) {
    cout << "Failed to start Gazebo, exiting." << endl;
    std::exit(-1);
  }

  cout << "Add Paparazzi paths: " << gazebodir << endl;
  gazebo::common::SystemPaths::Instance()->AddModelPaths(gazebodir + "models/");
  sdf::addURIPath("model://", gazebodir + "models/");
  sdf::addURIPath("world://", gazebodir + "world/");

  cout << "Add TU Delft paths: " << pprz_home + "/sw/ext/tudelft_gazebo_models/" << endl;
  gazebo::common::SystemPaths::Instance()->AddModelPaths(pprz_home + "/sw/ext/tudelft_gazebo_models/models/");
  sdf::addURIPath("model://", pprz_home + "/sw/ext/tudelft_gazebo_models/models/");
  sdf::addURIPath("world://", pprz_home + "/sw/ext/tudelft_gazebo_models/world/");

  // get vehicles
  string vehicle_uri = "model://" + string(NPS_GAZEBO_AC_NAME) + "/" + string(NPS_GAZEBO_AC_NAME) + ".sdf";
  string vehicle_filename = sdf::findFile(vehicle_uri, false);
  if (vehicle_filename.empty()) {
    cout << "ERROR, could not find vehicle " + vehicle_uri << endl;
    std::exit(-1);
  }
  cout << "Load vehicle: " << vehicle_filename << endl;
  sdf::SDFPtr vehicle_sdf(new sdf::SDF());
  sdf::init(vehicle_sdf);
  if (!sdf::readFile(vehicle_filename, vehicle_sdf)) {
    cout << "ERROR, could not read vehicle " + vehicle_filename << endl;
    std::exit(-1);
  }

  // add or set up sensors before the vehicle gets loaded
#if NPS_SIMULATE_VIDEO
  // Cameras
  sdf::ElementPtr link = vehicle_sdf->Root()->GetFirstElement()->GetElement("link");
  while (link) {
    if (link->Get<string>("name") == "front_camera" && link->GetElement("sensor")->Get<string>("name") == "mt9f002") {
      if (NPS_MT9F002_SENSOR_RES_DIVIDER != 1) {
        int w = link->GetElement("sensor")->GetElement("camera")->GetElement("image")->GetElement("width")->Get<int>();
        int h = link->GetElement("sensor")->GetElement("camera")->GetElement("image")->GetElement("height")->Get<int>();
        int env = link->GetElement("sensor")->GetElement("camera")->GetElement("lens")->GetElement("env_texture_size")->Get<int>();
        link->GetElement("sensor")->GetElement("camera")->GetElement("image")->GetElement("width")->Set(w / NPS_MT9F002_SENSOR_RES_DIVIDER);
        link->GetElement("sensor")->GetElement("camera")->GetElement("image")->GetElement("height")->Set(h / NPS_MT9F002_SENSOR_RES_DIVIDER);
        link->GetElement("sensor")->GetElement("camera")->GetElement("lens")->GetElement("env_texture_size")->Set(env / NPS_MT9F002_SENSOR_RES_DIVIDER);
      }
      if (MT9F002_TARGET_FPS){
        int fps = Min(MT9F002_TARGET_FPS, link->GetElement("sensor")->GetElement("update_rate")->Get<int>());
        link->GetElement("sensor")->GetElement("update_rate")->Set(fps);
      }
    } else if  (link->Get<string>("name") == "bottom_camera" && link->GetElement("sensor")->Get<string>("name") == "mt9v117") {
      if (MT9V117_TARGET_FPS){
        int fps = Min(MT9V117_TARGET_FPS, link->GetElement("sensor")->GetElement("update_rate")->Get<int>());
        link->GetElement("sensor")->GetElement("update_rate")->Set(fps);
      }
    }
    link = link->GetNextElement("link");
  }
#endif

  // laser range array
#if NPS_SIMULATE_LASER_RANGE_ARRAY
  vehicle_sdf->Root()->GetFirstElement()->AddElement("include")->GetElement("uri")->Set("model://range_sensors");
  sdf::ElementPtr range_joint = vehicle_sdf->Root()->GetFirstElement()->AddElement("joint");
  range_joint->GetAttribute("name")->Set("range_sensor_joint");
  range_joint->GetAttribute("type")->Set("fixed");
  range_joint->GetElement("parent")->Set("chassis");
  range_joint->GetElement("child")->Set("range_sensors::base");
#endif

  // get world
  string world_uri = "world://" + string(NPS_GAZEBO_WORLD);
  string world_filename = sdf::findFile(world_uri, false);
  if (world_filename.empty()) {
    cout << "ERROR, could not find world " + world_uri << endl;
    std::exit(-1);
  }
  cout << "Load world: " << world_filename << endl;
  sdf::SDFPtr world_sdf(new sdf::SDF());
  sdf::init(world_sdf);
  if (!sdf::readFile(world_filename, world_sdf)) {
    cout << "ERROR, could not read world " + world_filename << endl;
    std::exit(-1);
  }

  // add vehicles
  world_sdf->Root()->GetFirstElement()->InsertElement(vehicle_sdf->Root()->GetFirstElement());

  world_sdf->Write(pprz_home + "/var/gazebo.world");

  gazebo::physics::WorldPtr world = gazebo::loadWorld(pprz_home + "/var/gazebo.world");
  if (!world) {
    cout << "Failed to open world, exiting." << endl;
    std::exit(-1);
  }

  cout << "Get pointer to aircraft: " << NPS_GAZEBO_AC_NAME << endl;
  model = world->ModelByName(NPS_GAZEBO_AC_NAME);
  if (!model) {
    cout << "Failed to find '" << NPS_GAZEBO_AC_NAME << "', exiting."
         << endl;
    std::exit(-1);
  }

  // Initialize sensors
  gazebo::sensors::run_once(true);
  gazebo::sensors::run_threads();
  gazebo::runWorld(world, 1);
  cout << "Sensors initialized..." << endl;

  // Find sensors
  // Contact sensor
  gazebo::sensors::SensorManager *mgr = gazebo::sensors::SensorManager::Instance();
  ct = static_pointer_cast < gazebo::sensors::ContactSensor > (mgr->GetSensor("contactsensor"));
  ct->SetActive(true);
  // Sonar
  sonar = static_pointer_cast<gazebo::sensors::SonarSensor>(mgr->GetSensor("sonarsensor"));
  if(sonar) {
    cout << "Found sonar" << endl;
  }

  gazebo::physics::LinkPtr sonar_link = model->GetLink("sonar");
  if (sonar_link) {
    // Get a pointer to the sensor using its full name
    if (sonar_link->GetSensorCount() != 1) {
      cout << "ERROR: Link '" << sonar_link->GetName()
           << "' should only contain 1 sensor!" << endl;
    } else {
      string name = sonar_link->GetSensorName(0);
      sonar = static_pointer_cast< gazebo::sensors::SonarSensor > (mgr->GetSensor(name));
      if (!sonar) {
        cout << "ERROR: Could not get pointer to '" << name << "'!" << endl;
      } else {
        // Activate sensor
        sonar->SetActive(true);
      }
    }
  }

  // Overwrite motor directions as defined by motor_mixing
#ifdef MOTOR_MIXING_YAW_COEF
  const double yaw_coef[] = MOTOR_MIXING_YAW_COEF;

  for (uint8_t i = 0; i < NPS_COMMANDS_NB; i++) {
    gazebo_actuators.torques[i] = -fabs(gazebo_actuators.torques[i]) * yaw_coef[i] / fabs(yaw_coef[i]);
    gazebo_actuators.max_ang_momentum[i] = -fabs(gazebo_actuators.max_ang_momentum[i]) * yaw_coef[i] / fabs(yaw_coef[i]);
  }
#endif
  cout << "Gazebo initialized successfully!" << endl;
}

/**
 * Read Gazebo's simulation state and store the results in the fdm struct used
 * by NPS.
 *
 * Not all fields are filled at the moment, as some of them are unused by
 * paparazzi (see comments) and others are not available in Gazebo 7
 * (atmosphere).
 */
static void gazebo_read(void)
{
  static ignition::math::Vector3d vel_prev;
  static double time_prev;

  gazebo::physics::WorldPtr world = model->GetWorld();
  ignition::math::Pose3d pose = model->WorldPose(); // In LOCAL xyz frame
  ignition::math::Vector3d vel = model->WorldLinearVel();
  ignition::math::Vector3d ang_vel = model->WorldAngularVel();
  gazebo::common::SphericalCoordinatesPtr sphere = world->SphericalCoords();
  ignition::math::Quaterniond local_to_global_quat(0, 0, -sphere->HeadingOffset().Radian());

  /* Fill FDM struct */
  fdm.time = world->SimTime().Double();

  // Find world acceleration by differentiating velocity
  // model->GetWorldLinearAccel() does not seem to take the velocity_decay into account!
  // Derivation of the velocity also follows the IMU implementation of Gazebo itself:
  // https://bitbucket.org/osrf/gazebo/src/e26144434b932b4b6a760ddaa19cfcf9f1734748/gazebo/sensors/ImuSensor.cc?at=default&fileviewer=file-view-default#ImuSensor.cc-370
  double dt = fdm.time - time_prev;
  ignition::math::Vector3d accel = (vel - vel_prev) / dt;
  vel_prev = vel;
  time_prev = fdm.time;

  // init_dt: unused
  // curr_dt: unused
  // on_ground: unused
  // nan_count: unused

  // Transform ltp definition to double for accuracy
  struct LtpDef_d ltpdef_d;
  ltpdef_d.ecef.x = state.ned_origin_f.ecef.x;
  ltpdef_d.ecef.y = state.ned_origin_f.ecef.y;
  ltpdef_d.ecef.z = state.ned_origin_f.ecef.z;
  ltpdef_d.lla.lat = state.ned_origin_f.lla.lat;
  ltpdef_d.lla.lon = state.ned_origin_f.lla.lon;
  ltpdef_d.lla.alt = state.ned_origin_f.lla.alt;
  for (int i = 0; i < 3 * 3; i++) {
    ltpdef_d.ltp_of_ecef.m[i] = state.ned_origin_f.ltp_of_ecef.m[i];
  }
  ltpdef_d.hmsl = state.ned_origin_f.hmsl;

  /* position */
  fdm.ltpprz_pos = to_pprz_ned(sphere->PositionTransform(pose.Pos(), gazebo::common::SphericalCoordinates::LOCAL,
                               gazebo::common::SphericalCoordinates::GLOBAL));  // Allows Gazebo worlds rotated wrt north.
  fdm.hmsl = -fdm.ltpprz_pos.z;
  ecef_of_ned_point_d(&fdm.ecef_pos, &ltpdef_d, &fdm.ltpprz_pos);
  lla_of_ecef_d(&fdm.lla_pos, &fdm.ecef_pos);

  /* debug positions */
  fdm.lla_pos_pprz = fdm.lla_pos; // Don't really care...
  fdm.lla_pos_geod = fdm.lla_pos;
  fdm.lla_pos_geoc = fdm.lla_pos;
  fdm.lla_pos_geoc.lat = gc_of_gd_lat_d(fdm.lla_pos.lat, fdm.hmsl);

  if(sonar) {
    double agl = sonar->Range();
    if (agl > sonar->RangeMax()) agl = -1.0;
    fdm.agl = agl;
  } else {
    fdm.agl = pose.Pos().Z(); // TODO Measure with sensor
  }

  /* velocity */
  fdm.ltp_ecef_vel = to_pprz_ned(sphere->VelocityTransform(vel, gazebo::common::SphericalCoordinates::LOCAL,
                                 gazebo::common::SphericalCoordinates::GLOBAL));
  fdm.ltpprz_ecef_vel = fdm.ltp_ecef_vel; // ???
  fdm.body_ecef_vel = to_pprz_body(pose.Rot().RotateVectorReverse(vel)); // Note: unused
  ecef_of_ned_vect_d(&fdm.ecef_ecef_vel, &ltpdef_d, &fdm.ltp_ecef_vel);

  /* acceleration */
  fdm.ltp_ecef_accel = to_pprz_ned(sphere->VelocityTransform(accel, gazebo::common::SphericalCoordinates::LOCAL,
                                     gazebo::common::SphericalCoordinates::GLOBAL)); // Note: unused
  fdm.ltpprz_ecef_accel = fdm.ltp_ecef_accel; // ???
  fdm.body_ecef_accel = to_pprz_body(pose.Rot().RotateVectorReverse(accel));
  fdm.body_inertial_accel = fdm.body_ecef_accel; // Approximate, unused.
  fdm.body_accel = to_pprz_body(pose.Rot().RotateVectorReverse(accel - world->Gravity()));
  ecef_of_ned_vect_d(&fdm.ecef_ecef_accel, &ltpdef_d, &fdm.ltp_ecef_accel);

  /* attitude */
  // ecef_to_body_quat: unused
  fdm.ltp_to_body_eulers = to_global_pprz_eulers(local_to_global_quat * pose.Rot());
  double_quat_of_eulers(&(fdm.ltp_to_body_quat), &(fdm.ltp_to_body_eulers));
  fdm.ltpprz_to_body_quat = fdm.ltp_to_body_quat; // unused
  fdm.ltpprz_to_body_eulers = fdm.ltp_to_body_eulers; // unused

  /* angular velocity */
  fdm.body_ecef_rotvel = to_pprz_rates(pose.Rot().RotateVectorReverse(ang_vel));
  fdm.body_inertial_rotvel = fdm.body_ecef_rotvel; // Approximate

  /* angular acceleration */
  // body_ecef_rotaccel: unused
  // body_inertial_rotaccel: unused
  /* misc */
  fdm.ltp_g = to_pprz_ltp(sphere->VelocityTransform(-1 * world->Gravity(), gazebo::common::SphericalCoordinates::LOCAL,
                          gazebo::common::SphericalCoordinates::GLOBAL)); // unused
  fdm.ltp_h = to_pprz_ltp(sphere->VelocityTransform(world->MagneticField(), gazebo::common::SphericalCoordinates::LOCAL,
                          gazebo::common::SphericalCoordinates::GLOBAL));

  /* atmosphere */
#if GAZEBO_MAJOR_VERSION >= 8 && 0 // TODO implement

#else
  // Gazebo versions < 8 do not have atmosphere or wind support
  // Use placeholder values. Valid for low altitude, low speed flights.
  fdm.wind = (struct DoubleVect3) {0, 0, 0};
  fdm.pressure_sl = 101325; // Pa

  fdm.airspeed = 0;
  fdm.pressure = pprz_isa_pressure_of_height(fdm.hmsl, fdm.pressure_sl);
  fdm.dynamic_pressure = fdm.pressure_sl; // Pa
  fdm.temperature = 20.0; // C
  fdm.aoa = 0; // rad
  fdm.sideslip = 0; // rad
#endif
  /* flight controls: unused */
  fdm.elevator = 0;
  fdm.flap = 0;
  fdm.left_aileron = 0;
  fdm.right_aileron = 0;
  fdm.rudder = 0;
  /* engine: unused */
  fdm.num_engines = 0;
}

/**
 * Write actuator commands to Gazebo.
 *
 * This function takes the normalized commands and applies them as forces and
 * torques in Gazebo. Since the commands are normalized in [0,1], their
 * thrusts (NPS_ACTUATOR_THRUSTS) and torques (NPS_ACTUATOR_TORQUES) need to
 * be specified in the airframe file. Their values need to be specified in the
 * same order as the NPS_ACTUATOR_NAMES and should be provided in SI units.
 * See conf/airframes/examples/ardrone2_gazebo.xml for an example.
 *
 * The forces and torques are applied to (the origin of) the links named in
 * NPS_ACTUATOR_NAMES. See conf/simulator/gazebo/models/ardrone/ardrone.sdf
 * for an example.
 *
 * @param act_commands
 * @param commands_nb
 */
static void gazebo_write(double act_commands[], int commands_nb)
{
  for (int i = 0; i < commands_nb; ++i) {
    // Thrust setpoint
    double sp = autopilot.motors_on ? act_commands[i] : 0.0;  // Normalized thrust setpoint

    // Actuator dynamics, forces and torques
#ifdef NPS_ACTUATOR_TIME_CONSTANTS
    // Delayed response from actuator
    double u = update_first_order_low_pass(&gazebo_actuators.lowpass[i], sp);// Normalized actual thrust
#else
    double u = sp;
#endif
    double thrust = gazebo_actuators.thrusts[i] * u;
    double torque = gazebo_actuators.torques[i] * u;

#ifdef NPS_ACTUATOR_MAX_ANGULAR_MOMENTUM
    // Spinup torque
    double udot = update_first_order_high_pass(&gazebo_actuators.highpass[i], sp);
    double spinup_torque = gazebo_actuators.max_ang_momentum[i] / (2.0 * sqrt(u > 0.05 ? u : 0.05)) * udot;
    torque += spinup_torque;
#endif

    // Apply force and torque to gazebo model
    gazebo::physics::LinkPtr link = model->GetLink(gazebo_actuators.names[i]);
    link->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust));
    link->AddRelativeTorque(ignition::math::Vector3d(0, 0, torque));
  }
}

#if NPS_SIMULATE_VIDEO
/**
 * Set up cameras.
 *
 * This function finds the video devices added through add_video_device
 * (sw/airborne/modules/computer_vision/cv.h). The camera links in the Gazebo AC
 * model should have the same name as the .dev_name field in the corresponding
 * video_config_t struct stored in 'cameras[]' (computer_vision/
 * video_thread_nps.h). Pointers to Gazebo's cameras are stored in gazebo_cams
 * at the same index as their 'cameras[]' counterpart.
 *
 * The video_config_t parameters are updated using the values provided by
 * Gazebo. This should simplify the use of different UAVs with different camera
 * setups.
 */
static void init_gazebo_video(void)
{
  gazebo::sensors::SensorManager *mgr = gazebo::sensors::SensorManager::Instance();

  cout << "Initializing cameras..." << endl;
  // Loop over cameras registered in video_thread_nps
  for (int i = 0; i < VIDEO_THREAD_MAX_CAMERAS && cameras[i] != NULL; ++i) {
    // Find link in gazebo model
    cout << "Setting up '" << cameras[i]->dev_name << "'... ";
    gazebo::physics::LinkPtr link = model->GetLink(cameras[i]->dev_name);
    if (!link) {
      cout << "ERROR: Link '" << cameras[i]->dev_name << "' not found!"
           << endl;
      ;
      continue;
    }
    // Get a pointer to the sensor using its full name
    if (link->GetSensorCount() != 1) {
      cout << "ERROR: Link '" << link->GetName()
           << "' should only contain 1 sensor!" << endl;
      continue;
    }
    string name = link->GetSensorName(0);
    gazebo::sensors::CameraSensorPtr cam = static_pointer_cast
                                           < gazebo::sensors::CameraSensor > (mgr->GetSensor(name));
    if (!cam) {
      cout << "ERROR: Could not get pointer to '" << name << "'!" << endl;
      continue;
    }
    // Activate sensor
    cam->SetActive(true);

    // Add to list of cameras
    gazebo_cams[i].cam = cam;
    gazebo_cams[i].last_measurement_time = cam->LastMeasurementTime();

    // set default camera settings
    // Copy video_config settings from Gazebo's camera
    cameras[i]->output_size.w = cam->ImageWidth();
    cameras[i]->output_size.h = cam->ImageHeight();
    cameras[i]->sensor_size.w = cam->ImageWidth();
    cameras[i]->sensor_size.h = cam->ImageHeight();
    cameras[i]->crop.w = cam->ImageWidth();
    cameras[i]->crop.h = cam->ImageHeight();
    cameras[i]->fps = 0;
    cameras[i]->camera_intrinsics.focal_x = cameras[i]->output_size.w / 2.0f;
    cameras[i]->camera_intrinsics.center_x = cameras[i]->output_size.w / 2.0f;
    cameras[i]->camera_intrinsics.focal_y = cameras[i]->output_size.h / 2.0f;
    cameras[i]->camera_intrinsics.center_y = cameras[i]->output_size.h / 2.0f;

    if (cam->Name() == "mt9f002") {
      // See boards/bebop/mt9f002.c
      cameras[i]->output_size.w = MT9F002_OUTPUT_WIDTH;
      cameras[i]->output_size.h = MT9F002_OUTPUT_HEIGHT;
      cameras[i]->sensor_size.w = MT9F002_OUTPUT_WIDTH;
      cameras[i]->sensor_size.h = MT9F002_OUTPUT_HEIGHT;
      cameras[i]->crop.w = MT9F002_OUTPUT_WIDTH;
      cameras[i]->crop.h = MT9F002_OUTPUT_HEIGHT;
      cameras[i]->fps = MT9F002_TARGET_FPS;
      cameras[i]->camera_intrinsics = {
        .focal_x = MT9F002_FOCAL_X,
        .focal_y = MT9F002_FOCAL_Y,
        .center_x = MT9F002_CENTER_X,
        .center_y = MT9F002_CENTER_Y,
        .Dhane_k = MT9F002_DHANE_K
      };
    } else if (cam->Name() == "mt9v117") {
      // See boards/bebop/mt9v117.h
      cameras[i]->fps = MT9V117_TARGET_FPS;
      cameras[i]->camera_intrinsics = {
        .focal_x = MT9V117_FOCAL_X,
        .focal_y = MT9V117_FOCAL_Y,
        .center_x = MT9V117_CENTER_X,
        .center_y = MT9V117_CENTER_Y,
        .Dhane_k = MT9V117_DHANE_K
      };
    }
    cout << "ok" << endl;
  }
}

/**
 * Read camera images.
 *
 * Polls gazebo cameras. If the last measurement time has been updated, a new
 * frame is available. This frame is converted to Paparazzi's UYVY format
 * and passed to cv_run_device which runs the callbacks registered by various
 * modules.
 */
static void gazebo_read_video(void)
{
  for (int i = 0; i < VIDEO_THREAD_MAX_CAMERAS; ++i) {
    gazebo::sensors::CameraSensorPtr &cam = gazebo_cams[i].cam;
    // Skip unregistered or unfound cameras
    if (cam == NULL) { continue; }
    // Skip if not updated
    // Also skip when LastMeasurementTime() is zero (workaround)
    if ((cam->LastMeasurementTime() - gazebo_cams[i].last_measurement_time).Float() < 0.005
        || cam->LastMeasurementTime() == 0) { continue; }
    // Grab image, convert and send to video thread
    struct image_t img;
    read_image(&img, cam);

#if NPS_DEBUG_VIDEO
    cv::Mat RGB_cam(cam->ImageHeight(), cam->ImageWidth(), CV_8UC3, (uint8_t *)cam->ImageData());
    cv::cvtColor(RGB_cam, RGB_cam, cv::COLOR_RGB2BGR);
    cv::namedWindow(cameras[i]->dev_name, cv::WINDOW_AUTOSIZE);  // Create a window for display.
    cv::imshow(cameras[i]->dev_name, RGB_cam);
    cv::waitKey(1);
#endif

    cv_run_device(cameras[i], &img);
    // Free image buffer after use.
    image_free(&img);
    // Keep track of last update time.
    gazebo_cams[i].last_measurement_time = cam->LastMeasurementTime();
  }
}

/**
 * Read Gazebo image and convert.
 *
 * Converts the current camera frame to the format used by Paparazzi. This
 * includes conversion to UYVY. Gazebo's simulation time is used for the image
 * timestamp.
 *
 * @param img
 * @param cam
 */
static void read_image(struct image_t *img, gazebo::sensors::CameraSensorPtr cam)
{
  bool is_mt9f002 = false;
  if (cam->Name() == "mt9f002") {
    image_create(img, MT9F002_OUTPUT_WIDTH, MT9F002_OUTPUT_HEIGHT, IMAGE_YUV422);
    is_mt9f002 = true;
  } else {
    image_create(img, cam->ImageWidth(), cam->ImageHeight(), IMAGE_YUV422);
  }

  // Convert Gazebo's *RGB888* image to Paparazzi's YUV422
  const uint8_t *data_rgb = cam->ImageData();
  uint8_t *data_yuv = (uint8_t *)(img->buf);
  for (int x_yuv = 0; x_yuv < img->w; ++x_yuv) {
    for (int y_yuv = 0; y_yuv < img->h; ++y_yuv) {
      int x_rgb = x_yuv;
      int y_rgb = y_yuv;
      if (is_mt9f002) {
        // Change sampling points for zoomed and/or cropped image.
        // Use nearest-neighbour sampling for now.
        x_rgb = (mt9f002.offset_x + ((float)x_yuv / img->w) * mt9f002.sensor_width)
            / CFG_MT9F002_PIXEL_ARRAY_WIDTH * cam->ImageWidth();
        y_rgb = (mt9f002.offset_y + ((float)y_yuv / img->h) * mt9f002.sensor_height)
            / CFG_MT9F002_PIXEL_ARRAY_HEIGHT * cam->ImageHeight();
      }
      int idx_rgb = 3 * (cam->ImageWidth() * y_rgb + x_rgb);
      int idx_yuv = 2 * (img->w * y_yuv + x_yuv);
      int idx_px = img->w * y_yuv + x_yuv;
      if (idx_px % 2 == 0) { // Pick U or V
        data_yuv[idx_yuv] = - 0.148 * data_rgb[idx_rgb]
                            - 0.291 * data_rgb[idx_rgb + 1]
                            + 0.439 * data_rgb[idx_rgb + 2] + 128; // U
      } else {
        data_yuv[idx_yuv] =   0.439 * data_rgb[idx_rgb]
                              - 0.368 * data_rgb[idx_rgb + 1]
                              - 0.071 * data_rgb[idx_rgb + 2] + 128; // V
      }
      data_yuv[idx_yuv + 1] =   0.257 * data_rgb[idx_rgb]
                                + 0.504 * data_rgb[idx_rgb + 1]
                                + 0.098 * data_rgb[idx_rgb + 2] + 16; // Y
    }
  }
  // Fill miscellaneous fields
  gazebo::common::Time ts = cam->LastMeasurementTime();
  img->ts.tv_sec = ts.sec;
  img->ts.tv_usec = ts.nsec / 1000.0;
  img->pprz_ts = ts.Double() * 1e6;
  img->buf_idx = 0; // unused
}
#endif

#if NPS_SIMULATE_LASER_RANGE_ARRAY
/*
 * Simulate range sensors:
 *
 * In the airframe file, set NPS_SIMULATE_LASER_RANGE_ARRAY if you want to make use of the integrated Ray sensors.
 * These are defined in their own model which is added to the ardrone.sdf (called range_sensors). Here you can add
 * single ray point sensors with a specified position and orientation. It is also possible add noise.
 *
 * Within the airframe file (see e.g ardrone2_rangesensors_gazebo.xml), the amount of sensors
 * (LASER_RANGE_ARRAY_NUM_SENSORS) and their orientations
 * (LASER_RANGE_ARRAY_ORIENTATIONS={phi_1,theta_1,psi_1...phi_n,theta_n,psi_n}  n = number of sensors) need to be
 * specified as well. This is to keep it generic since this need to be done on the real platform with an external
 * ray sensor. The function will compare the orientations from the ray sensors of gazebo, with the ones specified
 * in the airframe, and will fill up an array to send and abi message to be used by other modules.
 *
 * NPS_GAZEBO_RANGE_SEND_AGL defines if the sensor that is defined as down should be used to send an AGL message.
 * to send an OBSTACLE_DETECTION message.
 *
 * Functions:
 *   gazebo_init_range_sensors() -> Finds and initializes all ray sensors in gazebo
 *   gazebo_read_range_sensors() -> Reads and evaluates the ray sensors values, and sending it to other pprz modules
 */

struct gazebo_ray_t {
  gazebo::sensors::RaySensorPtr sensor;
  gazebo::common::Time last_measurement_time;
};

static struct gazebo_ray_t rays[LASER_RANGE_ARRAY_NUM_SENSORS] = {{NULL, 0}};
static float range_orientation[] = LASER_RANGE_ARRAY_ORIENTATIONS;
static uint8_t ray_sensor_agl_index = 255;

#define VL53L0_MAX_VAL 8.191f

static void gazebo_init_range_sensors(void)
{
  gazebo::sensors::SensorManager *mgr = gazebo::sensors::SensorManager::Instance();
  gazebo::sensors::Sensor_V sensors = mgr->GetSensors();  // list of all sensors

  uint16_t sensor_count = model->GetSensorCount();

  gazebo::sensors::RaySensorPtr ray_sensor;
  uint8_t ray_sensor_count_selected = 0;

  // Loop though all sensors and only select ray sensors, which are saved within a struct
  for (uint16_t i = 0; i < sensor_count; i++) {
    if (ray_sensor_count_selected > LASER_RANGE_ARRAY_NUM_SENSORS) {
      break;
    }

    if (sensors.at(i)->Type() == "ray") {
      ray_sensor = static_pointer_cast<gazebo::sensors::RaySensor>(sensors.at(i));

      if (!ray_sensor) {
        cout << "ERROR: Could not get pointer to raysensor " << i << "!" << endl;
        continue;
      }

      // Read out the pose from per ray sensors in gazebo relative to body
      struct DoubleEulers pose_sensor = to_pprz_eulers(ray_sensor->Pose().Rot());

      struct DoubleQuat q_ray, q_def;
      double_quat_of_eulers(&q_ray, &pose_sensor);

      /* Check the orientations of the ray sensors found in gazebo, if they are similar (within 5 deg) to the orientations
       * given in the airframe file in LASER_RANGE_ARRAY_RANGE_ORIENTATION
       */
      for (int j = 0; j < LASER_RANGE_ARRAY_NUM_SENSORS; j++) {
        struct DoubleEulers def = {0, range_orientation[j * 2], range_orientation[j * 2 + 1]};
        double_quat_of_eulers(&q_def, &def);
        // get angle between required angle and ray angle
        double angle = acos(QUAT_DOT_PRODUCT(q_ray, q_def));

        if (fabs(angle) < RadOfDeg(5)) {
          ray_sensor_count_selected++;
          rays[j].sensor = ray_sensor;
          rays[j].sensor->SetActive(true);

#if LASER_RANGE_ARRAY_SEND_AGL
          // find the sensor pointing down
          if (fabs(range_orientation[j * 2] + M_PI_2) < RadOfDeg(5)) {
            ray_sensor_agl_index = j;
          }
#endif
          break;
        }
      }
    }
  }

  if (ray_sensor_count_selected != LASER_RANGE_ARRAY_NUM_SENSORS) {
    cout << "ERROR: you have defined " << LASER_RANGE_ARRAY_NUM_SENSORS << " sensors in your airframe file, but only "
         << ray_sensor_count_selected << " sensors have been found in the gazebo simulator, "
         "with the same orientation as in the airframe file " << endl;
    exit(0);
  }
  cout << "Initialized laser range array" << endl;
}

static void gazebo_read_range_sensors(void)
{
  static float range;

  // Loop through all ray sensors found in gazebo
  for (int i = 0; i < LASER_RANGE_ARRAY_NUM_SENSORS; i++) {
    if ((rays[i].sensor->LastMeasurementTime() - rays[i].last_measurement_time).Float() < 0.005
        || rays[i].sensor->LastMeasurementTime() == 0) { continue; }

    if (rays[i].sensor->Range(0) < 1e-5 || rays[i].sensor->Range(0) > VL53L0_MAX_VAL) {
      range = VL53L0_MAX_VAL;
    } else {
      range = rays[i].sensor->Range(0);
    }
    AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_RANGE_ARRAY_NPS_ID, range, range_orientation[i * 2],
                                 range_orientation[i * 2 + 1]);

    if (i == ray_sensor_agl_index) {
      uint32_t now_ts = get_sys_time_usec();
      float agl = rays[i].sensor->Range(0);
      // Down range sensor as agl
      if (agl > 1e-5 && !isinf(agl)) {
        AbiSendMsgAGL(AGL_RAY_SENSOR_GAZEBO_ID, now_ts, agl);
      }
    }
    rays[i].last_measurement_time = rays[i].sensor->LastMeasurementTime();
  }
}
#endif  // NPS_SIMULATE_LASER_RANGE_ARRAY

#pragma GCC diagnostic pop // Ignore -Wdeprecated-declarations
