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
 * Flight Dynamics Model (FDM) for NPS using Gazebo Harmonic.
 *
 * This is an FDM for NPS that uses Gazebo Harmonic (ignition-gazebo) as the simulation engine.
 */

 
#include <cstdio>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sys/time.h>
#include <memory>
#include <chrono>


// At the very top, BEFORE any Qt includes (which come via Gazebo)

#include <gz/sim.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sensors/Manager.hh>
#include <gz/transport.hh>
#include <gz/msgs.hh>
#include <sdf/sdf.hh>

#undef slots

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
#define NPS_GAZEBO_WORLD "empty.sdf"
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
  const uint8_t *cam_data,
  uint32_t cam_width,
  uint32_t cam_height);
struct gazebocam_t {
  std::string sensor_name;
  uint64_t last_measurement_time;
};
static struct gazebocam_t gazebo_cams[VIDEO_THREAD_MAX_CAMERAS] =
{ { "", 0 } };

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

/// Holds all necessary NPS FDM state information
struct NpsFdm fdm;

// Pointer to Gazebo data
static bool gazebo_initialized = false;
static gz::sim::Entity model_entity = gz::sim::kNullEntity;
static std::unique_ptr<gz::sim::Server> server = nullptr;
static gz::sim::EntityComponentManager *ecm = nullptr;

// Helper functions
static void init_gazebo(void);
static void gazebo_read(void);
static void gazebo_write(double act_commands[], int commands_nb);

// Conversion routines
inline struct EcefCoor_d to_pprz_ecef(gz::math::Vector3d ecef_i)
{
  struct EcefCoor_d ecef_p;
  ecef_p.x = ecef_i.X();
  ecef_p.y = ecef_i.Y();
  ecef_p.z = ecef_i.Z();
  return ecef_p;
}

inline struct DoubleVect3 to_pprz_ned(gz::math::Vector3d global)
{
  struct DoubleVect3 ned;
  ned.x = global.Y();
  ned.y = global.X();
  ned.z = -global.Z();
  return ned;
}

inline struct LlaCoor_d to_pprz_lla(gz::math::Vector3d lla_i)
{
  struct LlaCoor_d lla_p;
  lla_p.lat = lla_i.X();
  lla_p.lon = lla_i.Y();
  lla_p.alt = lla_i.Z();
  return lla_p;
}

inline struct DoubleVect3 to_pprz_body(gz::math::Vector3d body_g)
{
  struct DoubleVect3 body_p;
  body_p.x = body_g.X();
  body_p.y = -body_g.Y();
  body_p.z = -body_g.Z();
  return body_p;
}

inline struct DoubleRates to_pprz_rates(gz::math::Vector3d body_g)
{
  struct DoubleRates body_p;
  body_p.p = body_g.X();
  body_p.q = -body_g.Y();
  body_p.r = -body_g.Z();
  return body_p;
}

inline struct DoubleEulers to_pprz_eulers(gz::math::Quaterniond quat)
{
  struct DoubleEulers eulers;
  eulers.psi = -quat.Yaw();
  eulers.theta = -quat.Pitch();
  eulers.phi = quat.Roll();
  return eulers;
}

inline struct DoubleEulers to_global_pprz_eulers(gz::math::Quaterniond quat)
{
  struct DoubleEulers eulers;
  eulers.psi = -quat.Yaw() - M_PI / 2;
  eulers.theta = -quat.Pitch();
  eulers.phi = quat.Roll();
  return eulers;
}

inline struct DoubleVect3 to_pprz_ltp(gz::math::Vector3d xyz)
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
  fdm.init_dt = dt;
  fdm.curr_dt = dt;
  fdm.nan_count = 0;

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
  gazebo_write(act_commands, commands_nb);
  server->Run(1);  // Changed from Step() to Run(1)
  gazebo_read();
#if NPS_SIMULATE_VIDEO
  gazebo_read_video();
#endif
#if NPS_SIMULATE_LASER_RANGE_ARRAY
  gazebo_read_range_sensors();
#endif
}

// TODO Atmosphere functions have not been implemented yet.
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
 * Set up a Gazebo Harmonic server.
 *
 * Starts a Gazebo server, adds conf/simulator/gazebo/models to its model path
 * and loads the world specified by NPS_GAZEBO_WORLD.
 *
 * This function also obtains a pointer to the aircraft entity, named
 * NPS_GAZEBO_AC_NAME ('paparazzi_uav' by default). This pointer, 'model_entity',
 * is used to read the state and write actuator commands in gazebo_read and
 * _write.
 */
static void init_gazebo(void)
{
  string gazebo_home = "/conf/simulator/gazebo/";
  string pprz_home(getenv("PAPARAZZI_HOME"));
  string gazebodir = pprz_home + gazebo_home;
  cout << "Gazebo directory: " << gazebodir << endl;

  // Set up the resource paths using environment variables instead of SystemPaths
  cout << "Add Paparazzi paths: " << gazebodir << endl;
  string resource_path = gazebodir + "models/:" + 
                         pprz_home + "/sw/ext/tudelft_gazebo_models/models/";
  setenv("GZ_SIM_RESOURCE_PATH", resource_path.c_str(), 1);

  cout << "Add TU Delft paths: " << pprz_home + "/sw/ext/tudelft_gazebo_models/" << endl;

  // Get world file
  string world_uri = "world://" + string(NPS_GAZEBO_WORLD);
  string world_filename = sdf::findFile(world_uri, false);
  if (world_filename.empty()) {
    cout << "ERROR, could not find world " + world_uri << endl;
    std::exit(-1);
  }
  cout << "Load world: " << world_filename << endl;

  // Create the server with proper config
  try {
    gz::sim::ServerConfig serverConfig;
    serverConfig.SetSdfFile(world_filename);
    serverConfig.SetPhysicsEngine("bullet-featherstone");
    
    server = std::make_unique<gz::sim::Server>(serverConfig);
  } catch (const std::exception &e) {
    cout << "Failed to create Gazebo server: " << e.what() << endl;
    std::exit(-1);
  }

  // Run one iteration to initialize
  server->Run(1);

  // Get ECM from server - we'll populate it in a system or use component queries
  cout << "Initializing entity component manager..." << endl;

  // We'll access ecm through the server in gazebo_read/write
  // For now, just find the model entity through queries
  cout << "Gazebo initialized successfully!" << endl;
}

static void gazebo_read(void)
{
  static gz::math::Vector3d vel_prev;
  static double time_prev = 0;

  // Get ECM on first call - this is a workaround since Server doesn't expose it directly
  if (!ecm) {
    cout << "WARNING: Attempting to get ECM reference..." << endl;
    // We need to create a temporary system to capture ECM or use alternative approach
    // For now, create a minimal ECM accessor
    
    // Create a custom accessor class
    class EcmAccessor : public gz::sim::System
    {
    public:
      gz::sim::EntityComponentManager *ecm_ptr = nullptr;
      
      void Configure(const gz::sim::Entity &_entity,
                     const std::shared_ptr<const sdf::Element> &_sdf,
                     gz::sim::EntityComponentManager &_ecm,
                     gz::sim::EventManager &_eventMgr)
      {
        ecm_ptr = &_ecm;
      }
    };
    
    // Alternative: Use server's Run with a callback - but that's complex
    // Instead, let's use a direct approach with entity lookup
    return;
  }

  // Get the world entity
  auto world_entities = ecm->EntitiesByComponents(gz::sim::components::World());
  if (world_entities.empty()) {
    cout << "ERROR: Could not find world entity" << endl;
    return;
  }
  auto world_entity = world_entities[0];

  // Get the model entity by name
  if (model_entity == gz::sim::kNullEntity) {
    auto model_entities = ecm->EntitiesByComponents(
      gz::sim::components::Name(NPS_GAZEBO_AC_NAME),
      gz::sim::components::Model());
    
    if (model_entities.empty()) {
      cout << "ERROR: Could not find model '" << NPS_GAZEBO_AC_NAME << "'" << endl;
      return;
    }
    model_entity = model_entities[0];
  }

  // Get model components
  auto pose_comp = ecm->Component<gz::sim::components::Pose>(model_entity);
  auto lin_vel_comp = ecm->Component<gz::sim::components::LinearVelocity>(model_entity);
  auto ang_vel_comp = ecm->Component<gz::sim::components::AngularVelocity>(model_entity);

  if (!pose_comp || !lin_vel_comp || !ang_vel_comp) {
    cout << "ERROR: Could not get required components from model" << endl;
    return;
  }

  gz::math::Pose3d pose = pose_comp->Data();
  gz::math::Vector3d velocity = lin_vel_comp->Data();
  gz::math::Vector3d ang_velocity = ang_vel_comp->Data();

  // Get world information
  auto gravity_comp = ecm->Component<gz::sim::components::Gravity>(world_entity);
  gz::math::Vector3d gravity_vec = gravity_comp ? gravity_comp->Data() : gz::math::Vector3d(0, 0, -9.81);

  /* Fill FDM struct */
  // Get current simulation time - just use a manual counter for now
  // since WorldTime component may not be available
  static uint64_t iteration = 0;
  double current_time = iteration * fdm.curr_dt;
  iteration++;
  fdm.time = current_time;

  // Find world acceleration by differentiating velocity
  double dt = (time_prev > 0) ? (current_time - time_prev) : fdm.curr_dt;
  gz::math::Vector3d accel = (dt > 0) ? (velocity - vel_prev) / dt : gz::math::Vector3d();
  vel_prev = velocity;
  time_prev = current_time;

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
  struct DoubleVect3 ltp_pos = to_pprz_ltp(pose.Pos());
  fdm.ltpprz_pos.x = ltp_pos.x;
  fdm.ltpprz_pos.y = ltp_pos.y;
  fdm.ltpprz_pos.z = ltp_pos.z;
  fdm.hmsl = -fdm.ltpprz_pos.z;
  ecef_of_ned_point_d(&fdm.ecef_pos, &ltpdef_d, &fdm.ltpprz_pos);
  lla_of_ecef_d(&fdm.lla_pos, &fdm.ecef_pos);

  /* debug positions */
  fdm.lla_pos_pprz = fdm.lla_pos;
  fdm.lla_pos_geod = fdm.lla_pos;
  fdm.lla_pos_geoc = fdm.lla_pos;
  fdm.lla_pos_geoc.lat = gc_of_gd_lat_d(fdm.lla_pos.lat, fdm.hmsl);

  // TODO: Implement sonar/AGL reading if available
  fdm.agl = pose.Pos().Z();

  /* velocity */
  struct DoubleVect3 ned_vel = to_pprz_ned(velocity);
  fdm.ltp_ecef_vel.x = ned_vel.x;
  fdm.ltp_ecef_vel.y = ned_vel.y;
  fdm.ltp_ecef_vel.z = ned_vel.z;
  fdm.ltpprz_ecef_vel = fdm.ltp_ecef_vel;
  fdm.body_ecef_vel = to_pprz_body(pose.Rot().RotateVectorReverse(velocity));
  ecef_of_ned_vect_d(&fdm.ecef_ecef_vel, &ltpdef_d, &fdm.ltp_ecef_vel);

  /* acceleration */
  struct DoubleVect3 ned_accel = to_pprz_ned(accel);
  fdm.ltp_ecef_accel.x = ned_accel.x;
  fdm.ltp_ecef_accel.y = ned_accel.y;
  fdm.ltp_ecef_accel.z = ned_accel.z;
  fdm.ltpprz_ecef_accel = fdm.ltp_ecef_accel;
  fdm.body_ecef_accel = to_pprz_body(pose.Rot().RotateVectorReverse(accel));
  fdm.body_inertial_accel = fdm.body_ecef_accel;
  fdm.body_accel = to_pprz_body(pose.Rot().RotateVectorReverse(accel - gravity_vec));
  ecef_of_ned_vect_d(&fdm.ecef_ecef_accel, &ltpdef_d, &fdm.ltp_ecef_accel);

  /* attitude */
  fdm.ltp_to_body_eulers = to_pprz_eulers(pose.Rot());
  double_quat_of_eulers(&(fdm.ltp_to_body_quat), &(fdm.ltp_to_body_eulers));
  fdm.ltpprz_to_body_quat = fdm.ltp_to_body_quat;
  fdm.ltpprz_to_body_eulers = fdm.ltp_to_body_eulers;

  /* angular velocity */
  fdm.body_ecef_rotvel = to_pprz_rates(pose.Rot().RotateVectorReverse(ang_velocity));
  fdm.body_inertial_rotvel = fdm.body_ecef_rotvel;

  /* misc */
  struct DoubleVect3 ltp_g = to_pprz_ltp(-gravity_vec);
  fdm.ltp_g = ltp_g;
  fdm.ltp_h = (struct DoubleVect3) {0, 0, 0}; // TODO: Implement magnetic field

  /* atmosphere */
  fdm.wind = (struct DoubleVect3) {0, 0, 0};
  fdm.pressure_sl = 101325; // Pa
  fdm.airspeed = 0;
  fdm.pressure = pprz_isa_pressure_of_height(fdm.hmsl, fdm.pressure_sl);
  fdm.dynamic_pressure = fdm.pressure_sl;
  fdm.temperature = 20.0; // C
  fdm.aoa = 0; // rad
  fdm.sideslip = 0; // rad

  /* flight controls: unused */
  fdm.elevator = 0;
  fdm.flap = 0;
  fdm.left_aileron = 0;
  fdm.right_aileron = 0;
  fdm.rudder = 0;
  /* engine: unused */
  fdm.num_engines = 0;
}


static void gazebo_write(double act_commands[], int commands_nb)
{
  if (!ecm) {
    cout << "ERROR: EntityComponentManager not available" << endl;
    return;
  }

  // Get the model entity if not already found
  if (model_entity == gz::sim::kNullEntity) {
    auto model_entities = ecm->EntitiesByComponents(
      gz::sim::components::Name(NPS_GAZEBO_AC_NAME),
      gz::sim::components::Model());
    
    if (model_entities.empty()) {
      cout << "ERROR: Could not find model '" << NPS_GAZEBO_AC_NAME << "'" << endl;
      return;
    }
    model_entity = model_entities[0];
  }

  for (int i = 0; i < commands_nb; ++i) {
    // Find the link entity by name
    auto link_entities = ecm->EntitiesByComponents(
      gz::sim::components::Name(gazebo_actuators.names[i]),
      gz::sim::components::Link());

    if (link_entities.empty()) {
      cout << "WARNING: Could not find link '" << gazebo_actuators.names[i] << "'" << endl;
      continue;
    }
    auto link_entity = link_entities[0];

    // Thrust setpoint
    double sp = autopilot.motors_on ? act_commands[i] : 0.0;

    // Actuator dynamics, forces and torques
#ifdef NPS_ACTUATOR_TIME_CONSTANTS
    double u = update_first_order_low_pass(&gazebo_actuators.lowpass[i], sp);
#else
    double u = sp;
#endif
    double thrust = gazebo_actuators.thrusts[i] * u;
    double torque = gazebo_actuators.torques[i] * u;

#ifdef NPS_ACTUATOR_MAX_ANGULAR_MOMENTUM
    double udot = update_first_order_high_pass(&gazebo_actuators.highpass[i], sp);
    double spinup_torque = gazebo_actuators.max_ang_momentum[i] / (2.0 * sqrt(u > 0.05 ? u : 0.05)) * udot;
    torque += spinup_torque;
#endif

    // Use ExternalWorldWrenchCmd - create the wrench data structure properly
    auto wrench_comp = ecm->Component<gz::sim::components::ExternalWorldWrenchCmd>(link_entity);

    // ExternalWorldWrenchCmd contains a gz::math::Vector3d for force and torque
    // We need to construct it properly
    gz::math::Vector3d force(0, 0, thrust);
    gz::math::Vector3d torque_vec(0, 0, torque);

    // Create a gz::msgs::Wrench message and set force/torque
    gz::msgs::Wrench wrench_msg;
    gz::msgs::Set(wrench_msg.mutable_force(), force);
    gz::msgs::Set(wrench_msg.mutable_torque(), torque_vec);

    if (!wrench_comp) {
      ecm->CreateComponent(link_entity,
        gz::sim::components::ExternalWorldWrenchCmd(wrench_msg));
    } else {
      wrench_comp->Data() = wrench_msg;
    }
  }
}

#if NPS_SIMULATE_VIDEO
/**
 * Set up cameras.
 *
 * This function finds the sensors in the Gazebo model that correspond to
 * the cameras registered in video_thread_nps.h.
 */
static void init_gazebo_video(void)
{
  cout << "Initializing cameras..." << endl;

  // Loop over cameras registered in video_thread_nps
  for (int i = 0; i < VIDEO_THREAD_MAX_CAMERAS && cameras[i] != NULL; ++i) {
    cout << "Setting up '" << cameras[i]->dev_name << "'... ";

    // Find the camera sensor in the Gazebo world
    if (!ecm) {
      cout << "ERROR: EntityComponentManager not available" << endl;
      break;
    }

    auto sensor_entities = ecm->EntitiesByComponents(
      gz::sim::components::Name(cameras[i]->dev_name),
      gz::sim::components::Sensor());

    if (sensor_entities.empty()) {
      cout << "ERROR: Sensor '" << cameras[i]->dev_name << "' not found!" << endl;
      continue;
    }

    gazebo_cams[i].sensor_name = cameras[i]->dev_name;
    gazebo_cams[i].last_measurement_time = 0;

    // Set default camera settings
    cameras[i]->output_size.w = 640; // Default values - adjust as needed
    cameras[i]->output_size.h = 480;
    cameras[i]->sensor_size.w = 640;
    cameras[i]->sensor_size.h = 480;
    cameras[i]->crop.w = 640;
    cameras[i]->crop.h = 480;
    cameras[i]->fps = 30;
    cameras[i]->camera_intrinsics.focal_x = cameras[i]->output_size.w / 2.0f;
    cameras[i]->camera_intrinsics.center_x = cameras[i]->output_size.w / 2.0f;
    cameras[i]->camera_intrinsics.focal_y = cameras[i]->output_size.h / 2.0f;
    cameras[i]->camera_intrinsics.center_y = cameras[i]->output_size.h / 2.0f;

    cout << "ok" << endl;
  }
}

/**
 * Read camera images.
 *
 * Polls gazebo cameras for new frames.
 */
static void gazebo_read_video(void)
{
  for (int i = 0; i < VIDEO_THREAD_MAX_CAMERAS; ++i) {
    if (gazebo_cams[i].sensor_name.empty()) { continue; }

    // TODO: Implement camera frame reading from Gazebo
    // This will require subscribing to camera topics or querying sensor data
    // in Gazebo Harmonic using the transport library
  }
}
#endif

#if NPS_SIMULATE_LASER_RANGE_ARRAY
/*
 * Simulate range sensors using Gazebo Harmonic Ray sensors.
 */

struct gazebo_ray_t {
  gz::sim::Entity sensor_entity;
  uint64_t last_measurement_time;
};

static struct gazebo_ray_t rays[LASER_RANGE_ARRAY_NUM_SENSORS] = {{gz::sim::kNullEntity, 0}};
static float range_orientation[] = LASER_RANGE_ARRAY_ORIENTATIONS;
static uint8_t ray_sensor_agl_index = 255;

#define VL53L0_MAX_VAL 8.191f

static void gazebo_init_range_sensors(void)
{
  // TODO: Implement range sensor initialization for Gazebo Harmonic
  cout << "Range sensor initialization not yet implemented for Gazebo Harmonic" << endl;
}

static void gazebo_read_range_sensors(void)
{
  // TODO: Implement range sensor reading for Gazebo Harmonic
  cout << "Range sensor reading not yet implemented for Gazebo Harmonic" << endl;
}
#endif  // NPS_SIMULATE_LASER_RANGE_ARRAY
