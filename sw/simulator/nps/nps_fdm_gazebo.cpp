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
 * This is an FDM for NPS that uses Gazebo Harmonic (gz-sim) as the simulation
 * engine. It uses a custom gz::sim::System plugin to access the Entity
 * Component Manager (ECM) for reading simulation state and applying forces.
 */

#include <cstdio>
#include <cstdlib>
#include <string>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <memory>
#include <chrono>

#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <sdf/sdf.hh>

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

// Shared state: written by PaparazziSystem::PostUpdate, read by gazebo_read
static struct {
  gz::math::Pose3d pose;
  gz::math::Vector3d linear_vel;
  gz::math::Vector3d angular_vel;
  gz::math::Vector3d gravity;
  double sim_time;
  gz::math::SphericalCoordinates spherical_coords;
  bool has_spherical_coords;
  bool ready;
} gz_state = { {}, {}, {}, {}, 0.0, {}, false, false };

// Shared commands: written by gazebo_write, read by PaparazziSystem::PreUpdate
static struct {
  double thrusts[NPS_COMMANDS_NB];
  double torques[NPS_COMMANDS_NB];
} gz_actuator_cmd = {};

/**
 * Custom gz::sim::System that bridges Paparazzi's NPS with Gazebo Harmonic.
 *
 * In GZ Harmonic, the EntityComponentManager (ECM) is only accessible through
 * System callbacks. This class:
 *  - In Configure: finds the model and link entities
 *  - In PreUpdate: enables velocity tracking and applies actuator forces/torques
 *  - In PostUpdate: reads the simulation state for NPS
 */
class PaparazziSystem : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate,
                        public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &/*_sdf*/,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &/*_eventMgr*/) override
  {
    world_entity_ = _entity;
    FindModel(_ecm);
  }

  void PreUpdate(const gz::sim::UpdateInfo &/*_info*/,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    // Find model if not found during Configure
    if (model_entity_ == gz::sim::kNullEntity) {
      FindModel(_ecm);
    }
    if (model_entity_ == gz::sim::kNullEntity) { return; }

    // Enable velocity/acceleration tracking on canonical link
    if (!velocity_checks_enabled_ && canonical_link_ != gz::sim::kNullEntity) {
      gz::sim::Link link(canonical_link_);
      link.EnableVelocityChecks(_ecm, true);
      link.EnableAccelerationChecks(_ecm, true);
      velocity_checks_enabled_ = true;
    }

    // Apply forces and torques from shared command data
    for (int i = 0; i < NPS_COMMANDS_NB; i++) {
      if (link_entities_[i] == gz::sim::kNullEntity) { continue; }

      gz::sim::Link link(link_entities_[i]);
      auto link_pose = link.WorldPose(_ecm);
      if (!link_pose) { continue; }

      // Transform body-frame force/torque to world frame
      gz::math::Vector3d body_force(0, 0, gz_actuator_cmd.thrusts[i]);
      gz::math::Vector3d body_torque(0, 0, gz_actuator_cmd.torques[i]);
      gz::math::Vector3d world_force = link_pose->Rot().RotateVector(body_force);
      gz::math::Vector3d world_torque = link_pose->Rot().RotateVector(body_torque);

      link.AddWorldWrench(_ecm, world_force, world_torque);
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override
  {
    if (canonical_link_ == gz::sim::kNullEntity) { return; }

    gz::sim::Link link(canonical_link_);

    auto pose = link.WorldPose(_ecm);
    auto vel = link.WorldLinearVelocity(_ecm);
    auto ang_vel = link.WorldAngularVelocity(_ecm);

    // Get gravity from world
    auto gravity_comp = _ecm.Component<gz::sim::components::Gravity>(world_entity_);

    // Get spherical coordinates from world
    auto spherical_comp = _ecm.Component<gz::sim::components::SphericalCoordinates>(world_entity_);

    // Store state in shared struct
    if (pose) { gz_state.pose = *pose; }
    if (vel) { gz_state.linear_vel = *vel; }
    if (ang_vel) { gz_state.angular_vel = *ang_vel; }
    if (gravity_comp) { gz_state.gravity = gravity_comp->Data(); }
    if (spherical_comp) {
      gz_state.spherical_coords = spherical_comp->Data();
      gz_state.has_spherical_coords = true;
    }

    gz_state.sim_time = std::chrono::duration<double>(_info.simTime).count();
    gz_state.ready = true;
  }

  bool IsReady() const { return model_entity_ != gz::sim::kNullEntity; }

private:
  void FindModel(gz::sim::EntityComponentManager &_ecm)
  {
    // Find model by name
    _ecm.Each<gz::sim::components::Name, gz::sim::components::Model>(
      [&](const gz::sim::Entity &entity,
          const gz::sim::components::Name *name,
          const gz::sim::components::Model *) -> bool
      {
        if (name->Data() == NPS_GAZEBO_AC_NAME) {
          model_entity_ = entity;
          return false; // stop iteration
        }
        return true;
      });

    if (model_entity_ == gz::sim::kNullEntity) { return; }

    // Get canonical link for state queries
    gz::sim::Model model(model_entity_);
    canonical_link_ = model.CanonicalLink(_ecm);

    // Find link entities for actuators
    for (int i = 0; i < NPS_COMMANDS_NB; i++) {
      link_entities_[i] = gz::sim::kNullEntity;
      string target_name = gazebo_actuators.names[i];
      _ecm.Each<gz::sim::components::Name, gz::sim::components::Link>(
        [&](const gz::sim::Entity &entity,
            const gz::sim::components::Name *name,
            const gz::sim::components::Link *) -> bool
        {
          if (name->Data() == target_name) {
            link_entities_[i] = entity;
            return false;
          }
          return true;
        });
    }
  }

  gz::sim::Entity world_entity_{gz::sim::kNullEntity};
  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  gz::sim::Entity canonical_link_{gz::sim::kNullEntity};
  gz::sim::Entity link_entities_[NPS_COMMANDS_NB];
  bool velocity_checks_enabled_{false};
};

// Pointer to Gazebo data
static bool gazebo_initialized = false;
static std::unique_ptr<gz::sim::Server> server = nullptr;
static std::shared_ptr<PaparazziSystem> pprz_system = nullptr;

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

inline struct NedCoor_d to_pprz_ned(gz::math::Vector3d global)
{
  struct NedCoor_d ned;
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
  server->Run(true, 1, false);
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
 * The vehicle model (NPS_GAZEBO_AC_NAME) is loaded from the models directory
 * and inserted into the world SDF before loading. A custom System plugin
 * (PaparazziSystem) is registered with the server to access the ECM for
 * reading state and applying forces.
 */
static void init_gazebo(void)
{
  string gazebo_home = "/conf/simulator/gazebo/";
  string pprz_home(getenv("PAPARAZZI_HOME"));
  string gazebodir = pprz_home + gazebo_home;
  cout << "Gazebo directory: " << gazebodir << endl;

  // Register SDF URI paths for model:// and world:// resolution
  cout << "Add Paparazzi paths: " << gazebodir << endl;
  sdf::addURIPath("model://", gazebodir + "models/");
  sdf::addURIPath("world://", gazebodir + "world/");

  cout << "Add TU Delft paths: " << pprz_home + "/sw/ext/tudelft_gazebo_models/" << endl;
  sdf::addURIPath("model://", pprz_home + "/sw/ext/tudelft_gazebo_models/models/");
  sdf::addURIPath("world://", pprz_home + "/sw/ext/tudelft_gazebo_models/world/");

  // Set GZ resource paths for model resolution at runtime
  string resource_path = gazebodir + "models/:" +
                         pprz_home + "/sw/ext/tudelft_gazebo_models/models/";
  string existing_path = "";
  if (getenv("GZ_SIM_RESOURCE_PATH")) {
    existing_path = ":" + string(getenv("GZ_SIM_RESOURCE_PATH"));
  }
  setenv("GZ_SIM_RESOURCE_PATH", (resource_path + existing_path).c_str(), 1);

  // Load vehicle SDF
  string vehicle_uri = "model://" + string(NPS_GAZEBO_AC_NAME) + "/" +
                       string(NPS_GAZEBO_AC_NAME) + ".sdf";
  string vehicle_filename = sdf::findFile(vehicle_uri, false);
  if (vehicle_filename.empty()) {
    cout << "ERROR: could not find vehicle " << vehicle_uri << endl;
    std::exit(-1);
  }
  cout << "Load vehicle: " << vehicle_filename << endl;
  sdf::SDFPtr vehicle_sdf(new sdf::SDF());
  sdf::init(vehicle_sdf);
  if (!sdf::readFile(vehicle_filename, vehicle_sdf)) {
    cout << "ERROR: could not read vehicle " << vehicle_filename << endl;
    std::exit(-1);
  }

  // Load world SDF
  string world_uri = "world://" + string(NPS_GAZEBO_WORLD);
  string world_filename = sdf::findFile(world_uri, false);
  if (world_filename.empty()) {
    cout << "ERROR: could not find world " << world_uri << endl;
    std::exit(-1);
  }
  cout << "Load world: " << world_filename << endl;
  sdf::SDFPtr world_sdf(new sdf::SDF());
  sdf::init(world_sdf);
  if (!sdf::readFile(world_filename, world_sdf)) {
    cout << "ERROR: could not read world " << world_filename << endl;
    std::exit(-1);
  }

  // Insert vehicle model into the world SDF
  world_sdf->Root()->GetFirstElement()->InsertElement(
    vehicle_sdf->Root()->GetFirstElement());

  // Write merged SDF to disk
  string merged_path = pprz_home + "/var/gazebo.world";
  world_sdf->Write(merged_path);

  // Create GZ Sim server with merged world
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(merged_path);

  cout << "Creating Gazebo Harmonic server..." << endl;
  server = std::make_unique<gz::sim::Server>(serverConfig);

  // Register our custom system to access the ECM
  pprz_system = std::make_shared<PaparazziSystem>();
  server->AddSystem(pprz_system);

  // Run initial iteration to initialize entities and the system
  server->Run(true, 1, false);

  if (!pprz_system->IsReady()) {
    cout << "WARNING: Paparazzi system could not find model '"
         << NPS_GAZEBO_AC_NAME << "' after initial step" << endl;
    // Run additional iteration in case model loading was delayed
    server->Run(true, 1, false);
    if (!pprz_system->IsReady()) {
      cout << "ERROR: Model '" << NPS_GAZEBO_AC_NAME << "' not found, exiting." << endl;
      std::exit(-1);
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

  cout << "Gazebo Harmonic initialized successfully!" << endl;
}

/**
 * Read Gazebo's simulation state and store the results in the fdm struct
 * used by NPS.
 *
 * Reads from the shared state populated by PaparazziSystem::PostUpdate.
 */
static void gazebo_read(void)
{
  static gz::math::Vector3d vel_prev;
  static double time_prev = 0;

  if (!gz_state.ready) { return; }

  gz::math::Pose3d pose = gz_state.pose;
  gz::math::Vector3d vel = gz_state.linear_vel;
  gz::math::Vector3d ang_vel = gz_state.angular_vel;
  gz::math::Vector3d gravity_vec = gz_state.gravity;
  double current_time = gz_state.sim_time;

  // Compute LOCAL→GLOBAL rotation from spherical coordinates heading offset
  gz::math::Quaterniond local_to_global_quat(0, 0, 0);
  if (gz_state.has_spherical_coords) {
    local_to_global_quat = gz::math::Quaterniond(0, 0,
      -gz_state.spherical_coords.HeadingOffset().Radian());
  }

  /* Fill FDM struct */
  fdm.time = current_time;

  // Find world acceleration by differentiating velocity
  double dt = (time_prev > 0) ? (current_time - time_prev) : fdm.curr_dt;
  gz::math::Vector3d accel = (dt > 0) ? (vel - vel_prev) / dt : gz::math::Vector3d();
  vel_prev = vel;
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
  if (gz_state.has_spherical_coords) {
    fdm.ltpprz_pos = to_pprz_ned(
      gz_state.spherical_coords.PositionTransform(pose.Pos(),
        gz::math::SphericalCoordinates::LOCAL2,
        gz::math::SphericalCoordinates::GLOBAL));
  } else {
    fdm.ltpprz_pos = to_pprz_ned(pose.Pos());
  }
  fdm.hmsl = -fdm.ltpprz_pos.z;
  ecef_of_ned_point_d(&fdm.ecef_pos, &ltpdef_d, &fdm.ltpprz_pos);
  lla_of_ecef_d(&fdm.lla_pos, &fdm.ecef_pos);

  /* debug positions */
  fdm.lla_pos_pprz = fdm.lla_pos;
  fdm.lla_pos_geod = fdm.lla_pos;
  fdm.lla_pos_geoc = fdm.lla_pos;
  fdm.lla_pos_geoc.lat = gc_of_gd_lat_d(fdm.lla_pos.lat, fdm.hmsl);

  fdm.agl = pose.Pos().Z();

  /* velocity */
  if (gz_state.has_spherical_coords) {
    fdm.ltp_ecef_vel = to_pprz_ned(
      gz_state.spherical_coords.VelocityTransform(vel,
        gz::math::SphericalCoordinates::LOCAL2,
        gz::math::SphericalCoordinates::GLOBAL));
  } else {
    fdm.ltp_ecef_vel = to_pprz_ned(local_to_global_quat.RotateVector(vel));
  }
  fdm.ltpprz_ecef_vel = fdm.ltp_ecef_vel;
  fdm.body_ecef_vel = to_pprz_body(pose.Rot().RotateVectorReverse(vel));
  ecef_of_ned_vect_d(&fdm.ecef_ecef_vel, &ltpdef_d, &fdm.ltp_ecef_vel);

  /* acceleration */
  if (gz_state.has_spherical_coords) {
    fdm.ltp_ecef_accel = to_pprz_ned(
      gz_state.spherical_coords.VelocityTransform(accel,
        gz::math::SphericalCoordinates::LOCAL2,
        gz::math::SphericalCoordinates::GLOBAL));
  } else {
    fdm.ltp_ecef_accel = to_pprz_ned(local_to_global_quat.RotateVector(accel));
  }
  fdm.ltpprz_ecef_accel = fdm.ltp_ecef_accel;
  fdm.body_ecef_accel = to_pprz_body(pose.Rot().RotateVectorReverse(accel));
  fdm.body_inertial_accel = fdm.body_ecef_accel;
  fdm.body_accel = to_pprz_body(pose.Rot().RotateVectorReverse(accel - gravity_vec));
  ecef_of_ned_vect_d(&fdm.ecef_ecef_accel, &ltpdef_d, &fdm.ltp_ecef_accel);

  /* attitude */
  fdm.ltp_to_body_eulers = to_global_pprz_eulers(local_to_global_quat * pose.Rot());
  double_quat_of_eulers(&(fdm.ltp_to_body_quat), &(fdm.ltp_to_body_eulers));
  fdm.ltpprz_to_body_quat = fdm.ltp_to_body_quat;
  fdm.ltpprz_to_body_eulers = fdm.ltp_to_body_eulers;

  /* angular velocity */
  fdm.body_ecef_rotvel = to_pprz_rates(pose.Rot().RotateVectorReverse(ang_vel));
  fdm.body_inertial_rotvel = fdm.body_ecef_rotvel;

  /* misc */
  if (gz_state.has_spherical_coords) {
    fdm.ltp_g = to_pprz_ltp(
      gz_state.spherical_coords.VelocityTransform(-1 * gravity_vec,
        gz::math::SphericalCoordinates::LOCAL2,
        gz::math::SphericalCoordinates::GLOBAL));
  } else {
    fdm.ltp_g = to_pprz_ltp(local_to_global_quat.RotateVector(-gravity_vec));
  }
  fdm.ltp_h = (struct DoubleVect3) {0, 0, 0};

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

/**
 * Write actuator commands to Gazebo.
 *
 * This function takes the normalized commands and applies them as forces and
 * torques via the PaparazziSystem. Since the commands are normalized in [0,1],
 * their thrusts (NPS_ACTUATOR_THRUSTS) and torques (NPS_ACTUATOR_TORQUES) need
 * to be specified in the airframe file.
 */
static void gazebo_write(double act_commands[], int commands_nb)
{
  for (int i = 0; i < commands_nb; ++i) {
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

    // Store in shared command struct for PaparazziSystem::PreUpdate
    gz_actuator_cmd.thrusts[i] = thrust;
    gz_actuator_cmd.torques[i] = torque;
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

    gazebo_cams[i].sensor_name = cameras[i]->dev_name;
    gazebo_cams[i].last_measurement_time = 0;

    // Set default camera settings
    cameras[i]->output_size.w = 640;
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

    // TODO: Implement camera frame reading from Gazebo Harmonic
    // This will require subscribing to camera topics via gz::transport
  }
}
#endif

#if NPS_SIMULATE_LASER_RANGE_ARRAY
/*
 * Simulate range sensors using Gazebo Harmonic.
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
