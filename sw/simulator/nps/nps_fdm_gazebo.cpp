/*
 * Copyright (C) 2017 Tom van Dijk, Kirk Scheper
 * Copyright (C) 2025 Updated for Gazebo Harmonic
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
 * This is an FDM for NPS that uses Gazebo Harmonic as the simulation engine.
 */

#include <cstdio>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sys/time.h>

// Gazebo Harmonic includes
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components.hh>
#include <gz/sim/Util.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/sensors/ContactSensor.hh>
#include <gz/sensors/DistanceSensor.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Pose3.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs.hh>

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
  gz::sensors::CameraSensor *cam);

struct gazebocam_t {
  gz::sensors::CameraSensor *cam;
  gz::msgs::Image last_image_msg;
  std::chrono::steady_clock::time_point last_measurement_time;
};
static struct gazebocam_t gazebo_cams[VIDEO_THREAD_MAX_CAMERAS] =
{ { NULL, gz::msgs::Image(), std::chrono::steady_clock::time_point() } };

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

std::shared_ptr<gz::sensors::DistanceSensor> sonar = NULL;

/// Holds all necessary NPS FDM state information
struct NpsFdm fdm;

// Pointer to Gazebo data
static bool gazebo_initialized = false;
static gz::sim::Entity modelEntity;
static gz::sim::Model model;
static gz::sim::Server *server = NULL;
static gz::sim::EntityComponentManager *ecm = NULL;
static gz::sim::EventManager *eventMgr = NULL;
static gz::transport::Node node;

// Get contact sensor
static gz::sensors::ContactSensor *ct = NULL;

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
  fdm.init_dt = dt; // JSBsim specific
  fdm.curr_dt = dt; // JSBsim specific
  fdm.nan_count = 0; // JSBsim specific

#ifdef NPS_ACTUATOR_TIME_CONSTANTS
  // Set up low-pass filter to simulate delayed actuator response
  const float tau[NPS_COMMANDS_NB] = NPS_ACTUATOR_TIME_CONSTANTS;
  for (int i = 0; i < NPS_COMMANDS_NB; i++) {
    init_first_order_low_pass(&gazebo_actuators.lowpass[i], tau[i], dt, 0.f);
  }
#endif
#ifdef NPS_ACTUATOR_MAX_ANGULAR_MOMENTUM
  const float max_ang_mom[NPS_COMMANDS_NB] = NPS_ACTUATOR_MAX_ANGULAR_MOMENTUM;
  for (int i = 0; i < NPS_COMMANDS_NB; i++) {
    gazebo_actuators.max_ang_momentum[i] = max_ang_mom[i];
    init_first_order_high_pass(&gazebo_actuators.highpass[i], 1. / (2 * M_PI * 5), dt, 0.f);
  }
#endif

  init_gazebo();
  gazebo_read();
}

/**
 * Update the simulation state.
 * @param launch
 * @param act_commands
 * @param commands_nb
 */
void nps_fdm_run_step(bool launch __attribute__((unused)), double *act_commands, int commands_nb)
{
  // Actuator dynamics, thrust and torque
  gazebo_write(act_commands, commands_nb);

  // Run simulation step
  if (server) {
    server->Run(true, 1, false);
  }

  // Update FDM state
  gazebo_read();

#if NPS_SIMULATE_VIDEO
  gazebo_read_video();
#endif

#if NPS_SIMULATE_LASER_RANGE_ARRAY
  gazebo_read_range_sensors();
#endif
}

void nps_fdm_set_wind(double speed __attribute__((unused)),
                      double dir __attribute__((unused)))
{
  // Not implemented in Gazebo Harmonic yet
}

void nps_fdm_set_wind_ned(double wind_north __attribute__((unused)),
                          double wind_east __attribute__((unused)),
                          double wind_down __attribute__((unused)))
{
  // Not implemented in Gazebo Harmonic yet
}

void nps_fdm_set_turbulence(double wind_speed __attribute__((unused)),
                            int turbulence_severity __attribute__((unused)))
{
  // Not implemented in Gazebo Harmonic yet
}

void nps_fdm_set_temperature(double temp __attribute__((unused)),
                             double h __attribute__((unused)))
{
  // Not implemented in Gazebo Harmonic yet
}

/**
 * Set up a Gazebo Harmonic server and load the world.
 */
static void init_gazebo(void)
{
  cout << "Initializing Gazebo Harmonic server..." << endl;

  // Configure server
  gz::sim::ServerConfig serverConfig;
  
  // Load world file
  string worldFile = string(getenv("PAPARAZZI_HOME")) + "/conf/simulator/gazebo/worlds/" + NPS_GAZEBO_WORLD;
  serverConfig.SetSdfFile(worldFile);
  
  // Create server
  server = new gz::sim::Server(serverConfig);
  
  if (!server || !server->Running()) {
    cerr << "Failed to start Gazebo Harmonic server" << endl;
    exit(1);
  }

  // Run a few iterations to let the world load
  server->Run(true, 100, false);

  cout << "Gazebo Harmonic server started successfully" << endl;

  // Get entity component manager through a system callback
  bool ecmFound = false;
  server->SetUpdatePeriod(std::chrono::duration<double>(fdm.init_dt));
  
  // Access ECM and EventManager
  auto callback = [&](const gz::sim::UpdateInfo &_info,
                     gz::sim::EntityComponentManager &_ecm)
  {
    if (!ecmFound) {
      ecm = &_ecm;
      ecmFound = true;
      
      // Find model entity
      gz::sim::Entity worldEntity = gz::sim::worldEntity(*ecm);
      
      ecm->Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Model *,
            const gz::sim::components::Name *_name) -> bool
        {
          if (_name->Data() == NPS_GAZEBO_AC_NAME) {
            modelEntity = _entity;
            model = gz::sim::Model(_entity);
            cout << "Found model: " << NPS_GAZEBO_AC_NAME << endl;
            return false;
          }
          return true;
        });
      
      if (modelEntity == gz::sim::kNullEntity) {
        cerr << "Model '" << NPS_GAZEBO_AC_NAME << "' not found!" << endl;
        exit(1);
      }
    }
  };

  // Register the callback as a system
  server->AddSystem(std::make_shared<std::function<void(const gz::sim::UpdateInfo &,
                                                         gz::sim::EntityComponentManager &)>>(callback));
  
  // Run until ECM is found
  for (int i = 0; i < 100 && !ecmFound; i++) {
    server->Run(true, 1, false);
  }

  if (!ecmFound || !ecm) {
    cerr << "Failed to get EntityComponentManager" << endl;
    exit(1);
  }

  gazebo_initialized = true;

#if NPS_SIMULATE_VIDEO
  init_gazebo_video();
#endif

#if NPS_SIMULATE_LASER_RANGE_ARRAY
  gazebo_init_range_sensors();
#endif

  cout << "Gazebo initialization complete" << endl;
}

/**
 * Read Gazebo Harmonic state and populate FDM struct.
 */
static void gazebo_read(void)
{
  if (!ecm || modelEntity == gz::sim::kNullEntity) {
    return;
  }

  // Get world pose
  auto poseComp = ecm->Component<gz::sim::components::Pose>(modelEntity);
  if (!poseComp) {
    return;
  }
  gz::math::Pose3d pose = poseComp->Data();

  // Get world velocity
  auto worldLinVelComp = ecm->Component<gz::sim::components::WorldLinearVelocity>(modelEntity);
  auto worldAngVelComp = ecm->Component<gz::sim::components::WorldAngularVelocity>(modelEntity);
  
  gz::math::Vector3d worldLinVel(0, 0, 0);
  gz::math::Vector3d worldAngVel(0, 0, 0);
  
  if (worldLinVelComp) {
    worldLinVel = worldLinVelComp->Data();
  }
  if (worldAngVelComp) {
    worldAngVel = worldAngVelComp->Data();
  }

  // Get body velocity (in body frame)
  gz::math::Vector3d bodyLinVel = pose.Rot().RotateVectorReverse(worldLinVel);
  gz::math::Vector3d bodyAngVel = pose.Rot().RotateVectorReverse(worldAngVel);

  // Get world acceleration
  auto worldLinAccelComp = ecm->Component<gz::sim::components::WorldLinearAcceleration>(modelEntity);
  auto worldAngAccelComp = ecm->Component<gz::sim::components::WorldAngularAcceleration>(modelEntity);
  
  gz::math::Vector3d worldLinAccel(0, 0, 0);
  gz::math::Vector3d worldAngAccel(0, 0, 0);
  
  if (worldLinAccelComp) {
    worldLinAccel = worldLinAccelComp->Data();
  }
  if (worldAngAccelComp) {
    worldAngAccel = worldAngAccelComp->Data();
  }

  gz::math::Vector3d bodyLinAccel = pose.Rot().RotateVectorReverse(worldLinAccel);
  gz::math::Vector3d bodyAngAccel = pose.Rot().RotateVectorReverse(worldAngAccel);

  // Fill FDM struct
  fdm.ecef_pos = to_pprz_ecef(pose.Pos());
  fdm.ltpprz_pos = to_pprz_ltp(pose.Pos());
  fdm.ltp_ecef_vel = to_pprz_ltp(worldLinVel);
  fdm.body_ecef_vel = to_pprz_body(bodyLinVel);
  fdm.body_ecef_accel = to_pprz_body(bodyLinAccel);
  fdm.body_inertial_accel = fdm.body_ecef_accel;
  fdm.body_accel = fdm.body_ecef_accel;
  fdm.ltp_ecef_accel = to_pprz_ltp(worldLinAccel);

  fdm.ecef_ecef_vel = fdm.ltp_ecef_vel;
  fdm.ecef_ecef_accel = fdm.ltp_ecef_accel;

  fdm.hmsl = pose.Pos().Z();

  fdm.ltp_g = to_pprz_ltp(gz::math::Vector3d(0, 0, -9.81));
  fdm.ltp_h = to_pprz_ltp(gz::math::Vector3d(0, 0, -9.81));

  // Attitude
  fdm.ltp_to_body_eulers = to_global_pprz_eulers(pose.Rot());
  fdm.ltpprz_to_body_eulers = to_pprz_eulers(pose.Rot());

  // Rates
  fdm.body_ecef_rotvel = to_pprz_rates(bodyAngVel);
  fdm.body_ecef_rotaccel = to_pprz_rates(bodyAngAccel);
  fdm.body_inertial_rotvel = fdm.body_ecef_rotvel;
  fdm.body_inertial_rotaccel = fdm.body_ecef_rotaccel;

  // LLA position
  // For Gazebo Harmonic, we need to use the spherical coordinates component
  auto sphericalCoordComp = ecm->Component<gz::sim::components::SphericalCoordinates>(
      gz::sim::worldEntity(*ecm));
  
  if (sphericalCoordComp) {
    auto sc = sphericalCoordComp->Data();
    gz::math::Vector3d lla = sc.PositionTransform(pose.Pos(),
                                                   gz::math::SphericalCoordinates::LOCAL2,
                                                   gz::math::SphericalCoordinates::SPHERICAL);
    fdm.lla_pos = to_pprz_lla(lla);
  } else {
    // Fallback if no spherical coordinates
    fdm.lla_pos.lat = 0;
    fdm.lla_pos.lon = 0;
    fdm.lla_pos.alt = pose.Pos().Z();
  }

  // Atmospheric properties
  fdm.pressure_sl = pprz_isa_pressure_of_height(0);
  fdm.pressure = pprz_isa_pressure_of_height(fdm.hmsl);
  fdm.total_pressure = fdm.pressure + 0.5 * 1.225 * 
                       (fdm.body_ecef_vel.x * fdm.body_ecef_vel.x +
                        fdm.body_ecef_vel.y * fdm.body_ecef_vel.y +
                        fdm.body_ecef_vel.z * fdm.body_ecef_vel.z);
  fdm.dynamic_pressure = fdm.total_pressure - fdm.pressure;
  fdm.temperature = pprz_isa_temperature_of_height(fdm.hmsl);

  // Airspeed (simplified, assuming no wind)
  fdm.airspeed = sqrt(fdm.body_ecef_vel.x * fdm.body_ecef_vel.x +
                      fdm.body_ecef_vel.y * fdm.body_ecef_vel.y +
                      fdm.body_ecef_vel.z * fdm.body_ecef_vel.z);

  // Ground contact
  fdm.agl = pose.Pos().Z(); // Simplified
  if (ct) {
    // Check contact sensor (implementation depends on sensor setup)
    fdm.on_ground = (fdm.agl < 0.01);
  } else {
    fdm.on_ground = (fdm.agl < 0.01);
  }

  // Simulation time
  auto simTimeComp = ecm->Component<gz::sim::components::SimulatedTime>(
      gz::sim::worldEntity(*ecm));
  if (simTimeComp) {
    auto simTime = simTimeComp->Data();
    fdm.time = std::chrono::duration<double>(simTime).count();
  }
}

/**
 * Write actuator commands to Gazebo Harmonic.
 * 
 * @param act_commands Array of actuator commands
 * @param commands_nb Number of actuator commands
 */
static void gazebo_write(double act_commands[], int commands_nb)
{
  if (!ecm || modelEntity == gz::sim::kNullEntity) {
    return;
  }

  // Apply actuator dynamics
  for (int i = 0; i < commands_nb; i++) {
    double input = act_commands[i];
    
#ifdef NPS_ACTUATOR_TIME_CONSTANTS
    // Low-pass filter for actuator dynamics
    double filtered = update_first_order_low_pass(&gazebo_actuators.lowpass[i], input);
    act_commands[i] = filtered;
#endif

#ifdef NPS_ACTUATOR_MAX_ANGULAR_MOMENTUM
    // High-pass filter for back-emf effects
    double ang_vel = update_first_order_high_pass(&gazebo_actuators.highpass[i], act_commands[i]);
    double back_emf_reduction = fabs(ang_vel) / gazebo_actuators.max_ang_momentum[i];
    back_emf_reduction = (back_emf_reduction > 1.0) ? 1.0 : back_emf_reduction;
    act_commands[i] *= (1.0 - back_emf_reduction);
#endif
  }

  // Find links and apply forces/torques
  model = gz::sim::Model(modelEntity);
  
  for (int i = 0; i < commands_nb; i++) {
    // Find link by name
    gz::sim::Entity linkEntity = model.LinkByName(*ecm, gazebo_actuators.names[i]);
    
    if (linkEntity != gz::sim::kNullEntity) {
      // Calculate thrust force
      double thrust = gazebo_actuators.thrusts[i] * act_commands[i];
      double torque = gazebo_actuators.torques[i] * act_commands[i];
      
      // Apply force in link's local z-direction (upward)
      gz::math::Vector3d force(0, 0, thrust);
      gz::math::Vector3d torqueVec(0, 0, torque);
      
      // Get or create WorldForce component
      auto forceComp = ecm->Component<gz::sim::components::WorldForce>(linkEntity);
      if (!forceComp) {
        ecm->CreateComponent(linkEntity, 
                            gz::sim::components::WorldForce(force));
      } else {
        // Add to existing force
        auto currentForce = forceComp->Data();
        ecm->SetComponentData<gz::sim::components::WorldForce>(linkEntity, 
                                                                currentForce + force);
      }
      
      // Get or create WorldTorque component
      auto torqueComp = ecm->Component<gz::sim::components::WorldTorque>(linkEntity);
      if (!torqueComp) {
        ecm->CreateComponent(linkEntity, 
                            gz::sim::components::WorldTorque(torqueVec));
      } else {
        // Add to existing torque
        auto currentTorque = torqueComp->Data();
        ecm->SetComponentData<gz::sim::components::WorldTorque>(linkEntity, 
                                                                 currentTorque + torqueVec);
      }
    }
  }
}

#if NPS_SIMULATE_VIDEO
/**
 * Initialize Gazebo Harmonic cameras for video simulation.
 */
static void init_gazebo_video(void)
{
  cout << "Initializing cameras..." << endl;

  // Gazebo Harmonic uses gz-sensors for camera access
  gz::sensors::SensorFactory sensorFactory;
  
  // For now, this is a placeholder
  // In Gazebo Harmonic, camera sensors are typically accessed via topics
  // You would subscribe to camera image topics using gz::transport::Node
  
  cout << "Camera initialization for Gazebo Harmonic requires topic-based approach" << endl;
  cout << "Subscribe to camera topics like /camera or /camera/image" << endl;

  // Example: Set up topic subscribers for cameras
  for (int i = 0; i < VIDEO_THREAD_MAX_CAMERAS; ++i) {
    if (cameras[i] != NULL) {
      string topicName = string("/") + cameras[i]->dev_name + "/image";
      
      // Subscribe to camera topic
      std::function<void(const gz::msgs::Image &)> cb =
        [i](const gz::msgs::Image &_msg) {
          gazebo_cams[i].last_image_msg = _msg;
          gazebo_cams[i].last_measurement_time = std::chrono::steady_clock::now();
        };
      
      if (!node.Subscribe(topicName, cb)) {
        cout << "Failed to subscribe to topic: " << topicName << endl;
      } else {
        cout << "Subscribed to camera topic: " << topicName << endl;
      }
    }
  }
  
  cout << "Camera initialization complete" << endl;
}

/**
 * Read camera images from Gazebo Harmonic.
 */
static void gazebo_read_video(void)
{
  // Video handling in Gazebo Harmonic is done through message passing
  // Images are received via callbacks registered with gz::transport::Node
  
  for (int i = 0; i < VIDEO_THREAD_MAX_CAMERAS; ++i) {
    if (cameras[i] == NULL) continue;
    
    auto &lastMsg = gazebo_cams[i].last_image_msg;
    if (lastMsg.width() == 0 || lastMsg.height() == 0) continue;
    
    // Convert gz::msgs::Image to image_t
    struct image_t img;
    
    // Determine camera type
    bool is_mt9f002 = (string(cameras[i]->dev_name) == "mt9f002");
    
    if (is_mt9f002) {
      image_create(&img, MT9F002_OUTPUT_WIDTH, MT9F002_OUTPUT_HEIGHT, IMAGE_YUV422);
    } else {
      image_create(&img, lastMsg.width(), lastMsg.height(), IMAGE_YUV422);
    }
    
    // Convert RGB to YUV422
    const unsigned char *data_rgb = 
      reinterpret_cast<const unsigned char*>(lastMsg.data().c_str());
    uint8_t *data_yuv = (uint8_t *)(img.buf);
    
    for (int x_yuv = 0; x_yuv < img.w; ++x_yuv) {
      for (int y_yuv = 0; y_yuv < img.h; ++y_yuv) {
        int x_rgb = x_yuv;
        int y_rgb = y_yuv;
        
        if (is_mt9f002) {
          x_rgb = (mt9f002.offset_x + ((float)x_yuv / img.w) * mt9f002.sensor_width)
              / CFG_MT9F002_PIXEL_ARRAY_WIDTH * lastMsg.width();
          y_rgb = (mt9f002.offset_y + ((float)y_yuv / img.h) * mt9f002.sensor_height)
              / CFG_MT9F002_PIXEL_ARRAY_HEIGHT * lastMsg.height();
        }
        
        int idx_rgb = 3 * (lastMsg.width() * y_rgb + x_rgb);
        int idx_yuv = 2 * (img.w * y_yuv + x_yuv);
        int idx_px = img.w * y_yuv + x_yuv;
        
        if (idx_px % 2 == 0) {
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
    
    // Set timestamp from message
    auto sec = lastMsg.header().stamp().sec();
    auto nsec = lastMsg.header().stamp().nsec();
    img.ts.tv_sec = sec;
    img.ts.tv_usec = nsec / 1000;
    img.pprz_ts = (sec + nsec / 1e9) * 1e6;
    img.buf_idx = 0;

#if NPS_DEBUG_VIDEO
    cv::Mat RGB_cam(lastMsg.height(), lastMsg.width(), CV_8UC3, 
                    (uint8_t *)data_rgb);
    cv::cvtColor(RGB_cam, RGB_cam, cv::COLOR_RGB2BGR);
    cv::namedWindow(cameras[i]->dev_name, cv::WINDOW_AUTOSIZE);
    cv::imshow(cameras[i]->dev_name, RGB_cam);
    cv::waitKey(1);
#endif

    cv_run_device(cameras[i], &img);
    image_free(&img);
  }
}
#endif // NPS_SIMULATE_VIDEO

#if NPS_SIMULATE_LASER_RANGE_ARRAY
/*
 * Range sensor simulation for Gazebo Harmonic
 */

struct gazebo_ray_t {
  gz::sensors::DistanceSensor *sensor;
  std::chrono::steady_clock::time_point last_measurement_time;
  double last_range;
};

static struct gazebo_ray_t rays[LASER_RANGE_ARRAY_NUM_SENSORS] = {{NULL, std::chrono::steady_clock::time_point(), 0.0}};
static float range_orientation[] = LASER_RANGE_ARRAY_ORIENTATIONS;
static uint8_t ray_sensor_agl_index = 255;

#define VL53L0_MAX_VAL 8.191f

/**
 * Initialize range sensors in Gazebo Harmonic.
 */
static void gazebo_init_range_sensors(void)
{
  cout << "Initializing range sensors..." << endl;
  
  // In Gazebo Harmonic, sensors are accessed via topics
  // Subscribe to range/distance sensor topics
  
  for (int i = 0; i < LASER_RANGE_ARRAY_NUM_SENSORS; i++) {
    string sensorName = "range_sensor_" + to_string(i);
    string topicName = "/" + string(NPS_GAZEBO_AC_NAME) + "/" + sensorName;
    
    std::function<void(const gz::msgs::LaserScan &)> cb =
      [i](const gz::msgs::LaserScan &_msg) {
        if (_msg.ranges_size() > 0) {
          rays[i].last_range = _msg.ranges(0);
          rays[i].last_measurement_time = std::chrono::steady_clock::now();
        }
      };
    
    if (node.Subscribe(topicName, cb)) {
      cout << "Subscribed to range sensor: " << topicName << endl;
    }
    
    // Check for sensor pointing down for AGL
#if LASER_RANGE_ARRAY_SEND_AGL
    if (fabs(range_orientation[i * 2] + M_PI_2) < RadOfDeg(5)) {
      ray_sensor_agl_index = i;
    }
#endif
  }
  
  cout << "Range sensors initialized" << endl;
}

/**
 * Read range sensor data and send ABI messages.
 */
static void gazebo_read_range_sensors(void)
{
  auto now = std::chrono::steady_clock::now();
  
  for (int i = 0; i < LASER_RANGE_ARRAY_NUM_SENSORS; i++) {
    // Check if we have a recent measurement
    auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - rays[i].last_measurement_time).count();
    
    if (timeDiff > 100) { // Skip if older than 100ms
      continue;
    }
    
    float range = rays[i].last_range;
    
    // Clamp range to valid values
    if (range < 1e-5 || range > VL53L0_MAX_VAL || std::isinf(range)) {
      range = VL53L0_MAX_VAL;
    }
    
    // Send obstacle detection message
    AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_RANGE_ARRAY_NPS_ID, range, 
                                 range_orientation[i * 2],
                                 range_orientation[i * 2 + 1]);
    
    // Send AGL if this is the down-facing sensor
    if (i == ray_sensor_agl_index) {
      uint32_t now_ts = get_sys_time_usec();
      if (range > 1e-5 && !std::isinf(range)) {
        AbiSendMsgAGL(AGL_RAY_SENSOR_GAZEBO_ID, now_ts, range);
      }
    }
  }
}
#endif // NPS_SIMULATE_LASER_RANGE_ARRAY