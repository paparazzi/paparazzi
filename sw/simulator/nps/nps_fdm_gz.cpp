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
 * @file nps_fdm_gz.cpp
 * Flight Dynamics Model (FDM) for NPS using the new Gazebo (gz-sim).
 *
 * Port of the Gazebo Classic FDM (nps_fdm_gazebo.cpp) to gz-sim (Harmonic and
 * newer). It uses the same models, worlds and airframe defines.
 *
 * gz-sim only gives access to the simulation state from a System plugin. The
 * PaparazziSystem below is added to an embedded gz::sim::Server, which is
 * stepped from nps_fdm_run_step with a blocking RunOnce call in the same thread,
 * so the shared structs need no locking.
 *
 * Frames: the worlds keep the Gazebo Classic workaround of a 180 degree
 * heading offset in <spherical_coordinates> (Classic used a WSU frame). With
 * that offset the local x, y, z axes of a world are East, North, Up, as in
 * Gazebo Classic. The heading offset is applied here directly, so the result
 * does not depend on the (changing) frame conventions of gz-math.
 *
 * Not ported yet: cameras (NPS_SIMULATE_VIDEO), the laser range array
 * (NPS_SIMULATE_LASER_RANGE_ARRAY) and the sonar.
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <memory>
#include <csignal>
#include <chrono>

#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/MagneticField.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <sdf/sdf.hh>

extern "C" {
#include "nps_fdm.h"
#include "nps_autopilot.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "autopilot.h"

#include "math/pprz_isa.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_geodetic_double.h"
#include "filters/low_pass_filter.h"
#include "filters/high_pass_filter.h"
#include "state.h"
}

#ifndef NPS_GAZEBO_WORLD
#define NPS_GAZEBO_WORLD "empty.world"
#endif
#ifndef NPS_GAZEBO_AC_NAME
#define NPS_GAZEBO_AC_NAME "bebop"
#endif

/// Holds all necessary NPS FDM state information
struct NpsFdm fdm;

struct gazebo_actuators_t {
  std::string names[NPS_COMMANDS_NB];
  double thrusts[NPS_COMMANDS_NB];
  double torques[NPS_COMMANDS_NB];
  struct FirstOrderLowPass lowpass[NPS_COMMANDS_NB];
  struct FirstOrderHighPass highpass[NPS_COMMANDS_NB];
  double max_ang_momentum[NPS_COMMANDS_NB];
};

static struct gazebo_actuators_t gazebo_actuators = { NPS_ACTUATOR_NAMES, NPS_ACTUATOR_THRUSTS, NPS_ACTUATOR_TORQUES, { }, { }, { } };

/// Simulation state, written in PaparazziSystem::PostUpdate (gz local frame, FLU body)
static struct {
  gz::math::Pose3d pose;
  gz::math::Vector3d linear_vel;
  gz::math::Vector3d angular_vel;
  gz::math::Vector3d gravity;
  gz::math::Vector3d magnetic_field;
  double heading_offset;          ///< heading of <spherical_coordinates> [rad]
  double sim_time;                ///< [s]
  bool valid;
} gz_state;

/// Actuator commands in the body frame of each motor link, read in PaparazziSystem::PreUpdate
static struct {
  double thrusts[NPS_COMMANDS_NB];
  double torques[NPS_COMMANDS_NB];
} gz_cmd;

/**
 * gz-sim System that connects NPS with the simulation.
 * - Configure: find the aircraft model and its links
 * - PreUpdate: apply the motor forces and torques
 * - PostUpdate: read the state of the aircraft
 */
class PaparazziSystem : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override
  {
    world_ = entity;
    FindModel(ecm);
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
  {
    if (model_ == gz::sim::kNullEntity) {
      FindModel(ecm);
      return;
    }
    if (info.paused) {
      return;
    }
    for (int i = 0; i < NPS_COMMANDS_NB; i++) {
      gz::sim::Link link(motor_links_[i]);
      auto pose = link.WorldPose(ecm);
      if (!pose) {
        continue;
      }
      gz::math::Vector3d force = pose->Rot().RotateVector(gz::math::Vector3d(0, 0, gz_cmd.thrusts[i]));
      gz::math::Vector3d torque = pose->Rot().RotateVector(gz::math::Vector3d(0, 0, gz_cmd.torques[i]));
      link.AddWorldWrench(ecm, force, torque);
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override
  {
    if (model_ == gz::sim::kNullEntity) {
      return;
    }
    gz::sim::Link body(body_link_);
    auto pose = body.WorldPose(ecm);
    auto vel = body.WorldLinearVelocity(ecm);
    auto ang_vel = body.WorldAngularVelocity(ecm);
    if (!pose || !vel || !ang_vel) {
      return;
    }
    gz_state.pose = *pose;
    gz_state.linear_vel = *vel;
    gz_state.angular_vel = *ang_vel;

    auto gravity = ecm.Component<gz::sim::components::Gravity>(world_);
    gz_state.gravity = gravity ? gravity->Data() : gz::math::Vector3d(0, 0, -9.81);
    auto mag = ecm.Component<gz::sim::components::MagneticField>(world_);
    gz_state.magnetic_field = mag ? mag->Data() : gz::math::Vector3d::Zero;
    auto sphere = ecm.Component<gz::sim::components::SphericalCoordinates>(world_);
    gz_state.heading_offset = sphere ? sphere->Data().HeadingOffset().Radian() : M_PI;

    gz_state.sim_time = std::chrono::duration<double>(info.simTime).count();
    gz_state.valid = true;
  }

  bool Ready() const { return model_ != gz::sim::kNullEntity; }

private:
  void FindModel(gz::sim::EntityComponentManager &ecm)
  {
    gz::sim::Model model(gz::sim::World(world_).ModelByName(ecm, NPS_GAZEBO_AC_NAME));
    if (!model.Valid(ecm)) {
      return;
    }
    gz::sim::Entity motor_links[NPS_COMMANDS_NB];
    for (int i = 0; i < NPS_COMMANDS_NB; i++) {
      motor_links[i] = model.LinkByName(ecm, gazebo_actuators.names[i]);
      if (motor_links[i] == gz::sim::kNullEntity) {
        fprintf(stderr, "NPS gz: link '%s' not found in model '%s'\n",
                gazebo_actuators.names[i].c_str(), NPS_GAZEBO_AC_NAME);
        exit(EXIT_FAILURE);
      }
    }
    body_link_ = model.CanonicalLink(ecm);
    gz::sim::Link body(body_link_);
    body.EnableVelocityChecks(ecm, true);
    for (int i = 0; i < NPS_COMMANDS_NB; i++) {
      motor_links_[i] = motor_links[i];
    }
    model_ = model.Entity();
  }

  gz::sim::Entity world_{gz::sim::kNullEntity};
  gz::sim::Entity model_{gz::sim::kNullEntity};
  gz::sim::Entity body_link_{gz::sim::kNullEntity};
  gz::sim::Entity motor_links_[NPS_COMMANDS_NB];
};

static std::unique_ptr<gz::sim::Server> server;
static std::shared_ptr<PaparazziSystem> pprz_system;

static void init_gazebo(void);
static void gazebo_read(void);
static void gazebo_write(double act_commands[], int commands_nb);

/* Conversions. The local gz frame is ENU after rotating by (heading_offset - pi),
 * see the file description. Body frames: gz FLU, paparazzi FRD. */

static gz::math::Quaterniond local_to_enu(void)
{
  return gz::math::Quaterniond(0, 0, gz_state.heading_offset - M_PI);
}

static struct NedCoor_d ned_of_enu(const gz::math::Vector3d &enu)
{
  struct NedCoor_d ned = { enu.Y(), enu.X(), -enu.Z() };
  return ned;
}

static struct DoubleVect3 vect_ned_of_enu(const gz::math::Vector3d &enu)
{
  struct DoubleVect3 ned = { enu.Y(), enu.X(), -enu.Z() };
  return ned;
}

static struct DoubleVect3 frd_of_flu(const gz::math::Vector3d &flu)
{
  struct DoubleVect3 frd = { flu.X(), -flu.Y(), -flu.Z() };
  return frd;
}

static struct DoubleRates rates_frd_of_flu(const gz::math::Vector3d &flu)
{
  struct DoubleRates frd = { flu.X(), -flu.Y(), -flu.Z() };
  return frd;
}

/** NED to FRD euler angles from the ENU to FLU attitude */
static struct DoubleEulers eulers_ned_of_enu(const gz::math::Quaterniond &q_enu)
{
  struct DoubleEulers eulers;
  eulers.phi = q_enu.Roll();
  eulers.theta = -q_enu.Pitch();
  eulers.psi = M_PI_2 - q_enu.Yaw();
  eulers.psi = atan2(sin(eulers.psi), cos(eulers.psi));
  return eulers;
}

// External functions, interface with Paparazzi's NPS as declared in nps_fdm.h

void nps_fdm_init(double dt)
{
  fdm.init_dt = dt;
  fdm.curr_dt = dt;
  fdm.nan_count = 0;

#ifdef NPS_ACTUATOR_TIME_CONSTANTS
  // Low-pass filter to simulate the delayed actuator response
  const float tau[NPS_COMMANDS_NB] = NPS_ACTUATOR_TIME_CONSTANTS;
  for (int i = 0; i < NPS_COMMANDS_NB; i++) {
    init_first_order_low_pass(&gazebo_actuators.lowpass[i], tau[i], dt, 0.f);
  }
#ifdef NPS_ACTUATOR_MAX_ANGULAR_MOMENTUM
  // High-pass filter to simulate the spin-up torque
  const float Iwmax[NPS_COMMANDS_NB] = NPS_ACTUATOR_MAX_ANGULAR_MOMENTUM;
  for (int i = 0; i < NPS_COMMANDS_NB; i++) {
    init_first_order_high_pass(&gazebo_actuators.highpass[i], tau[i], dt, 0.f);
    gazebo_actuators.max_ang_momentum[i] = Iwmax[i];
  }
#endif
#endif
}

void nps_fdm_run_init_step(void)
{
  // nothing to do, gz is started at the first run step (video devices are added after init)
}

void nps_fdm_run_step(bool launch __attribute__((unused)), double *act_commands, int commands_nb)
{
  if (!server) {
    init_gazebo();
  }
  gazebo_write(act_commands, commands_nb);
  server->RunOnce(false);
  gazebo_read();
}

// Atmosphere functions are not implemented (same as Gazebo Classic)
void nps_fdm_set_wind(double speed __attribute__((unused)), double dir __attribute__((unused))) {}
void nps_fdm_set_wind_ned(double wind_north __attribute__((unused)), double wind_east __attribute__((unused)),
                          double wind_down __attribute__((unused))) {}
void nps_fdm_set_turbulence(double wind_speed __attribute__((unused)),
                            int turbulence_severity __attribute__((unused))) {}
void nps_fdm_set_temperature(double temp __attribute__((unused)), double h __attribute__((unused))) {}

/**
 * Start an embedded gz-sim server.
 *
 * The vehicle NPS_GAZEBO_AC_NAME is inserted in the world NPS_GAZEBO_WORLD, both
 * found in conf/simulator/gazebo and sw/ext/tudelft_gazebo_models, and the
 * merged world is written to var/gazebo.world (as with Gazebo Classic).
 */
static void init_gazebo(void)
{
  const char *home = getenv("PAPARAZZI_HOME");
  if (home == NULL) {
    fprintf(stderr, "NPS gz: PAPARAZZI_HOME is not set\n");
    exit(EXIT_FAILURE);
  }
  std::string pprz_home(home);
  const std::string dirs[] = { pprz_home + "/conf/simulator/gazebo/", pprz_home + "/sw/ext/tudelft_gazebo_models/" };

  std::string resource_path;
  for (const std::string &dir : dirs) {
    sdf::addURIPath("model://", dir + "models/");
    sdf::addURIPath("world://", dir + "world/");
    resource_path += dir + "models/:";
  }
  // model:// URIs in the merged world are resolved by gz-sim through this path
  const char *env_path = getenv("GZ_SIM_RESOURCE_PATH");
  if (env_path != NULL) {
    resource_path += env_path;
  }
  setenv("GZ_SIM_RESOURCE_PATH", resource_path.c_str(), 1);

  std::string vehicle_uri = std::string("model://") + NPS_GAZEBO_AC_NAME + "/" + NPS_GAZEBO_AC_NAME + ".sdf";
  std::string vehicle_file = sdf::findFile(vehicle_uri, false);
  sdf::SDFPtr vehicle_sdf(new sdf::SDF());
  sdf::init(vehicle_sdf);
  if (vehicle_file.empty() || !sdf::readFile(vehicle_file, vehicle_sdf)) {
    fprintf(stderr, "NPS gz: could not load vehicle '%s'\n", vehicle_uri.c_str());
    exit(EXIT_FAILURE);
  }

  std::string world_uri = std::string("world://") + NPS_GAZEBO_WORLD;
  std::string world_file = sdf::findFile(world_uri, false);
  sdf::SDFPtr world_sdf(new sdf::SDF());
  sdf::init(world_sdf);
  if (world_file.empty() || !sdf::readFile(world_file, world_sdf)) {
    fprintf(stderr, "NPS gz: could not load world '%s'\n", world_uri.c_str());
    exit(EXIT_FAILURE);
  }
  printf("NPS gz: vehicle %s, world %s\n", vehicle_file.c_str(), world_file.c_str());

  sdf::ElementPtr world = world_sdf->Root()->GetElement("world");
  world->InsertElement(vehicle_sdf->Root()->GetFirstElement());
  // One gz iteration per NPS step: use the NPS time step for the physics
  world->GetElement("physics")->GetElement("max_step_size")->Set(fdm.init_dt);
  std::string merged_file = pprz_home + "/var/gazebo.world";
  world_sdf->Write(merged_file);

  gz::sim::ServerConfig config;
  config.SetSdfFile(merged_file);
  server = std::make_unique<gz::sim::Server>(config);
  pprz_system = std::make_shared<PaparazziSystem>();
  if (!server->AddSystem(pprz_system).value_or(false)) {
    fprintf(stderr, "NPS gz: could not add the paparazzi system\n");
    exit(EXIT_FAILURE);
  }

  // Run the first iteration paused: creates the entities and configures the system
  server->RunOnce(true);
  if (!pprz_system->Ready()) {
    fprintf(stderr, "NPS gz: model '%s' not found in the world\n", NPS_GAZEBO_AC_NAME);
    exit(EXIT_FAILURE);
  }

  // gz-sim catches SIGINT and SIGTERM to stop its own run loop, which NPS does not use:
  // restore the default handlers so Ctrl-C and the Paparazzi Center can stop the simulator
  signal(SIGINT, SIG_DFL);
  signal(SIGTERM, SIG_DFL);

  // Motor directions as defined by the motor mixing
#ifdef MOTOR_MIXING_YAW_COEF
  const double yaw_coef[] = MOTOR_MIXING_YAW_COEF;
  for (int i = 0; i < NPS_COMMANDS_NB; i++) {
    if (yaw_coef[i] != 0.) {
      double sign = yaw_coef[i] > 0. ? -1. : 1.;
      gazebo_actuators.torques[i] = sign * fabs(gazebo_actuators.torques[i]);
      gazebo_actuators.max_ang_momentum[i] = sign * fabs(gazebo_actuators.max_ang_momentum[i]);
    }
  }
#endif

#if NPS_SIMULATE_VIDEO
  fprintf(stderr, "NPS gz: camera simulation is not supported yet, no images\n");
#endif
#if NPS_SIMULATE_LASER_RANGE_ARRAY
  fprintf(stderr, "NPS gz: laser range array simulation is not supported yet\n");
#endif
}

/**
 * Fill the fdm struct from the gz state.
 *
 * As with Gazebo Classic, the atmosphere is not simulated and the AHRS and INS
 * should be bypassed (NPS_BYPASS_AHRS, NPS_BYPASS_INS).
 */
static void gazebo_read(void)
{
  static gz::math::Vector3d vel_prev;
  static double time_prev = -1.;

  if (!gz_state.valid) {
    return;
  }
  const gz::math::Pose3d &pose = gz_state.pose;
  const gz::math::Vector3d &vel = gz_state.linear_vel;
  gz::math::Quaterniond to_enu = local_to_enu();

  fdm.time = gz_state.sim_time;

  // World acceleration from the velocity (as the Gazebo IMU does)
  double dt = fdm.time - time_prev;
  gz::math::Vector3d accel = (time_prev >= 0. && dt > 0.) ? (vel - vel_prev) / dt : gz::math::Vector3d::Zero;
  vel_prev = vel;
  time_prev = fdm.time;

  // ltp definition in double for accuracy
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
  fdm.ltpprz_pos = ned_of_enu(to_enu.RotateVector(pose.Pos()));
  fdm.hmsl = -fdm.ltpprz_pos.z;
  ecef_of_ned_point_d(&fdm.ecef_pos, &ltpdef_d, &fdm.ltpprz_pos);
  lla_of_ecef_d(&fdm.lla_pos, &fdm.ecef_pos);
  fdm.lla_pos_pprz = fdm.lla_pos;
  fdm.lla_pos_geod = fdm.lla_pos;
  fdm.lla_pos_geoc = fdm.lla_pos;
  fdm.lla_pos_geoc.lat = gc_of_gd_lat_d(fdm.lla_pos.lat, fdm.hmsl);
  fdm.agl = pose.Pos().Z(); // TODO sonar

  /* velocity */
  fdm.ltp_ecef_vel = ned_of_enu(to_enu.RotateVector(vel));
  fdm.ltpprz_ecef_vel = fdm.ltp_ecef_vel;
  fdm.body_ecef_vel = frd_of_flu(pose.Rot().RotateVectorReverse(vel));
  ecef_of_ned_vect_d(&fdm.ecef_ecef_vel, &ltpdef_d, &fdm.ltp_ecef_vel);

  /* acceleration */
  fdm.ltp_ecef_accel = ned_of_enu(to_enu.RotateVector(accel));
  fdm.ltpprz_ecef_accel = fdm.ltp_ecef_accel;
  fdm.body_ecef_accel = frd_of_flu(pose.Rot().RotateVectorReverse(accel));
  fdm.body_inertial_accel = fdm.body_ecef_accel;
  fdm.body_accel = frd_of_flu(pose.Rot().RotateVectorReverse(accel - gz_state.gravity));
  ecef_of_ned_vect_d(&fdm.ecef_ecef_accel, &ltpdef_d, &fdm.ltp_ecef_accel);

  /* attitude */
  fdm.ltp_to_body_eulers = eulers_ned_of_enu(to_enu * pose.Rot());
  double_quat_of_eulers(&fdm.ltp_to_body_quat, &fdm.ltp_to_body_eulers);
  fdm.ltpprz_to_body_quat = fdm.ltp_to_body_quat;
  fdm.ltpprz_to_body_eulers = fdm.ltp_to_body_eulers;

  /* angular velocity */
  fdm.body_ecef_rotvel = rates_frd_of_flu(pose.Rot().RotateVectorReverse(gz_state.angular_vel));
  fdm.body_inertial_rotvel = fdm.body_ecef_rotvel;

  /* gravity and magnetic field */
  fdm.ltp_g = vect_ned_of_enu(to_enu.RotateVector(-gz_state.gravity));
  fdm.ltp_h = vect_ned_of_enu(to_enu.RotateVector(gz_state.magnetic_field));

  /* atmosphere: not simulated */
  fdm.wind = (struct DoubleVect3) {0, 0, 0};
  fdm.pressure_sl = PPRZ_ISA_SEA_LEVEL_PRESSURE;
  fdm.airspeed = 0;
  fdm.pressure = pprz_isa_pressure_of_height(fdm.hmsl, fdm.pressure_sl);
  fdm.dynamic_pressure = fdm.pressure_sl;
  fdm.temperature = 20.0;
  fdm.aoa = 0;
  fdm.sideslip = 0;

  /* flight controls and engines: unused */
  fdm.elevator = 0;
  fdm.flap = 0;
  fdm.left_aileron = 0;
  fdm.right_aileron = 0;
  fdm.rudder = 0;
  fdm.num_engines = 0;
}

/**
 * Compute the motor thrusts and torques from the normalized commands [0, 1],
 * with NPS_ACTUATOR_THRUSTS [N] and NPS_ACTUATOR_TORQUES [Nm] from the airframe.
 */
static void gazebo_write(double act_commands[], int commands_nb)
{
  for (int i = 0; i < commands_nb && i < NPS_COMMANDS_NB; ++i) {
    double sp = autopilot.motors_on ? act_commands[i] : 0.0;

#ifdef NPS_ACTUATOR_TIME_CONSTANTS
    double u = update_first_order_low_pass(&gazebo_actuators.lowpass[i], sp);
#else
    double u = sp;
#endif
    double thrust = gazebo_actuators.thrusts[i] * u;
    double torque = gazebo_actuators.torques[i] * u;

#ifdef NPS_ACTUATOR_MAX_ANGULAR_MOMENTUM
    double udot = update_first_order_high_pass(&gazebo_actuators.highpass[i], sp);
    torque += gazebo_actuators.max_ang_momentum[i] / (2.0 * sqrt(u > 0.05 ? u : 0.05)) * udot;
#endif

    gz_cmd.thrusts[i] = thrust;
    gz_cmd.torques[i] = torque;
  }
}
