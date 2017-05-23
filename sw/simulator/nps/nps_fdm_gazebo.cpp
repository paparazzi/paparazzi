/*
 * Copyright (C) 2017 Tom van Dijk <tomvand@users.noreply.github.com>
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

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include "nps_fdm.h"
#include "math/pprz_algebra_double.h"

#include "generated/airframe.h"
#include "autopilot.h"

using namespace std;

#ifndef GAZEBO_WORLD
#define GAZEBO_WORLD "ardrone.world"
#endif
#ifndef GAZEBO_AC_NAME
#define GAZEBO_AC_NAME "paparazzi_uav"
#endif

#ifdef NPS_SIMULATE_VIDEO
#include "modules/computer_vision/video_thread_nps.h"
static bool video_initialized = false;
static void init_gazebo_video(void);
static void dummy_callback(ConstImageStampedPtr &msg);
#endif

/// Holds all necessary NPS FDM state information
struct NpsFdm fdm;

// Pointer to Gazebo data
static gazebo::physics::ModelPtr model = NULL;

static void set_irrelevant_data(double dt);
static void init_gazebo(void);
static void gazebo_read(void);
static void gazebo_write(double commands[], int commands_nb);

// Conversion routines
inline struct EcefCoor_d to_pprz_ecef(ignition::math::Vector3d ecef_i) {
	struct EcefCoor_d ecef_p;
	ecef_p.x = ecef_i.X();
	ecef_p.y = ecef_i.Y();
	ecef_p.z = ecef_i.Z();
	return ecef_p;
}

inline struct NedCoor_d to_pprz_ned(ignition::math::Vector3d global) {
	struct NedCoor_d ned;
	ned.x = global.Y();
	ned.y = global.X();
	ned.z = -global.Z();
	return ned;
}

inline struct LlaCoor_d to_pprz_lla(ignition::math::Vector3d lla_i) {
	struct LlaCoor_d lla_p;
	lla_p.lat = lla_i.X();
	lla_p.lon = lla_i.Y();
	lla_p.alt = lla_i.Z();
	return lla_p;
}

inline struct DoubleVect3 to_pprz_body(gazebo::math::Vector3 body_g) {
	struct DoubleVect3 body_p;
	body_p.x = body_g.x;
	body_p.y = -body_g.y;
	body_p.z = -body_g.z;
	return body_p;
}

inline struct DoubleRates to_pprz_rates(gazebo::math::Vector3 body_g) {
	struct DoubleRates body_p;
	body_p.p = body_g.x;
	body_p.q = -body_g.y;
	body_p.r = -body_g.z;
	return body_p;
}

inline struct DoubleEulers to_pprz_eulers(gazebo::math::Quaternion quat) {
	struct DoubleEulers eulers;
	eulers.psi = -quat.GetYaw() - M_PI / 2;
	eulers.theta = -quat.GetPitch();
	eulers.phi = quat.GetRoll();
	return eulers;
}

inline struct DoubleVect3 to_pprz_ltp(gazebo::math::Vector3 xyz) {
	struct DoubleVect3 ltp;
	ltp.x = xyz.y;
	ltp.y = xyz.x;
	ltp.z = -xyz.z;
	return ltp;
}

// External functions, interface with Paparazzi's NPS as declared in nps_fdm.h

void nps_fdm_init(double dt) {
	set_irrelevant_data(dt); // Not all fields in fdm are used.
	init_gazebo();
	gazebo_read();
}

void nps_fdm_run_step(bool launch, double *commands, int commands_nb) {
	static int test = 100;
#ifdef NPS_SIMULATE_VIDEO
	// Initialize here, as the video thread is not ready for use during fdm_init.
//	if (!video_initialized) {
//		init_gazebo_video();
//		video_initialized = true;
//	}
#endif
	try {
		cout << "Run WORLD @ " << model->GetWorld() << endl;
		gazebo::runWorld(model->GetWorld(), 1);
	} catch (...) {
		cout << "ERROR: runWorld caused exception!" << endl;
	}
	try {
		cout << "Run sensors... ";
		gazebo::sensors::run_once(); // <== CRASH!
		cout << "finished." << endl;
	} catch (...) {
		cout << "ERROR: sensor call resulted in exception!" << endl;
	}
	try {
		gazebo::sensors::SensorManager *mgr =
				gazebo::sensors::SensorManager::Instance();
		gazebo::sensors::CameraSensorPtr cam = std::static_pointer_cast
				< gazebo::sensors::CameraSensor
				> (mgr->GetSensor(
						"front_camera"));
		cout << "CameraSensorPtr: " << cam << endl;
		cout << "CameraSensor::IsActive(): " << cam->IsActive() << endl;
		cout << "CameraSensor::UpdateRate(): " << cam->UpdateRate() << endl;
		cout << "CameraSensor::LastMeasurementTime(): "
				<< cam->LastMeasurementTime() << endl;
		cout << "CameraSensor::LastUpdateTime(): " << cam->LastUpdateTime()
				<< endl;
		cout << "CameraPtr: " << cam->Camera() << endl;
		cout << "ImageDataPtr: " << (void*)(cam->ImageData()) << endl;
		cout << endl;
	} catch (...) {
		cout << "ERROR: Cam readout caused exception!" << endl;
	}
	try {
		gazebo_write(commands, commands_nb);
	} catch (...) {
		cout << "ERROR: gazebo_write caused exception" << endl;
	}
	try {
		gazebo_read();
	} catch (...) {
		cout << "ERROR: gazebo_read caused exception!" << endl;
	}
}
void nps_fdm_set_wind(double speed, double dir) {
}
void nps_fdm_set_wind_ned(
		double wind_north,
		double wind_east,
		double wind_down) {
}
void nps_fdm_set_turbulence(double wind_speed, int turbulence_severity) {
}
/** Set temperature in degrees Celcius at given height h above MSL */
void nps_fdm_set_temperature(double temp, double h) {
}

// Internal functions
static void set_irrelevant_data(double dt) {
	fdm.init_dt = dt; // JSBsim specific
	fdm.curr_dt = dt; // JSBsim specific
	fdm.nan_count = 0; // JSBsim specific
}

static void init_gazebo(void) {
	string gazebo_home = "/conf/simulator/gazebo/";
	string pprz_home(getenv("PAPARAZZI_HOME"));
	string gazebodir = pprz_home + gazebo_home;
	cout << "Gazebo directory: " << gazebodir << endl;

	if (!gazebo::setupServer(0, NULL)) {
		cout << "Failed to start Gazebo, exiting." << endl;
		std::exit(-1);
	}

	cout << "Load world: " << gazebodir + "world/" + GAZEBO_WORLD << endl;
	gazebo::physics::WorldPtr world = gazebo::loadWorld(
			gazebodir + "world/" + GAZEBO_WORLD);
	if (!world) {
		cout << "Failed to open world, exiting." << endl;
		std::exit(-1);
	}
	cout << "WORLD @ " << world << endl;

	cout << "Get pointer to aircraft: " << GAZEBO_AC_NAME << endl;
	model = world->GetModel(GAZEBO_AC_NAME);
	if (!model) {
		cout << "Failed to find '" << GAZEBO_AC_NAME << "', exiting." << endl;
		std::exit(-1);
	}
	cout << "MODEL @ " << model << endl;

	// Rendering engine settings
//	cout << "Render path type: "
//			<< gazebo::rendering::RenderEngine::Instance()->GetRenderPathType()
//			<< endl;

	// Initialize sensors
	try {
		gazebo::sensors::run_once(true);
	} catch (...) {
		cout << "ERROR: Initialization run_once(true)!" << endl;
	}
	try {
		gazebo::sensors::run_threads();
	} catch (...) {
		cout << "ERROR: Initialization run_threads()!" << endl;
	}
	gazebo::runWorld(world, 1);
	cout << "Sensors initialized..." << endl;

	cout << "Gazebo initialized successfully!" << endl;
}

#ifdef NPS_SIMULATE_VIDEO
static void init_gazebo_video(void) {
	// Prepare to subscribe dummy callback (see below)
//	gazebo::transport::NodePtr node(new gazebo::transport::Node());
//	node->Init();

	// Add dummy callbacks to all Gazebo cameras
	// Might fix unknown bug that causes Gazebo to crash
	// or cameras to withold images that occurs otherwise...
	// Nope, does not solve crashes.
	// Btw, callback can also take class method, which might be
	// an alternative to polling once the crashes are solved...
	gazebo::sensors::SensorManager *mgr =
			gazebo::sensors::SensorManager::Instance();
	gazebo::sensors::Sensor_V sensors = mgr->GetSensors();
	cout << "Adding sensor callbacks..." << endl;
	for (auto& sensor : sensors) {
		if (sensor->Category() != gazebo::sensors::SensorCategory::IMAGE) continue;
		cout << sensor->Topic() << endl;
//		node->Subscribe(sensor->Topic(), dummy_callback);
	}

	cout << "Camera initialization finished." << endl;
}

static void dummy_callback(ConstImageStampedPtr &msg) {
}
#endif

static void gazebo_read(void) {
	gazebo::physics::WorldPtr world = model->GetWorld();
	gazebo::math::Pose pose = model->GetWorldPose(); // In LOCAL xyz frame
	gazebo::math::Vector3 vel = model->GetWorldLinearVel();
	gazebo::math::Vector3 accel = model->GetWorldLinearAccel();
	gazebo::math::Vector3 ang_vel = model->GetWorldAngularVel();
	gazebo::common::SphericalCoordinatesPtr sphere =
			world->GetSphericalCoordinates();
	gazebo::math::Quaternion local_to_global_quat(0, 0,
			-sphere->HeadingOffset().Radian());

	/* Fill FDM struct */
	fdm.time = world->GetSimTime().Double();

	// init_dt: unused
	// curr_dt: unused
	// on_ground: unused
	// nan_count: unused

	/* position */
	fdm.ecef_pos = to_pprz_ecef(
			sphere->PositionTransform(pose.pos.Ign(),
					gazebo::common::SphericalCoordinates::LOCAL,
					gazebo::common::SphericalCoordinates::ECEF));
	fdm.ltpprz_pos = to_pprz_ned(
			sphere->PositionTransform(pose.pos.Ign(),
					gazebo::common::SphericalCoordinates::LOCAL,
					gazebo::common::SphericalCoordinates::GLOBAL));
	fdm.lla_pos = to_pprz_lla(
			sphere->PositionTransform(pose.pos.Ign(),
					gazebo::common::SphericalCoordinates::LOCAL,
					gazebo::common::SphericalCoordinates::SPHERICAL));
	fdm.hmsl = pose.pos.z;
	/* debug positions */
	fdm.lla_pos_pprz = fdm.lla_pos; // Don't really care...
	fdm.lla_pos_geod = fdm.lla_pos;
	fdm.lla_pos_geoc = fdm.lla_pos;
	fdm.agl = pose.pos.z; // TODO Measure with sensor

	/* velocity */
	fdm.ecef_ecef_vel = to_pprz_ecef(
			sphere->VelocityTransform(vel.Ign(),
					gazebo::common::SphericalCoordinates::LOCAL,
					gazebo::common::SphericalCoordinates::ECEF));
	fdm.body_ecef_vel = to_pprz_body(pose.rot.RotateVectorReverse(vel)); // Note: unused
	fdm.ltp_ecef_vel = to_pprz_ned(
			sphere->VelocityTransform(vel.Ign(),
					gazebo::common::SphericalCoordinates::LOCAL,
					gazebo::common::SphericalCoordinates::GLOBAL));
	fdm.ltpprz_ecef_vel = fdm.ltp_ecef_vel; // ???

	/* acceleration */
	fdm.ecef_ecef_accel = to_pprz_ecef(
			sphere->VelocityTransform(accel.Ign(),
					gazebo::common::SphericalCoordinates::LOCAL,
					gazebo::common::SphericalCoordinates::ECEF)); // Note: unused
	fdm.body_ecef_accel = to_pprz_body(pose.rot.RotateVectorReverse(accel));
	fdm.ltp_ecef_accel = to_pprz_ned(
			sphere->VelocityTransform(accel.Ign(),
					gazebo::common::SphericalCoordinates::LOCAL,
					gazebo::common::SphericalCoordinates::GLOBAL)); // Note: unused
	fdm.ltpprz_ecef_accel = fdm.ltp_ecef_accel; // ???
	fdm.body_inertial_accel = fdm.body_ecef_accel; // Approximate, unused.
	fdm.body_accel = to_pprz_body(
			pose.rot.RotateVectorReverse(accel.Ign() - world->Gravity()));

	/* attitude */
	// ecef_to_body_quat: unused
	fdm.ltp_to_body_eulers = to_pprz_eulers(local_to_global_quat * pose.rot);
	double_quat_of_eulers(&(fdm.ltp_to_body_quat), &(fdm.ltp_to_body_eulers));
	fdm.ltpprz_to_body_quat = fdm.ltp_to_body_quat; // unused
	fdm.ltpprz_to_body_eulers = fdm.ltp_to_body_eulers; // unused

	/* angular velocity */
	fdm.body_ecef_rotvel = to_pprz_rates(pose.rot.RotateVectorReverse(ang_vel));
	fdm.body_inertial_rotvel = fdm.body_ecef_rotvel; // Approximate

	/* angular acceleration */
	// body_ecef_rotaccel: unused
	// body_inertial_rotaccel: unused
	/* misc */
	fdm.ltp_g = to_pprz_ltp(
			sphere->VelocityTransform(-1 * world->Gravity(),
					gazebo::common::SphericalCoordinates::LOCAL,
					gazebo::common::SphericalCoordinates::GLOBAL)); // unused
	fdm.ltp_h = to_pprz_ltp(
			sphere->VelocityTransform(world->MagneticField(),
					gazebo::common::SphericalCoordinates::LOCAL,
					gazebo::common::SphericalCoordinates::GLOBAL));

	/* atmosphere */
	// TODO after upgrade to gazebo 8!
	/* flight controls: unused */
	/* engine: unused */
}

static void gazebo_write(double commands[], int commands_nb) {
	const string names[] = NPS_ACTUATOR_NAMES;
	const double thrusts[] = { NPS_ACTUATOR_THRUSTS };
	const double torques[] = { NPS_ACTUATOR_TORQUES };

	for (int i = 0; i < commands_nb; ++i) {
		double thrust = autopilot.motors_on ? thrusts[i] * commands[i] : 0.0;
		double torque = autopilot.motors_on ? torques[i] * commands[i] : 0.0;
		gazebo::physics::LinkPtr link = model->GetLink(names[i]);
		link->AddRelativeForce(gazebo::math::Vector3(0, 0, thrust));
		link->AddRelativeTorque(gazebo::math::Vector3(0, 0, torque));
		// cout << "Motor '" << link->GetName() << "': thrust = " << thrust
		//		<< " N, torque = " << torque << " Nm" << endl;
	}
}

