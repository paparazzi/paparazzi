/*
 * ins_qkf_predict.cpp
 *
 *  Created on: Sep 2, 2009
 *      Author: Jonathan Brandmeyer

 *          This file is part of libeknav.
 *
 *  Libeknav is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 3.
 *
 *  Libeknav is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with libeknav.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * This is a modified version by Martin Dieblich
 * The general changes are a more consequent naming
 * 
 */


#include "ins_qkf.hpp"
#include "assertions.hpp"
#ifdef TIME_OPS
//#include "timer.hpp"
#include <iostream>
#endif

using namespace Eigen;

namespace {

Matrix<double, 3, 3>
axis_scale(const Vector3d& axis, double scale)
{
	return (scale - 1) * axis * axis.transpose() + Matrix3d::Identity();
}

void
linear_predict(basic_ins_qkf& _this, const Vector3d& gyro_meas, const Vector3d& accel_meas, double dt)
{
	// The two components of rotation that do not spin about the gravity vector
	// have an influence on the position and velocity of the vehicle.
	// Let r be an error axis of rotation, and z be the gravity vector.
	// Increasing r creates increasing error in the direction _|_ to r and z.
	// By the small angle theorem, the amount of error is ~ abs(r)*abs(z).
	// Increasing r also creates increasing error in the direction || to -z.
	// By the small angle theorem, the amount of error is ~ zero.
	// Therefore, rotate the error block about the z axis by -90 degrees, and
	// zero out the error vector in the z direction.
	// accel_cov is the relationship between error vectors in the tangent space
	// of the vehicle orientation and the translational reference frame.
	
	
	Matrix3d rot = _this.avg_state.orientation.conjugate().toRotationMatrix();
	
	Vector3d accel_ecef = rot*accel_meas;		// a_e = (q_e2b)^* x a_b = q_b2e x a_b
	Vector3d accel_gravity = _this.avg_state.position.normalized()*(-9.81);         
	Vector3d accel_resid = accel_ecef + accel_gravity;								// a = (xdd-g)+g = xdd;
	
	#if 0
	printf("==================================================\n");
	printf("Quaternion: % 1.4f % 1.4f % 1.4f % 1.4f\n",
														_this.avg_state.orientation.w(),
														_this.avg_state.orientation.x(),
														_this.avg_state.orientation.y(),
														_this.avg_state.orientation.z());
	printf("Accel_meas: % 1.4f % 1.4f % 1.4f\n", accel_meas(0), accel_meas(1), accel_meas(2));
	printf("Accel_ecef: % 1.4f % 1.4f % 1.4f\n", accel_ecef(0), accel_ecef(1), accel_ecef(2));
	printf("Accel_grav: % 1.4f % 1.4f % 1.4f\n", accel_gravity(0), accel_gravity(1), accel_gravity(2));
	#endif
	
#if 0
	// This form works well with zero static acceleration.
	Matrix<double, 3, 3> accel_cov =
		Eigen::AngleAxisd(-M_PI*0.5, _this.avg_state.position.normalized())
		* axis_scale(_this.avg_state.position.normalized(), 0) * 9.81;
#elif 1
	Matrix<double, 3, 3> accel_cov =
		Eigen::AngleAxisd(-M_PI*0.5, accel_ecef.normalized())
		* axis_scale(accel_ecef.normalized(), 0) * accel_meas.norm();
#else
	// The following form ends up being identical to the simpler one
	// above
	Matrix<double, 3, 3> accel_cov =
		Eigen::AngleAxisd(-M_PI*0.5, _this.avg_state.position.normalized())
		* axis_scale(_this.avg_state.position.normalized(), 0) * 9.81
		+ Eigen::AngleAxisd(-M_PI*0.5, accel_resid.normalized())
		* axis_scale(accel_resid.normalized(), 0)*accel_resid.norm();
#endif
	// TODO: Optimization opportunity: the accel_cov doesn't change much over
	// the life of a mission. Precompute it once and then retain the original.
	// Then, only one 3x3 block ever gets updated in the A matrix below.

	// The linearized Kalman state projection matrix.
#if 0
	Matrix<double, 12, 12> A;
	     // gyro bias row
	A << Matrix<double, 3, 3>::Identity(), Matrix<double, 3, 9>::Zero(),
		 // Orientation row
		 _this.avg_state.orientation.conjugate().toRotationMatrix()*-dt,
			 Matrix<double, 3, 3>::Identity(), Matrix<double, 3, 6>::Zero(),
		 // Position row
		 Matrix<double, 3, 3>::Zero(), -accel_cov*0.5*dt*dt,
			 Matrix<double, 3, 3>::Identity(), Matrix<double, 3, 3>::Identity()*dt,
		 // Velocity row
		 Matrix<double, 3, 3>::Zero(), -accel_cov * dt,
			 Matrix<double, 3, 3>::Zero(), Matrix<double, 3, 3>::Identity();

	// 800x realtime, with vectorization
	_this.cov.part<Eigen::SelfAdjoint>() = A * _this.cov * A.transpose();
#else
	// 1500x realtime, without vectorization, on 2.2 GHz Athlon X2
	const Matrix<double, 12, 12> cov = _this.cov;
	const Matrix3d dtR = dt * _this.avg_state.orientation.conjugate().toRotationMatrix();
	const Matrix3d dtQ = accel_cov * dt;

	_this.cov.block<3, 3>(0, 3) -= cov.block<3,3>(0, 0)*dtR.transpose();
	_this.cov.block<3, 3>(0, 6) += dt * cov.block<3, 3>(0, 9);
	_this.cov.block<3, 3>(0, 9) -= cov.block<3, 3>(0, 3) * dtQ.transpose();
	_this.cov.block<3, 3>(3, 3).part<Eigen::SelfAdjoint>() += dtR*cov.block<3, 3>(0, 0)*dtR.transpose()
			- dtR*cov.block<3, 3>(0, 3) - cov.block<3, 3>(3, 0)*dtR.transpose();
	_this.cov.block<3, 3>(3, 6) += -dtR * (cov.block<3, 3>(0, 6) + dt*cov.block<3, 3>(0, 9))
			+ dt*cov.block<3, 3>(3, 9);
	_this.cov.block<3, 3>(3, 9) += -dtR*( -cov.block<3, 3>(0, 3)*dtQ.transpose() + cov.block<3, 3>(0, 9))
			- cov.block<3, 3>(3, 3)*dtQ.transpose();
	_this.cov.block<3, 3>(6, 6).part<Eigen::SelfAdjoint>() += dt*cov.block<3, 3>(6, 9) + dt*dt*cov.block<3, 3>(9, 9)
			+ dt*cov.block<3, 3>(9, 6);
	_this.cov.block<3, 3>(6, 9) += -cov.block<3, 3>(6, 3)*dtQ.transpose() + dt*cov.block<3, 3>(9, 9)
			- dt*cov.block<3, 3>(9, 3)*dtQ.transpose();
	_this.cov.block<3, 3>(9, 9).part<Eigen::SelfAdjoint>() += dtQ*cov.block<3, 3>(3, 3)*dtQ.transpose()
			- dtQ*cov.block<3, 3>(3, 9) - cov.block<3, 3>(9, 3)*dtQ.transpose();

	_this.cov.block<3, 3>(3, 0) = _this.cov.block<3, 3>(0, 3).transpose();
	_this.cov.block<3, 3>(6, 0) = _this.cov.block<3, 3>(0, 6).transpose();
	_this.cov.block<3, 3>(6, 3) = _this.cov.block<3, 3>(3, 6).transpose();
	_this.cov.block<3, 3>(9, 0) = _this.cov.block<3, 3>(0, 9).transpose();
	_this.cov.block<3, 3>(9, 3) = _this.cov.block<3, 3>(3, 9).transpose();
	_this.cov.block<3, 3>(9, 6) = _this.cov.block<3, 3>(6, 9).transpose();
#endif

	_this.cov.block<3, 3>(0, 0) += _this.gyro_stability_noise.asDiagonal() * dt;
	_this.cov.block<3, 3>(3, 3) += _this.gyro_white_noise.asDiagonal() * dt;
	_this.cov.block<3, 3>(6, 6) += _this.accel_white_noise.asDiagonal() * 0.5*dt*dt;
	_this.cov.block<3, 3>(9, 9) += _this.accel_white_noise.asDiagonal() * dt;

	Quaterniond orientation = exp<double>((gyro_meas - _this.avg_state.gyro_bias) * dt)
			* _this.avg_state.orientation;
	//Vector3d accel = accel_ecef - _this.avg_state.position.normalized() * 9.81;
	//std::cout << "   ACCEL______(" << accel.transpose() << ")\n";
	Vector3d position = _this.avg_state.position + _this.avg_state.velocity * dt + 0.5*accel_resid*dt*dt;
	Vector3d velocity = _this.avg_state.velocity + accel_resid*dt;

	_this.avg_state.position = position;
	_this.avg_state.velocity = velocity;
	_this.avg_state.orientation = orientation;
}

} // !namespace (anon)

void
basic_ins_qkf::predict(const Vector3d& gyro_meas, const Vector3d& accel_meas, double dt)
{
#ifdef TIME_OPS
	timer clock;
	clock.start();
#endif

	// Always use linearized prediction
	linear_predict(*this, gyro_meas, accel_meas, dt);

#ifdef TIME_OPS
	double time = clock.stop()*1e6;
	std::cout << "unscented predict time: " << time << "\n";
#endif
}

