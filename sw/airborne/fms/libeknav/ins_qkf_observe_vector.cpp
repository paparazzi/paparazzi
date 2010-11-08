/*
 * ins_qkf_observe_vector.cpp
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

#include "ins_qkf.hpp"
#include "assertions.hpp"

#ifdef TIME_OPS
#include "timer.hpp"
#include <iostream>
#endif

using namespace Eigen;
#define RANK_ONE_UPDATES

#include <Eigen/LU>

void
basic_ins_qkf::obs_gyro_bias(const Vector3d& bias, const Vector3d& bias_error)
{
	Matrix<double, 12, 3> kalman_gain = cov.block<12, 3>(0, 0) 
			* (cov.block<3, 3>(0, 0) + bias_error.asDiagonal()).inverse();
	cov -= kalman_gain * cov.block<3, 12>(0, 0);
	Vector3d innovation = bias - avg_state.gyro_bias;

	// Apply the Kalman gain to obtain the posterior state and error estimates.
	avg_state.apply_kalman_vec_update(kalman_gain * innovation);
}

void
basic_ins_qkf::obs_vector(const Vector3d& ref, const Vector3d& obs, double error)
{
#ifdef TIME_OPS
	timer clock;
	clock.start();
#endif
#define DEBUG_VECTOR_OBS 0
	// Optimization opportunity: re-use sigma points from the state prediction
#if 0
	std::vector<vector_obs_state, aligned_allocator<vector_obs_state> > points;
	Matrix<double, 15, 30> state_errors;
	decompose_sigma_points(points, state_errors, error);

	std::vector<Quaterniond> projected_points;
	projected_points.reserve(points.size());
	for (auto i = points.begin(), end = points.end(); i != end; ++i) {
		projected_points.push_back(observe_vector(*i, ref));
	}

	Vector3d expected_obs = avg_state.orientation * ref;
	Quaterniond expected_observation_prediction = Quaterniond().setFromTwoVectors(ref, expected_obs);
	// Compute the observation error matrix and its self-covariance
	Quaterniond mean_observation_pred = quaternion_avg_johnson(projected_points);


	// assert(obs_projection.transpose().isUnitary());
#if DEBUG_VECTOR_OBS
	std::cout << "\n\nref: " << ref.transpose();
	std::cout << "\nobs: " << obs.transpose();
	std::cout << "\nexpected observation: " << expected_observation_prediction.coeffs().transpose();
	std::cout << "\nUKF observation: " << mean_observation_pred.coeffs().transpose();
	std::cout << "\nexpected innovation: " << log<double>(Quaterniond().setFromTwoVectors(
		expected_obs, obs)).transpose();
	std::cout << "\ntangent space innovation: " << log<double>(avg_state.orientation.conjugate() *
		Quaterniond().setFromTwoVectors(expected_obs, obs) * avg_state.orientation) << "\n";
#endif
	Matrix<double, 3, Dynamic> obs_errors(3, projected_points.size());
	for (unsigned i = 0; i != projected_points.size(); ++i) {
		if (projected_points[i].coeffs().dot(mean_observation_pred.coeffs()) < 0) {
			// Choose the point on the same hemisphere as the mean.  otherwise, the error vectors
			// get screwed up.
			projected_points[i].coeffs() *= -1;
		}
		obs_errors.col(i) = log<double>(mean_observation_pred.conjugate() * projected_points[i]);
	}

	// Construct an observation matrix composed of the two unit vectors
	// orthogonal to the direction that pivots about the expected observation
	// Vector3d obs = obs.normalized();
	Matrix<double, 2, 3> obs_projection;
#if 1
	SelfAdjointEigenSolver<Matrix<double, 3, 3> > obs_cov((obs_errors * obs_errors.transpose())*0.5);
	obs_projection = obs_cov.eigenvectors().block<3, 2>(0, 1).transpose();
#else
	typedef Vector3d vector_t;
	obs_projection.row(0) = expected_obs.cross((expected_obs.dot(vector_t::UnitX()) < 0.707)
				? vector_t::UnitX() : vector_t::UnitY()).normalized();
	obs_projection.row(1) = expected_obs.cross(obs_projection.row(0));
#endif
	assert(!hasNaN(obs_projection));
#if DEBUG_VECTOR_OBS
	std::cout << "predicted obs: " << expected_obs.transpose() << "\n";
	std::cout << "actual obs: " << obs.transpose() << "\n";
	std::cout << "reference obs: " << ref.transpose() << "\n";
	std::cout << "obs cov eigenvalues: " << obs_cov.eigenvalues().transpose() << "\n";
	std::cout << "obs cov eigenvectors:\n" << obs_cov.eigenvectors() << "\n";
	std::cout << "measurement subspace:\n" << obs_projection.transpose() << "\n";
#endif

	Matrix<double, 2, Dynamic> projected_obs_errors = obs_projection*obs_errors;
	Matrix<double, 2, 2> obs_pred_cov = (projected_obs_errors * projected_obs_errors.transpose())*0.5;
#if DEBUG_VECTOR_OBS
	std::cout << "obs errors: " << obs_errors << "\n";
	std::cout << "S: " << obs_pred_cov << "\n";
	std::cout << "expected S: " << cov.block<3, 3>(3, 3) << "\n";
#endif

	Matrix<double, 12, 2> state_meas_cross_cov =
			(state_errors.block<12, 30>(0,0) * projected_obs_errors.transpose())*0.5;
	Matrix<double, 12, 2> kalman_gain = state_meas_cross_cov * obs_pred_cov.inverse();
	// Innovation rotation in the tangent space at the mean
	Vector2d innovation = obs_projection*log<double>(
			Quaterniond().setFromTwoVectors((mean_observation_pred * ref), obs));
#if DEBUG_VECTOR_OBS
	std::cout << "inverse(S): " << obs_pred_cov.inverse() << "\n";
	std::cout << "used innovation: " << log<double>(Quaterniond().setFromTwoVectors(
		mean_observation_pred * ref, obs)).transpose() << "\n";
	std::cout << "projected innovation: " << innovation.transpose() << "\n";
	std::cout << "deprojected innovation: " << (obs_projection.transpose() * innovation).transpose() << "\n";
#endif
	// actually, cov -= K*S*K^T, but this saves a multiplication.
	cov -= (kalman_gain * obs_pred_cov * kalman_gain.transpose());
#else
	// BIG optimization opportunity: Use a pseudo-linear measurement model.

	Vector3d obs_ref = avg_state.orientation.conjugate()*obs;
	Vector3d v_residual = log<double>(Quaterniond().setFromTwoVectors(ref, obs_ref));

	Matrix<double, 3, 2> h_trans;
	h_trans.col(0) = ref.cross(
			(abs(ref.dot(obs_ref)) < 0.9994) ? obs_ref :
				(abs(ref.dot(Vector3d::UnitX())) < 0.707)
				? Vector3d::UnitX() : Vector3d::UnitY()).normalized();
  
	h_trans.col(1) = -ref.cross(h_trans.col(0));
	assert(!hasNaN(h_trans));
	assert(h_trans.isUnitary());
	Vector2d innovation = h_trans.transpose() * v_residual;
#ifdef RANK_ONE_UPDATES
	// Running a rank-one update here is a strict win.
	Matrix<double, 12, 1> update = Matrix<double, 12, 1>::Zero();
	for (int i = 0; i < 2; ++i) {
		double obs_error = error;
		double obs_cov = (h_trans.col(i).transpose() * cov.block<3, 3>(3, 3) * h_trans.col(i))[0];
		Matrix<double, 12, 1> gain = cov.block<12, 3>(0, 3) * h_trans.col(i) / (obs_error + obs_cov);
		update += gain * h_trans.col(i).transpose() * v_residual;
		cov -= gain * h_trans.col(i).transpose() * cov.block<3, 12>(3, 0);
	}
#else
	Matrix<double, 12, 2> kalman_gain = cov.block<12, 3>(0, 3) * h_trans
			* (h_trans.transpose() * cov.block<3, 3>(3, 3) * h_trans 
				+ (Vector2d() << error, error).finished().asDiagonal()).inverse();
	cov -= kalman_gain * h_trans.transpose() * cov.block<3, 12>(3, 0);
#endif

#endif

	// Apply the Kalman gain to obtain the posterior state and error estimates.
#ifndef RANK_ONE_UPDATES
	Matrix<double, 12, 1> update = (kalman_gain * innovation);
#endif

#if DEBUG_VECTOR_OBS
	// std::cout << "projected update: " << (obs_projection * update.segment<3>(3)).transpose() << "\n";
	std::cout << "deprojected update: " << update.segment<3>(3).transpose() << "\n";
#endif
	Quaterniond posterior_update = avg_state.apply_kalman_vec_update(update);
	counter_rotate_cov(posterior_update);

	assert(is_real());
#ifdef TIME_OPS
	double time = clock.stop() * 1e6;
	std::cout << "observe_vector(): " << time << "\n";
#endif

}
