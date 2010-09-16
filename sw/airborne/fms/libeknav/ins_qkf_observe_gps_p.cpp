/*
 * ins_qkf_observe_gps_p.cpp
 *
 *  Created on: Jun 10, 2010
 *      Author: Jonathan Brandmeyer
 *
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
 *
 */

#include "ins_qkf.hpp"
#include "assertions.hpp"

#define RANK_ONE_UPDATES

#ifndef RANK_ONE_UPDATES
#include "timer.hpp"
#include <Eigen/LU>
#endif

using namespace Eigen;

void
basic_ins_qkf::obs_gps_p_report(const Vector3d& pos, const Vector3d& p_error)
{
	Matrix<double, 3, 1> residual = pos - avg_state.position;	
	std::cout << "diff_p(" <<residual(0) << ", " << residual(1) << ", " << residual(2) <<")\n";


#ifdef RANK_ONE_UPDATES
	Matrix<double, 12, 1> update = Matrix<double, 12, 1>::Zero();
	for (int i = 0; i < 3; ++i) {
		double innovation_cov_inv = 1.0/(cov(6+i, 6+i) + p_error[i]);
		Matrix<double, 12, 1> gain = cov.block<12, 1>(0, 6+i) * innovation_cov_inv;
		update += gain * (residual[i] - update[6+i]);
		cov -= gain * cov.block<1, 12>(6+i, 0);
	}

#else
	Matrix<double, 3, 3> innovation_cov = cov.block<3, 3>(6, 6);
	innovation_cov += p_error.asDiagonal();

	Matrix<double, 12, 3> kalman_gain = cov.block<12, 3>(0, 6)
		* innovation_cov.part<Eigen::SelfAdjoint>().inverse();
	Matrix<double, 12, 1> update = kalman_gain * residual;
	cov.part<Eigen::SelfAdjoint>() -= kalman_gain * cov.block<3, 12>(6, 0);
#endif
	Quaterniond rotor = avg_state.apply_kalman_vec_update(update);
	std::cout << "update(" << update.segment<6>(0).transpose()*180/M_PI << "\t" << update.segment<6>(6).transpose() << ")\n";
	counter_rotate_cov(rotor);
	assert(is_real());
}
