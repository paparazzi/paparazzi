/*
 * assertions.hpp
 *
 *  Created on: Aug 6, 2009
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

#ifndef AHRS_ASSERTIONS_HPP
#define AHRS_ASSERTIONS_HPP

#include <Eigen/Core>
#include <cmath>

template<typename MatrixBase>
bool hasNaN(const MatrixBase& expr);

template<typename MatrixBase>
bool hasInf(const MatrixBase& expr);

template<typename MatrixBase>
bool hasNaN(const MatrixBase& expr)
{
	for (int j = 0; j != expr.cols(); ++j) {
		for (int i = 0; i != expr.rows(); ++i) {
			if (std::isnan(expr.coeff(i, j)))
				return true;
		}
	}
	return false;
}

template<typename MatrixBase>
bool hasInf(const MatrixBase& expr)
{
	for (int i = 0; i != expr.cols(); ++i) {
		for (int j = 0; j != expr.rows(); ++j) {
			if (std::isinf(expr.coeff(j, i)))
				return true;
		}
	}
	return false;
}

template<typename MatrixBase>
bool isReal(const MatrixBase& exp)
{
	return !hasNaN(exp) && ! hasInf(exp);
}

#endif /* ASSERTIONS_HPP_ */
