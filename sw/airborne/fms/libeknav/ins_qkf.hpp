#ifndef AHRS_INS_QKF_HPP
#define AHRS_INS_QKF_HPP
/*
 * ins_qkf.cpp
 *
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
 */

//#include "sigma_points.hpp"
#include "quaternions.hpp"
#include <Eigen/StdVector>

#define BARO_CENTER_OF_MASS 1

using Eigen::Vector3f;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::Quaterniond;
using Eigen::Matrix;
using Eigen::aligned_allocator;


struct basic_ins_qkf
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/// The maximum number of satellites that may be tracked by the filter
	static const size_t max_sv = 12;
	/**
	 * The covariance of the zero-mean gaussian white noise that is added to
	 * the gyro bias at each time step.  This value is treated as a diagonal
	 *  matrix, in units of radians^2/second^2.
	 */
	const Vector3d gyro_stability_noise;

	/**
	 * The covariance of the zero-mean gaussian white noise that is added to
	 * the gyro measurement at each time step, in rad^2/second^2
	 */
	const Vector3d gyro_white_noise;

	/**
	 * The covariance of the zero-mean gaussian white noise that is added to
	 * the accelerometer measurement at each time step, in (m/s/s)^2
	 */
	const Vector3d accel_white_noise;

	/**
	 * A term for the basic state of the system
	 */
	struct state
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		typedef std::vector<state, aligned_allocator<state> > container_t;

		/**
		 * Construct a state variable from mean and error terms
		 * @param mean The average state
		 * @param error An error vector of length 12 or greater
		 */
		template <typename Vector_T>
		state(const state& mean, const Vector_T& error)
			: gyro_bias(mean.gyro_bias + error.template segment<3>(0))
			, orientation(mean.orientation * exp<double>(error.template segment<3>(3)))
			, position(mean.position + error.template segment<3>(6).template cast<double>())
			, velocity(mean.velocity + error.template segment<3>(9))
		{
			assert(!has_nan());
		}

#if 0
		template <typename Vector_T>
		state(const state& mean, const Vector_T& error, bool)
			: gyro_bias(mean.gyro_bias + error.segment(0, 3))
			, orientation(mean.orientation * exp<double>(error.segment(3, 3)))
			, position(mean.position + error.segment(6, 3).template cast<double>())
			, velocity(mean.velocity + error.segment(9, 3))
		{
			assert(!has_nan());
		}
#endif
		/**
		 * Default-construct an undefined state object.
		 * @return
		 */
		state(){}

		/**
		 * Provided a kalman update vector, apply the vector as an offset to this
		 * state.
		 * @param update A 12-vector to be applied
		 * @return The rotation applied to the mean orientation
		 */
		Quaterniond apply_kalman_vec_update(const Matrix<double, 12, 1> update);
		Quaterniond apply_left_kalman_vec_update(const Matrix<double, 12, 1> update);

		/**
		 * An estimate of the bias error in the rate gyros, in radians/second
		 */
		Vector3d gyro_bias;

		/**
		 * An estimate of the orientation of the vehicle.  This quaternion represents
		 * a transformation from ECEF coordinates to the vehicle body frame.
		 */
		Quaterniond orientation;

		/// Position in Earth-centered Earth-fixed reference frame, in meters
		Vector3d position;

		/// Velocity in Earth-centered Earth-fixed reference frame, in m/s
		Vector3d velocity;

		/**
		 * @return True if the state vector contains any NaNs
		 */
		bool has_nan(void) const;

		/**
		 * @return True if the state vector does not contain any NaNs or Infs
		 */
		bool is_real(void) const;

		/**
		 * Print a representation of this object to the stream str.
		 * @param str An output stream.
		 */
		void print(std::ostream& str);
	};

	/// The average state of the filter at any time t.
	state avg_state;

	/// Covariance term.  Elements are ordered exactly as in struct state
	Matrix<double, 12, 12> cov;

	/**
	 * Initialize a new basic INS QKF
	 * @param pos_estimate Initial estimate of the position
	 * @param pos_error one-sigma initial bounds for position error
	 * @param bias_error one-sigma initial bounds for bias error in the gyros
	 * @param v_error one-sigma bounds for velocity error (initial v == 0)
	 * @param gyro_white_noise The diagonal matrix of gyro white noise
	 * @param gyro_stability_noise The diagonal matrix of gyro instability noise
	 * @param accel_white_noise The diagonal matrix of accelerometer white noise
	 */
			//Old one without orientation_init()
	basic_ins_qkf(const Vector3d& pos_estimate,
			double pos_error, double bias_error, double v_error,
			const Vector3d& gyro_white_noise,
			const Vector3d& gyro_stability_noise,
			const Vector3d& accel_white_noise,
			Quaterniond initial_orientation = Quaterniond::Identity(),
			const Vector3d& vel_estimate = Vector3d::Zero());
	

	/*		//Old one without orientation_init()
	basic_ins_qkf(const Vector3d& pos_estimate,
			double pos_error, double bias_error, double v_error,
			const Vector3d& gyro_white_noise,
			const Vector3d& gyro_stability_noise,
			const Vector3d& accel_white_noise,
			const Vector3d& vel_estimate = Vector3d::Zero());
	*/
	/**
	 * Report an INS observation, to propagate the filter forward by one time
	 * step. The coordinate system is maintained in ECEF coordinates.
	 * TODO: Provide a workspace parameter for storage that should be
	 * carried forward to an observation function carried out in this
	 * time step.
	 * @param gyro_meas The measured angular velocity, in rad/sec
	 * @param accel_meas The measured inertial reference frame acceleration, in m/s
	 * @param dt The elapsed time since the last measurement, in seconds.
	 */
	void predict(const Vector3d& gyro_meas, const Vector3d& accel_meas, double dt);

	/**
	 * Report an INS observation, to propagate the filter forward by one time
	 * step. This function differs from predict() in that it uses the NED frame
	 * instead of the ECEF frame.
	 */
	void predict_ned(const Vector3d& gyro_meas, const Vector3d& accel_meas, double dt);

	/**
	 * Make a single vector observation, with some angular uncertainty.
	 * Warning: only one of obs_vector or obs_gps_pv_report can be called
	 * after a single call to predict().
	 *	@param ref Reference vector, when the orientation is the identity.
	 *		Must be a unit vector.
	 *	@param obs Vector observation, should not be a unit vector
	 *	@param error one-sigma squared magnitude error in the observation
	 */
	void obs_vector(const Vector3d& ref, const Vector3d& obs, double error);

	/**
	 * Incorporate a GPS PVT report.
	 * @param pos The sensor position, in earth-centered, earth-fixed coords, meters
	 * @param vel The sensor velocity, in meters/second
	 * @param p_error The RMS position error, (m)^2
	 * @param v_error The RMS velocity error, (m/s)^2
	 */
	void obs_gps_pv_report(const Vector3d& pos, const Vector3d& vel, const Vector3d& p_error, const Vector3d v_error);

	/**
	 * Incorporate a GPS position report, in either ECEF or NED coordinates.
	 * @param pos The position, in meters
	 * @param p_error The position error, in meters.
	 */
	void obs_gps_p_report(const Vector3d& pos, const Vector3d& p_error);
  
  /**
   * Incoporate a barometer report, the altitude should be ein ENU coordinates (Up=Positive)
   * It's also necessary to provide the ECEF2ENU matrix.
   * @param altitude The altitude, in meters
   * @param baro_error, The altitude error, in meters
   * @param ecef2enu, The rotational martrix form ECEF to ENU
   * @param pos_0, the position where ENU would be (0, 0, 0).
   */
  #if BARO_CENTER_OF_MASS
  void obs_baro_report(double altitude, double baro_error);
  #else
  void obs_baro_report(double altitude, double baro_error, Matrix<double, 3, 3> ecef2enu, const Vector3d& pos_0);
  #endif

	/**
	 * Incorporate a GPS velocity report, in ECEF 3d coordinates.
	 * @param vel The 3d velocity, relative to the fixed earth frame, in (m/s).
	 * @param v_error The one-sigma RMS velocity error (m/s)^2
	 */
	void obs_gps_v_report(const Vector3d& vel, const Vector3d& v_error);

	/**
	 * Observe a GPS vector track over ground report, in north-east-down coordinates
	 * @param vel The 2d velocity value, parallel to the ground, in m/s
	 * @param v_error The one-sigma RMS velocity error (m/s)^2
	 */
	void obs_gps_vtg_report(const Vector2d vel, const double v_error);

	/**
	 * Directly observe the gyro sensor bias. In practice, we cannot do this. However,
	 * the true bias is not a random walk. It tends to return towards zero when it is
	 * farther away from zero (not temperature dependent). Therefore, we can
	 * incorporate this extra knowledge through a periodic "observation" of zero 
	 * bias with a large error.
	 *
	 * Use with extreme caution. It is almost certainly better to just clamp the
	 * bias term after making an observation.
	 *
	 * @param bias The observed bias, in radians/sec
	 * @param bias_error The one-sigma estimate of the gyro bias error, in radians/sec
	 */
	void obs_gyro_bias(const Vector3d& bias, const Vector3d& bias_error);

	/**
	 * Measure the total angular error between the filter's attitude estimate
	 * and some other orientation.
	 * @param orientation The attitude to compare against
	 * @return The angular difference between them, in radians
	 */
	double angular_error(const Quaterniond& orientation) const;

	/**
	 * Measure the total gyro bias error between the filter's estimate and
	 * some other bias
	 * @param gyro_bias The gyro bias vector to compare against
	 * @return The vector difference between them, in radians/second
	 */
	double gyro_bias_error(const Vector3d& gyro_bias) const;

	/**
	 * Determine the statistical distance between an example sample point
	 * and the distribution computed by the estimator.
	 */
	double mahalanobis_distance(const state& sample) const;

private:

	/**
	 * The type of an error term between two state vectors.
	 */
	typedef Eigen::Matrix<double, 12, 1> state_error_t;
	/// Compute the error difference between a sigma point and the mean as: point - mean
	state_error_t sigma_point_difference(const state& mean, const state& point) const;


public:
	/** Perform the posterior counter-rotation of the covariance matrix by
	  the update that gets applied to the estimated state.
	  */
	void counter_rotate_cov(const Quaterniond& update);

	/**
	 * Verify that the covariance and average state are niether NaN nor Inf
	 * @return True, iff no element in the covariance or mean are NaN or Inf
	 */
	bool is_real(void) const;
};

#endif
