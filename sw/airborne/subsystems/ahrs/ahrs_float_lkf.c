/*
 * $Id:  $
 *
 * Copyright (C) 2009 Felix Ruess <felix.ruess@gmail.com>
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

#include "ahrs_float_lkf.h"

#include "subsystems/imu.h"
#include "subsystems/ahrs/ahrs_aligner.h"

#include "generated/airframe.h"
#include "math/pprz_algebra_float.h"

#include <stdio.h>


#define BAFL_DEBUG

static void ahrs_do_update_accel(void);
static void ahrs_do_update_mag(void);


/* our estimated attitude  (ltp <-> imu)      */
struct FloatQuat bafl_quat;
/* our estimated gyro biases                  */
struct FloatRates bafl_bias;
/* we get unbiased body rates as byproduct    */
struct FloatRates bafl_rates;
/* maintain a euler angles representation     */
struct FloatEulers bafl_eulers;
/* C n->b rotation matrix representation */
struct FloatRMat bafl_dcm;

/* estimated attitude errors from accel update   */
struct FloatQuat bafl_q_a_err;
/* estimated attitude errors from mag update   */
struct FloatQuat bafl_q_m_err;
/* our estimated gyro bias errors from accel update  */
struct FloatRates bafl_b_a_err;
/* our estimated gyro bias errors from mag update  */
struct FloatRates bafl_b_m_err;
/* temporary quaternion */
struct FloatQuat bafl_qtemp;
/* correction quaternion of strapdown computation */
struct FloatQuat bafl_qr;
/* norm of attitude quaternion */
float bafl_qnorm;

/* pseude measured angles for debug */
float bafl_phi_accel;
float bafl_theta_accel;

#ifndef BAFL_SSIZE
#define BAFL_SSIZE 6
#endif
/* error covariance matrix */
float bafl_P[BAFL_SSIZE][BAFL_SSIZE];
/* filter state */
float bafl_X[BAFL_SSIZE];

/*
 * F represents the Jacobian of the derivative of the system with respect
 * its states.  We do not allocate the bottom three rows since we know that
 * the derivatives of bias_dot are all zero.
 */
float bafl_F[3][3];
/*
 * T represents the discrete state transition matrix
 * T = e^(F * dt)
 */
float bafl_T[6][6];

/*
 * Kalman filter variables.
 */
float bafl_Pprio[BAFL_SSIZE][BAFL_SSIZE];
/* temporary variable used during state covariance update */
float bafl_tempP[BAFL_SSIZE][BAFL_SSIZE];
float bafl_K[6][3];
float bafl_tempK[6][3];
float bafl_S[3][3];
float bafl_tempS[3][6];
float bafl_invS[3][3];
struct FloatVect3 bafl_ya;
struct FloatVect3 bafl_ym;

/*
 * H represents the Jacobian of the measurements of the attitude
 * with respect to the states of the filter.  We do not allocate the bottom
 * three rows since we know that the attitude measurements have no
 * relationship to gyro bias.
 * The last three columns always stay zero.
 */
float bafl_H[3][3];

/* low pass of accel measurements */
struct FloatVect3 bafl_accel_measure;
struct FloatVect3 bafl_accel_last;

struct FloatVect3 bafl_mag;

/* temporary variables for the strapdown computation */
float bafl_qom[4][4];

#define BAFL_g 9.81

#define BAFL_hx 1.0
#define BAFL_hy 0.0
#define BAFL_hz 1.0
struct FloatVect3 bafl_h;

/*
 * Q is our estimate noise variance.  It is supposed to be an NxN
 * matrix, but with elements only on the diagonals.  Additionally,
 * since the quaternion has no expected noise (we can't directly measure
 * it), those are zero.  For the gyro, we expect around 5 deg/sec noise,
 * which is 0.08 rad/sec.  The variance is then 0.08^2 ~= 0.0075.
 */
/* I measured about 0.009 rad/s noise */
#define BAFL_Q_GYRO 1e-04
#define BAFL_Q_ATT  0
float bafl_Q_gyro;
float bafl_Q_att;

/*
 * R is our measurement noise estimate.  Like Q, it is supposed to be
 * an NxN matrix with elements on the diagonals.  However, since we can
 * not directly measure the gyro bias, we have no estimate for it.
 * We only have an expected noise in the pitch and roll accelerometers
 * and in the compass.
 */
#define BAFL_SIGMA_ACCEL 1000.0
#define BAFL_SIGMA_MAG   20.
float bafl_sigma_accel;
float bafl_sigma_mag;
float bafl_R_accel;
float bafl_R_mag;

#ifndef BAFL_DT
#define BAFL_DT (1./512.)
#endif


#define FLOAT_MAT33_INVERT(_mo, _mi) {											\
	float _det = 0.;															\
	_det =   _mi[0][0] * (_mi[2][2] * _mi[1][1] - _mi[2][1] * _mi[1][2])		\
	       - _mi[1][0] * (_mi[2][2] * _mi[0][1] - _mi[2][1] * _mi[0][2])		\
	       + _mi[2][0] * (_mi[1][2] * _mi[0][1] - _mi[1][1]	* _mi[0][2]);		\
	if (_det != 0.) { /* ? test for zero? */							\
		_mo[0][0] = (_mi[1][1] * _mi[2][2] - _mi[1][2] * _mi[2][1]) / _det;		\
		_mo[0][1] = (_mi[0][2] * _mi[2][1] - _mi[0][1] * _mi[2][2]) / _det;		\
		_mo[0][2] = (_mi[0][1] * _mi[1][2] - _mi[0][2] * _mi[1][1]) / _det;		\
																				\
		_mo[1][0] = (_mi[1][2] * _mi[2][0] - _mi[1][0] * _mi[2][2]) / _det;		\
		_mo[1][1] = (_mi[0][0] * _mi[2][2] - _mi[0][2] * _mi[2][0]) / _det;		\
		_mo[1][2] = (_mi[0][2] * _mi[1][0] - _mi[0][0] * _mi[1][2]) / _det;		\
																				\
		_mo[2][0] = (_mi[1][0] * _mi[2][1] - _mi[1][1] * _mi[2][0]) / _det;		\
		_mo[2][1] = (_mi[0][1] * _mi[2][0] - _mi[0][0] * _mi[2][1]) / _det;		\
		_mo[2][2] = (_mi[0][0] * _mi[1][1] - _mi[0][1] * _mi[1][0]) / _det;		\
	} else {																	\
		printf("ERROR! Not invertible!\n");										\
		for (int _i=0; _i<3; _i++) {											\
			for (int _j=0; _j<3; _j++) {										\
				_mo[_i][_j] = 0.0;												\
			}																	\
		}																		\
	}																			\
}

#define AHRS_TO_BFP() {												\
	/* IMU rate */													\
	RATES_BFP_OF_REAL(ahrs.imu_rate, bafl_rates);				\
	/* LTP to IMU eulers      */									\
	EULERS_BFP_OF_REAL(ahrs.ltp_to_imu_euler, bafl_eulers);	\
	/* LTP to IMU quaternion */										\
	QUAT_BFP_OF_REAL(ahrs.ltp_to_imu_quat, bafl_quat);			\
	/* LTP to IMU rotation matrix */								\
	RMAT_BFP_OF_REAL(ahrs.ltp_to_imu_rmat, bafl_dcm);			\
}

#define AHRS_LTP_TO_BODY() {																				\
	/* Compute LTP to BODY quaternion */																	\
	INT32_QUAT_COMP_INV(ahrs.ltp_to_body_quat, ahrs.ltp_to_imu_quat, imu.body_to_imu_quat);	\
	/* Compute LTP to BODY rotation matrix */																\
	INT32_RMAT_COMP_INV(ahrs.ltp_to_body_rmat, ahrs.ltp_to_imu_rmat, imu.body_to_imu_rmat);	\
	/* compute LTP to BODY eulers */																		\
	INT32_EULERS_OF_RMAT(ahrs.ltp_to_body_euler, ahrs.ltp_to_body_rmat);							\
	/* compute body rates */																				\
	INT32_RMAT_TRANSP_RATEMULT(ahrs.body_rate, imu.body_to_imu_rmat, ahrs.imu_rate);			\
}


void ahrs_init(void) {
	int i, j;

	for (i = 0; i < BAFL_SSIZE; i++) {
		for (j = 0; j < BAFL_SSIZE; j++) {
			bafl_T[i][j] = 0.;
			bafl_P[i][j] = 0.;
		}
		/* Insert the diagonal elements of the T-Matrix. These will never change. */
		bafl_T[i][i] = 1.0;
		/* initial covariance values */
		if (i<3) {
			bafl_P[i][i] = 1.0;
		} else {
			bafl_P[i][i] = 0.1;
		}
	}

	FLOAT_QUAT_ZERO(bafl_quat);

	ahrs.status = AHRS_UNINIT;
	INT_EULERS_ZERO(ahrs.ltp_to_body_euler);
	INT_EULERS_ZERO(ahrs.ltp_to_imu_euler);
	INT32_QUAT_ZERO(ahrs.ltp_to_body_quat);
	INT32_QUAT_ZERO(ahrs.ltp_to_imu_quat);
	INT_RATES_ZERO(ahrs.body_rate);
	INT_RATES_ZERO(ahrs.imu_rate);

	ahrs_float_lkf_SetRaccel(BAFL_SIGMA_ACCEL);
	ahrs_float_lkf_SetRmag(BAFL_SIGMA_MAG);

	bafl_Q_att = BAFL_Q_ATT;
	bafl_Q_gyro = BAFL_Q_GYRO;

	FLOAT_VECT3_ASSIGN(bafl_h, BAFL_hx,BAFL_hy, BAFL_hz);

}

void ahrs_align(void) {
	RATES_FLOAT_OF_BFP(bafl_bias, ahrs_aligner.lp_gyro);
	ACCELS_FLOAT_OF_BFP(bafl_accel_measure, ahrs_aligner.lp_accel);
	ahrs.status = AHRS_RUNNING;
}

static inline void ahrs_lowpass_accel(void) {
	// get latest measurement
	ACCELS_FLOAT_OF_BFP(bafl_accel_last, imu.accel);
	// low pass it
	VECT3_ADD(bafl_accel_measure, bafl_accel_last);
	VECT3_SDIV(bafl_accel_measure, bafl_accel_measure, 2);

#ifdef BAFL_DEBUG
	bafl_phi_accel = atan2f( -bafl_accel_measure.y, -bafl_accel_measure.z);
	bafl_theta_accel = atan2f(bafl_accel_measure.x, sqrtf(bafl_accel_measure.y*bafl_accel_measure.y + bafl_accel_measure.z*bafl_accel_measure.z));
#endif
}

/*
 * Propagate our dynamic system in time
 *
 * Run the strapdown computation and the predict step of the filter
 *
 *
 *      quat_dot = Wxq(pqr) * quat
 *      bias_dot = 0
 *
 * Wxq is the quaternion omega matrix:
 *
 *              [ 0, -p, -q, -r ]
 *      1/2 *   [ p,  0,  r, -q ]
 *              [ q, -r,  0,  p ]
 *              [ r,  q, -p,  0 ]
 *
 */
void ahrs_propagate(void) {
	int i, j, k;

    ahrs_lowpass_accel();

	/* compute unbiased rates */
	RATES_FLOAT_OF_BFP(bafl_rates, imu.gyro);
	RATES_SUB(bafl_rates, bafl_bias);


	/* run strapdown computation
	 *
	 */

	/* multiplicative quaternion update */
	/* compute correction quaternion */
	QUAT_ASSIGN(bafl_qr, 1., bafl_rates.p * BAFL_DT / 2, bafl_rates.q * BAFL_DT / 2, bafl_rates.r * BAFL_DT / 2);
	/* normalize it. maybe not necessary?? */
	float q_sq;
	q_sq = (bafl_qr.qx * bafl_qr.qx +bafl_qr.qy * bafl_qr.qy + bafl_qr.qz * bafl_qr.qz) / 4;
	if (q_sq > 1) { /* this should actually never happen */
		FLOAT_QUAT_SMUL(bafl_qr, bafl_qr, 1 / sqrtf(1 + q_sq));
	} else {
		bafl_qr.qi = sqrtf(1 - q_sq);
	}
	/* propagate correction quaternion */
	FLOAT_QUAT_COMP(bafl_qtemp, bafl_quat, bafl_qr);
	FLOAT_QUAT_COPY(bafl_quat, bafl_qtemp);

	bafl_qnorm = FLOAT_QUAT_NORM(bafl_quat);
	//TODO check if broot force normalization is good, use lagrange normalization?
	FLOAT_QUAT_NORMALIZE(bafl_quat);


	/*
	 *  compute all representations
	 */
	/* maintain rotation matrix representation */
	FLOAT_RMAT_OF_QUAT(bafl_dcm, bafl_quat);
	/* maintain euler representation */
	FLOAT_EULERS_OF_RMAT(bafl_eulers, bafl_dcm);
	AHRS_TO_BFP();
	AHRS_LTP_TO_BODY();


	/* error KF-Filter
	 * propagate only the error covariance matrix since error is corrected after each measurement
	 *
	 * F = [ 0  0  0          ]
	 *     [ 0  0  0   -Cbn   ]
	 *     [ 0  0  0          ]
	 *     [ 0  0  0  0  0  0 ]
	 *     [ 0  0  0  0  0  0 ]
	 *     [ 0  0  0  0  0  0 ]
	 *
	 * T = e^(dt * F)
	 *
	 * T = [ 1   0   0             ]
	 *     [ 0   1   0   -Cbn*dt   ]
	 *     [ 0   0   1             ]
	 *     [ 0   0   0   1   0   0 ]
	 *     [ 0   0   0   0   1   0 ]
	 *     [ 0   0   0   0   0   1 ]
	 *
	 * P_prio = T * P * T_T + Q
	 *
	 */

	/*
	 *  compute state transition matrix T
	 *
	 *  diagonal elements of T are always one
	 */
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			bafl_T[i][j + 3] = -RMAT_ELMT(bafl_dcm, j, i); /* inverted bafl_dcm */
		}
	}


	/*
	 * estimate the a priori error covariance matrix P_k = T * P_k-1 * T_T + Q
	 */
	/* Temporary multiplication temp(6x6) = T(6x6) * P(6x6)      */
	for (i = 0; i < BAFL_SSIZE; i++) {
		for (j = 0; j < BAFL_SSIZE; j++) {
			bafl_tempP[i][j] = 0.;
			for (k = 0; k < BAFL_SSIZE; k++) {
				bafl_tempP[i][j] += bafl_T[i][k] * bafl_P[k][j];
			}
		}
	}

	/* P_k(6x6) = temp(6x6) * T_T(6x6) + Q  */
	for (i = 0; i < BAFL_SSIZE; i++) {
		for (j = 0; j < BAFL_SSIZE; j++) {
			bafl_P[i][j] = 0.;
			for (k = 0; k < BAFL_SSIZE; k++) {
				bafl_P[i][j] += bafl_tempP[i][k] * bafl_T[j][k]; //T[j][k] = T_T[k][j]
			}
		}
		if (i<3) {
			bafl_P[i][i] += bafl_Q_att;
		} else {
			bafl_P[i][i] += bafl_Q_gyro;
		}
	}

#ifdef LKF_PRINT_P
	printf("Pp =");
	for (i = 0; i < 6; i++) {
		printf("[");
		for (j = 0; j < 6; j++) {
			printf("%f\t", bafl_P[i][j]);
		}
		printf("]\n    ");
	}
	printf("\n");
#endif
}

void ahrs_update_accel(void) {
  RunOnceEvery(50, ahrs_do_update_accel());
}

static void ahrs_do_update_accel(void) {
	int i, j, k;

#ifdef BAFL_DEBUG2
	printf("Accel update.\n");
#endif

	/* P_prio = P */
	for (i = 0; i < BAFL_SSIZE; i++) {
		for (j = 0; j < BAFL_SSIZE; j++) {
			bafl_Pprio[i][j] = bafl_P[i][j];
		}
	}

	/*
	 * set up measurement matrix
	 *
	 * g = 9.81
	 * gx = [ 0 -g  0
	 *        g  0  0
	 *        0  0  0 ]
	 * H = [          0 0 0 ]
	 *	   [ -Cnb*gx  0 0 0 ]
	 *     [          0 0 0 ]
	 *
	 * */
	bafl_H[0][0] = -RMAT_ELMT(bafl_dcm, 0, 1) * BAFL_g;
	bafl_H[0][1] =  RMAT_ELMT(bafl_dcm, 0, 0) * BAFL_g;
	bafl_H[1][0] = -RMAT_ELMT(bafl_dcm, 1, 1) * BAFL_g;
	bafl_H[1][1] =  RMAT_ELMT(bafl_dcm, 1, 0) * BAFL_g;
	bafl_H[2][0] = -RMAT_ELMT(bafl_dcm, 2, 1) * BAFL_g;
	bafl_H[2][1] =  RMAT_ELMT(bafl_dcm, 2, 0) * BAFL_g;
	/* rest is zero
	bafl_H[0][2] = 0.;
	bafl_H[1][2] = 0.;
	bafl_H[2][2] = 0.;*/

	/**********************************************
	 * compute Kalman gain K
	 *
	 * K = P_prio * H_T * inv(S)
	 * S = H * P_prio * HT + R
	 *
	 * K = P_prio * HT * inv(H * P_prio * HT + R)
	 *
	 **********************************************/

	/* covariance residual S = H * P_prio * HT + R    */

	/* temp_S(3x6) = H(3x6) * P_prio(6x6)
	 * last 4 columns of H are zero
	 */
	for (i = 0; i < 3; i++) {
		for (j = 0; j < BAFL_SSIZE; j++) {
			bafl_tempS[i][j]  = bafl_H[i][0] * bafl_Pprio[0][j];
			bafl_tempS[i][j] += bafl_H[i][1] * bafl_Pprio[1][j];
		}
	}

	/* S(3x3) = temp_S(3x6) * HT(6x3) + R(3x3)
	 *
	 * bottom 4 rows of HT are zero
	 */
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			bafl_S[i][j]  = bafl_tempS[i][0] * bafl_H[j][0]; /* H[j][0] = HT[0][j] */
			bafl_S[i][j] += bafl_tempS[i][1] * bafl_H[j][1]; /* H[j][0] = HT[0][j] */
		}
		bafl_S[i][i] += bafl_R_accel;
	}

	/* invert S
	 */
	FLOAT_MAT33_INVERT(bafl_invS, bafl_S);


	/* temp_K(6x3) = P_prio(6x6) * HT(6x3)
	 *
	 * bottom 4 rows of HT are zero
	 */
	for (i = 0; i < BAFL_SSIZE; i++) {
		for (j = 0; j < 3; j++) {
			/* not computing zero elements */
			bafl_tempK[i][j]  = bafl_Pprio[i][0] * bafl_H[j][0]; /* H[j][0] = HT[0][j] */
			bafl_tempK[i][j] += bafl_Pprio[i][1] * bafl_H[j][1]; /* H[j][1] = HT[1][j] */
		}
	}

	/* K(6x3) = temp_K(6x3) * invS(3x3)
	 */
	for (i = 0; i < BAFL_SSIZE; i++) {
		for (j = 0; j < 3; j++) {
			bafl_K[i][j]  = bafl_tempK[i][0] * bafl_invS[0][j];
			bafl_K[i][j] += bafl_tempK[i][1] * bafl_invS[1][j];
			bafl_K[i][j] += bafl_tempK[i][2] * bafl_invS[2][j];
		}
	}

	/**********************************************
	 * Update filter state.
	 *
	 *  The a priori filter state is zero since the errors are corrected after each update.
	 *
	 *  X = X_prio + K (y - H * X_prio)
	 *  X = K * y
	 **********************************************/

	/*printf("R = ");
	for (i = 0; i < 3; i++) {
		printf("[");
		for (j = 0; j < 3; j++) {
			printf("%f\t", RMAT_ELMT(bafl_dcm, i, j));
		}
		printf("]\n    ");
	}
	printf("\n");*/

	/* innovation
	 * y = Cnb * -[0; 0; g] - accel
	 */
	bafl_ya.x = -RMAT_ELMT(bafl_dcm, 0, 2) * BAFL_g - bafl_accel_measure.x;
	bafl_ya.y = -RMAT_ELMT(bafl_dcm, 1, 2) * BAFL_g - bafl_accel_measure.y;
	bafl_ya.z = -RMAT_ELMT(bafl_dcm, 2, 2) * BAFL_g - bafl_accel_measure.z;

	/* X(6) = K(6x3) * y(3)
	 */
	for (i = 0; i < BAFL_SSIZE; i++) {
		bafl_X[i]  = bafl_K[i][0] * bafl_ya.x;
		bafl_X[i] += bafl_K[i][1] * bafl_ya.y;
		bafl_X[i] += bafl_K[i][2] * bafl_ya.z;
	}

	/**********************************************
	 * Update the filter covariance.
	 *
	 *  P = P_prio - K * H * P_prio
	 *  P = ( I - K * H ) * P_prio
	 *
	 **********************************************/

	/*  temp(6x6) = I(6x6) - K(6x3)*H(3x6)
	 *
	 *  last 4 columns of H are zero
	 */
	for (i = 0; i < BAFL_SSIZE; i++) {
		for (j = 0; j < BAFL_SSIZE; j++) {
			if (i == j) {
				bafl_tempP[i][j] = 1.;
			} else {
				bafl_tempP[i][j] = 0.;
			}
			if (j < 2) { /* omit the parts where H is zero */
				for (k = 0; k < 3; k++) {
					bafl_tempP[i][j] -= bafl_K[i][k] * bafl_H[k][j];
				}
			}
		}
	}

	/*  P(6x6) = temp(6x6) * P_prio(6x6)
	 */
	for (i = 0; i < BAFL_SSIZE; i++) {
		for (j = 0; j < BAFL_SSIZE; j++) {
			bafl_P[i][j] = bafl_tempP[i][0] * bafl_Pprio[0][j];
			for (k = 1; k < BAFL_SSIZE; k++) {
				bafl_P[i][j] += bafl_tempP[i][k] * bafl_Pprio[k][j];
			}
		}
	}

#ifdef LKF_PRINT_P
	printf("Pua=");
	for (i = 0; i < 6; i++) {
		printf("[");
		for (j = 0; j < 6; j++) {
			printf("%f\t", bafl_P[i][j]);
		}
		printf("]\n    ");
	}
	printf("\n");
#endif

	/**********************************************
	 *  Correct errors.
	 *
	 *
	 **********************************************/

	/*  Error quaternion.
	 */
	QUAT_ASSIGN(bafl_q_a_err, 1.0, bafl_X[0]/2, bafl_X[1]/2, bafl_X[2]/2);
	FLOAT_QUAT_INVERT(bafl_q_a_err, bafl_q_a_err);

	/* normalize
	 * Is this needed? Or is the approximation good enough?*/
	float q_sq;
	q_sq = bafl_q_a_err.qx * bafl_q_a_err.qx + bafl_q_a_err.qy * bafl_q_a_err.qy + bafl_q_a_err.qz * bafl_q_a_err.qz;
	if (q_sq > 1) { /* this should actually never happen */
		FLOAT_QUAT_SMUL(bafl_q_a_err, bafl_q_a_err, 1 / sqrtf(1 + q_sq));
		printf("Accel error quaternion q_sq > 1!!\n");
	} else {
		bafl_q_a_err.qi = sqrtf(1 - q_sq);
	}

	/*  correct attitude
	 */
	FLOAT_QUAT_COMP(bafl_qtemp, bafl_q_a_err, bafl_quat);
	FLOAT_QUAT_COPY(bafl_quat, bafl_qtemp);


	/*  correct gyro bias
	 */
	RATES_ASSIGN(bafl_b_a_err, bafl_X[3], bafl_X[4], bafl_X[5]);
	RATES_SUB(bafl_bias, bafl_b_a_err);

	/*
	 *  compute all representations
	 */
	/* maintain rotation matrix representation */
	FLOAT_RMAT_OF_QUAT(bafl_dcm, bafl_quat);
	/* maintain euler representation */
	FLOAT_EULERS_OF_RMAT(bafl_eulers, bafl_dcm);
	AHRS_TO_BFP();
	AHRS_LTP_TO_BODY();
}


void ahrs_update_mag(void) {
  RunOnceEvery(10, ahrs_do_update_mag());
}

static void ahrs_do_update_mag(void) {
	int i, j, k;

#ifdef BAFL_DEBUG2
	printf("Mag update.\n");
#endif

	MAGS_FLOAT_OF_BFP(bafl_mag, imu.mag);

	/* P_prio = P */
	for (i = 0; i < BAFL_SSIZE; i++) {
		for (j = 0; j < BAFL_SSIZE; j++) {
			bafl_Pprio[i][j] = bafl_P[i][j];
		}
	}

	/*
	 * set up measurement matrix
	 *
	 * h = [236.; -2.; 396.];
	 * hx = [ h(2); -h(1); 0]; //only psi (phi and theta = 0)
	 * Hm = Cnb * hx;
	 * H = [ 0  0        0  0  0
	 *       0  0 Cnb*hx 0  0  0
	 *       0  0        0  0  0 ];
	 * */
	/*bafl_H[0][2] = RMAT_ELMT(bafl_dcm, 0, 0) * bafl_h.y - RMAT_ELMT(bafl_dcm, 0, 1) * bafl_h.x;
	bafl_H[1][2] = RMAT_ELMT(bafl_dcm, 1, 0) * bafl_h.y - RMAT_ELMT(bafl_dcm, 1, 1) * bafl_h.x;
	bafl_H[2][2] = RMAT_ELMT(bafl_dcm, 2, 0) * bafl_h.y - RMAT_ELMT(bafl_dcm, 2, 1) * bafl_h.x;*/
	bafl_H[0][2] = -RMAT_ELMT(bafl_dcm, 0, 1);
	bafl_H[1][2] = -RMAT_ELMT(bafl_dcm, 1, 1);
	bafl_H[2][2] = -RMAT_ELMT(bafl_dcm, 2, 1);
	//rest is zero

	/**********************************************
	 * compute Kalman gain K
	 *
	 * K = P_prio * H_T * inv(S)
	 * S = H * P_prio * HT + R
	 *
	 * K = P_prio * HT * inv(H * P_prio * HT + R)
	 *
	 **********************************************/

	/* covariance residual S = H * P_prio * HT + R    */

	/* temp_S(3x6) = H(3x6) * P_prio(6x6)
	 *
	 * only third column of H is non-zero
	 */
	for (i = 0; i < 3; i++) {
		for (j = 0; j < BAFL_SSIZE; j++) {
			bafl_tempS[i][j] = bafl_H[i][2] * bafl_Pprio[2][j];
		}
	}

	/* S(3x3) = temp_S(3x6) * HT(6x3) + R(3x3)
	 *
	 * only third row of HT is non-zero
	 */
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			bafl_S[i][j] = bafl_tempS[i][2] * bafl_H[j][2]; /* H[j][2] = HT[2][j] */
		}
		bafl_S[i][i] += bafl_R_mag;
	}

	/* invert S
	 */
	FLOAT_MAT33_INVERT(bafl_invS, bafl_S);

	/* temp_K(6x3) = P_prio(6x6) * HT(6x3)
	 *
	 * only third row of HT is non-zero
	 */
	for (i = 0; i < BAFL_SSIZE; i++) {
		for (j = 0; j < 3; j++) {
			/* not computing zero elements */
			bafl_tempK[i][j] = bafl_Pprio[i][2] * bafl_H[j][2]; /* H[j][2] = HT[2][j] */
		}
	}

	/* K(6x3) = temp_K(6x3) * invS(3x3)
	 */
	for (i = 0; i < BAFL_SSIZE; i++) {
		for (j = 0; j < 3; j++) {
			bafl_K[i][j]  = bafl_tempK[i][0] * bafl_invS[0][j];
			bafl_K[i][j] += bafl_tempK[i][1] * bafl_invS[1][j];
			bafl_K[i][j] += bafl_tempK[i][2] * bafl_invS[2][j];
		}
	}

	/**********************************************
	 * Update filter state.
	 *
	 *  The a priori filter state is zero since the errors are corrected after each update.
	 *
	 *  X = X_prio + K (y - H * X_prio)
	 *  X = K * y
	 **********************************************/

	/*  innovation
	 *  y = Cnb * [hx; hy; hz] - mag
	 */
	FLOAT_RMAT_VECT3_MUL(bafl_ym, bafl_dcm, bafl_h); //can be optimized
	FLOAT_VECT3_SUB(bafl_ym, bafl_mag);

	/* X(6) = K(6x3) * y(3)
	 */
	for (i = 0; i < BAFL_SSIZE; i++) {
		bafl_X[i]  = bafl_K[i][0] * bafl_ym.x;
		bafl_X[i] += bafl_K[i][1] * bafl_ym.y;
		bafl_X[i] += bafl_K[i][2] * bafl_ym.z;
	}

	/**********************************************
	 * Update the filter covariance.
	 *
	 *  P = P_prio - K * H * P_prio
	 *  P = ( I - K * H ) * P_prio
	 *
	 **********************************************/

	/*  temp(6x6) = I(6x6) - K(6x3)*H(3x6)
	 *
	 *  last 3 columns of H are zero
	 */
	for (i = 0; i < 6; i++) {
		for (j = 0; j < 6; j++) {
			if (i == j) {
				bafl_tempP[i][j] = 1.;
			} else {
				bafl_tempP[i][j] = 0.;
			}
			if (j == 2) { /* omit the parts where H is zero */
				for (k = 0; k < 3; k++) {
					bafl_tempP[i][j] -= bafl_K[i][k] * bafl_H[k][j];
				}
			}
		}
	}

	/*  P(6x6) = temp(6x6) * P_prio(6x6)
	 */
	for (i = 0; i < BAFL_SSIZE; i++) {
		for (j = 0; j < BAFL_SSIZE; j++) {
			bafl_P[i][j] = bafl_tempP[i][0] * bafl_Pprio[0][j];
			for (k = 1; k < BAFL_SSIZE; k++) {
				bafl_P[i][j] += bafl_tempP[i][k] * bafl_Pprio[k][j];
			}
		}
	}

#ifdef LKF_PRINT_P
	printf("Pum=");
	for (i = 0; i < 6; i++) {
		printf("[");
		for (j = 0; j < 6; j++) {
			printf("%f\t", bafl_P[i][j]);
		}
		printf("]\n    ");
	}
	printf("\n");
#endif

	/**********************************************
	 *  Correct errors.
	 *
	 *
	 **********************************************/

	/*  Error quaternion.
	 */
	QUAT_ASSIGN(bafl_q_m_err, 1.0, bafl_X[0]/2, bafl_X[1]/2, bafl_X[2]/2);
	FLOAT_QUAT_INVERT(bafl_q_m_err, bafl_q_m_err);
	/* normalize */
	float q_sq;
	q_sq = bafl_q_m_err.qx * bafl_q_m_err.qx + bafl_q_m_err.qy * bafl_q_m_err.qy + bafl_q_m_err.qz * bafl_q_m_err.qz;
	if (q_sq > 1) { /* this should actually never happen */
		FLOAT_QUAT_SMUL(bafl_q_m_err, bafl_q_m_err, 1 / sqrtf(1 + q_sq));
		printf("mag error quaternion q_sq > 1!!\n");
	} else {
		bafl_q_m_err.qi = sqrtf(1 - q_sq);
	}

	/*  correct attitude
	 */
	FLOAT_QUAT_COMP(bafl_qtemp, bafl_q_m_err, bafl_quat);
	FLOAT_QUAT_COPY(bafl_quat, bafl_qtemp);

	/*  correct gyro bias
	 */
	RATES_ASSIGN(bafl_b_m_err, bafl_X[3], bafl_X[4], bafl_X[5]);
	RATES_SUB(bafl_bias, bafl_b_m_err);

	/*
	 *  compute all representations
	 */
	/* maintain rotation matrix representation */
	FLOAT_RMAT_OF_QUAT(bafl_dcm, bafl_quat);
	/* maintain euler representation */
	FLOAT_EULERS_OF_RMAT(bafl_eulers, bafl_dcm);
	AHRS_TO_BFP();
	AHRS_LTP_TO_BODY();
}

void ahrs_update(void) {
	ahrs_update_accel();
	ahrs_update_mag();
}
