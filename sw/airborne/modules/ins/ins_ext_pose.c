/*
 * Copyright (C) 2023 MAVLab
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
 * @file modules/ins/ins_ext_pose.c
 * Integrated Navigation System interface.
 */


#include <time.h>

#include "ins_ext_pose.h"
#include "state.h"
#include "math/pprz_algebra_float.h"
#include "modules/imu/imu.h"
#include "modules/ins/ins.h"
#include "generated/flight_plan.h"

#include "modules/core/abi.h"

#if 0
#include <stdio.h>
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) {}
#endif




struct InsExtPose {
  struct LtpDef_i  ltp_def;

  /* output LTP NED */
  struct NedCoor_i ltp_pos;
  struct NedCoor_i ltp_speed;
  struct NedCoor_i ltp_accel;
};

struct InsExtPose ins_expo;


static void ins_ext_pose_init_from_flightplan(void) {

  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_expo.ltp_def, &ecef_nav0);
  ins_expo.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_expo.ltp_def);
}

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_expo.ltp_pos.x, &ins_expo.ltp_pos.y, &ins_expo.ltp_pos.z,
                    &ins_expo.ltp_speed.x, &ins_expo.ltp_speed.y, &ins_expo.ltp_speed.z,
                    &ins_expo.ltp_accel.x, &ins_expo.ltp_accel.y, &ins_expo.ltp_accel.z);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  static float fake_baro_z = 0.0;
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      (float *)&fake_baro_z, &ins_expo.ltp_pos.z,
                      &ins_expo.ltp_speed.z, &ins_expo.ltp_accel.z);
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  static float fake_qfe = 0.0;
  pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &ins_expo.ltp_def.ecef.x, &ins_expo.ltp_def.ecef.y, &ins_expo.ltp_def.ecef.z,
                          &ins_expo.ltp_def.lla.lat, &ins_expo.ltp_def.lla.lon, &ins_expo.ltp_def.lla.alt,
                          &ins_expo.ltp_def.hmsl, (float *)&fake_qfe);
}
#endif




float ekf_X[EKF_NUM_STATES];
float ekf_U[EKF_NUM_INPUTS];
float ekf_Z[EKF_NUM_OUTPUTS];
float ekf_P[EKF_NUM_STATES][EKF_NUM_STATES];
float ekf_Q[EKF_NUM_INPUTS][EKF_NUM_INPUTS];
float ekf_R[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];

float ekf_H[EKF_NUM_OUTPUTS][EKF_NUM_STATES] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0}};

float ev_pos[3] = {0.f,0.f,0.f};
float ev_att[3] = {0.f,0.f,0.f};

bool start = false;
bool measurement_update = false;

float t0;
float t1;

void ekf_set_diag(float **a, float *b, int n)
{
	int i, j;
	for (i = 0 ; i < n; i++) {
	  for (j = 0 ; j < n; j++) {
	    if (i == j) {
		a[i][j] = b[i];
	    } else {
		a[i][j] = 0.0;
	    }
	  }
	}
}

void ekf_init(void)
{
	ins_ext_pose_init_from_flightplan();

	DEBUG_PRINT("ekf init");
	float X0[EKF_NUM_STATES] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	float Pdiag[EKF_NUM_STATES] = {1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.};
	float Qdiag[EKF_NUM_INPUTS] = {0.5, 0.5, 0.5, 0.01, 0.01, 0.01};

	float Rdiag[EKF_NUM_OUTPUTS] = {0.001, 0.001, 0.001, 0.1, 0.1, 0.1};

	MAKE_MATRIX_PTR(ekf_P_, ekf_P, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_Q_, ekf_Q, EKF_NUM_INPUTS);
	MAKE_MATRIX_PTR(ekf_R_, ekf_R, EKF_NUM_OUTPUTS);

	ekf_set_diag(ekf_P_, Pdiag, EKF_NUM_STATES);
	ekf_set_diag(ekf_Q_, Qdiag, EKF_NUM_INPUTS);
	ekf_set_diag(ekf_R_, Rdiag, EKF_NUM_OUTPUTS);
	float_vect_copy(ekf_X, X0, EKF_NUM_STATES);


#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
#endif

	// Todo: get IMU through ABI
  //AbiBindMsgGPS(INS_PT_GPS_ID, &gps_ev, gps_cb);
  //AbiBindMsgIMU_ACCEL(INS_PT_IMU_ID, &accel_ev, accel_cb);

}

void ekf_f(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES])
{
	float x0=cos(X[8]);
	float x1=U[0] - X[9];
	float x2=cos(X[7]);
	float x3=x1*x2;
	float x4=U[2] - X[11];
	float x5=sin(X[6]);
	float x6=sin(X[8]);
	float x7=x5*x6;
	float x8=sin(X[7]);
	float x9=cos(X[6]);
	float x10=x0*x9;
	float x11=U[1] - X[10];
	float x12=x6*x9;
	float x13=x0*x5;
	float x14=tan(X[7]);
	float x15=U[4] - X[13];
	float x16=x15*x5;
	float x17=U[5] - X[14];
	float x18=x17*x9;
	float x19=1.0/x2;
	out[0]=X[3];
	out[1]=X[4];
	out[2]=X[5];
	out[3]=x0*x3 + x11*(-x12 + x13*x8) + x4*(x10*x8 + x7);
	out[4]=x11*(x10 + x7*x8) + x3*x6 + x4*(x12*x8 - x13);
	out[5]=-x1*x8 + x11*x2*x5 + x2*x4*x9 + 9.8100000000000005;
	out[6]=U[3] - X[12] + x14*x16 + x14*x18;
	out[7]=x15*x9 - x17*x5;
	out[8]=x16*x19 + x18*x19;
	out[9]=0;
	out[10]=0;
	out[11]=0;
	out[12]=0;
	out[13]=0;
	out[14]=0;
}

void ekf_F(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES][EKF_NUM_STATES])
{
	float x0=U[1] - X[10];
	float x1=sin(X[6]);
	float x2=sin(X[8]);
	float x3=x1*x2;
	float x4=sin(X[7]);
	float x5=cos(X[6]);
	float x6=cos(X[8]);
	float x7=x5*x6;
	float x8=x4*x7;
	float x9=x3 + x8;
	float x10=U[2] - X[11];
	float x11=x2*x5;
	float x12=x1*x6;
	float x13=x12*x4;
	float x14=x11 - x13;
	float x15=U[0] - X[9];
	float x16=x15*x4;
	float x17=cos(X[7]);
	float x18=x0*x17;
	float x19=x10*x17;
	float x20=x17*x2;
	float x21=x11*x4;
	float x22=x12 - x21;
	float x23=-x3*x4 - x7;
	float x24=x17*x6;
	float x25=x17*x5;
	float x26=x1*x17;
	float x27=x4*x5;
	float x28=U[4] - X[13];
	float x29=tan(X[7]);
	float x30=x29*x5;
	float x31=U[5] - X[14];
	float x32=x1*x29;
	float x33=pow(x29, 2) + 1;
	float x34=x1*x28;
	float x35=1.0/x17;
	float x36=x35*x5;
	float x37=x1*x35;
	float x38=pow(x17, -2);
	out[0][0]=0;
	out[0][1]=0;
	out[0][2]=0;
	out[0][3]=1;
	out[0][4]=0;
	out[0][5]=0;
	out[0][6]=0;
	out[0][7]=0;
	out[0][8]=0;
	out[0][9]=0;
	out[0][10]=0;
	out[0][11]=0;
	out[0][12]=0;
	out[0][13]=0;
	out[0][14]=0;
	out[1][0]=0;
	out[1][1]=0;
	out[1][2]=0;
	out[1][3]=0;
	out[1][4]=1;
	out[1][5]=0;
	out[1][6]=0;
	out[1][7]=0;
	out[1][8]=0;
	out[1][9]=0;
	out[1][10]=0;
	out[1][11]=0;
	out[1][12]=0;
	out[1][13]=0;
	out[1][14]=0;
	out[2][0]=0;
	out[2][1]=0;
	out[2][2]=0;
	out[2][3]=0;
	out[2][4]=0;
	out[2][5]=1;
	out[2][6]=0;
	out[2][7]=0;
	out[2][8]=0;
	out[2][9]=0;
	out[2][10]=0;
	out[2][11]=0;
	out[2][12]=0;
	out[2][13]=0;
	out[2][14]=0;
	out[3][0]=0;
	out[3][1]=0;
	out[3][2]=0;
	out[3][3]=0;
	out[3][4]=0;
	out[3][5]=0;
	out[3][6]=x0*x9 + x10*x14;
	out[3][7]=x12*x18 - x16*x6 + x19*x7;
	out[3][8]=x0*x23 + x10*x22 - x15*x20;
	out[3][9]=-x24;
	out[3][10]=x14;
	out[3][11]=-x3 - x8;
	out[3][12]=0;
	out[3][13]=0;
	out[3][14]=0;
	out[4][0]=0;
	out[4][1]=0;
	out[4][2]=0;
	out[4][3]=0;
	out[4][4]=0;
	out[4][5]=0;
	out[4][6]=x0*(-x12 + x21) + x10*x23;
	out[4][7]=x11*x19 - x16*x2 + x18*x3;
	out[4][8]=x0*(-x11 + x13) + x10*x9 + x15*x24;
	out[4][9]=-x20;
	out[4][10]=x23;
	out[4][11]=x22;
	out[4][12]=0;
	out[4][13]=0;
	out[4][14]=0;
	out[5][0]=0;
	out[5][1]=0;
	out[5][2]=0;
	out[5][3]=0;
	out[5][4]=0;
	out[5][5]=0;
	out[5][6]=x0*x25 - x10*x26;
	out[5][7]=-x0*x1*x4 - x10*x27 + x17*(-U[0] + X[9]);
	out[5][8]=0;
	out[5][9]=x4;
	out[5][10]=-x26;
	out[5][11]=-x25;
	out[5][12]=0;
	out[5][13]=0;
	out[5][14]=0;
	out[6][0]=0;
	out[6][1]=0;
	out[6][2]=0;
	out[6][3]=0;
	out[6][4]=0;
	out[6][5]=0;
	out[6][6]=x28*x30 - x31*x32;
	out[6][7]=x31*x33*x5 + x33*x34;
	out[6][8]=0;
	out[6][9]=0;
	out[6][10]=0;
	out[6][11]=0;
	out[6][12]=-1;
	out[6][13]=-x32;
	out[6][14]=-x30;
	out[7][0]=0;
	out[7][1]=0;
	out[7][2]=0;
	out[7][3]=0;
	out[7][4]=0;
	out[7][5]=0;
	out[7][6]=-x34 + x5*(-U[5] + X[14]);
	out[7][7]=0;
	out[7][8]=0;
	out[7][9]=0;
	out[7][10]=0;
	out[7][11]=0;
	out[7][12]=0;
	out[7][13]=-x5;
	out[7][14]=x1;
	out[8][0]=0;
	out[8][1]=0;
	out[8][2]=0;
	out[8][3]=0;
	out[8][4]=0;
	out[8][5]=0;
	out[8][6]=x28*x36 - x31*x37;
	out[8][7]=x27*x31*x38 + x34*x38*x4;
	out[8][8]=0;
	out[8][9]=0;
	out[8][10]=0;
	out[8][11]=0;
	out[8][12]=0;
	out[8][13]=-x37;
	out[8][14]=-x36;
	out[9][0]=0;
	out[9][1]=0;
	out[9][2]=0;
	out[9][3]=0;
	out[9][4]=0;
	out[9][5]=0;
	out[9][6]=0;
	out[9][7]=0;
	out[9][8]=0;
	out[9][9]=0;
	out[9][10]=0;
	out[9][11]=0;
	out[9][12]=0;
	out[9][13]=0;
	out[9][14]=0;
	out[10][0]=0;
	out[10][1]=0;
	out[10][2]=0;
	out[10][3]=0;
	out[10][4]=0;
	out[10][5]=0;
	out[10][6]=0;
	out[10][7]=0;
	out[10][8]=0;
	out[10][9]=0;
	out[10][10]=0;
	out[10][11]=0;
	out[10][12]=0;
	out[10][13]=0;
	out[10][14]=0;
	out[11][0]=0;
	out[11][1]=0;
	out[11][2]=0;
	out[11][3]=0;
	out[11][4]=0;
	out[11][5]=0;
	out[11][6]=0;
	out[11][7]=0;
	out[11][8]=0;
	out[11][9]=0;
	out[11][10]=0;
	out[11][11]=0;
	out[11][12]=0;
	out[11][13]=0;
	out[11][14]=0;
	out[12][0]=0;
	out[12][1]=0;
	out[12][2]=0;
	out[12][3]=0;
	out[12][4]=0;
	out[12][5]=0;
	out[12][6]=0;
	out[12][7]=0;
	out[12][8]=0;
	out[12][9]=0;
	out[12][10]=0;
	out[12][11]=0;
	out[12][12]=0;
	out[12][13]=0;
	out[12][14]=0;
	out[13][0]=0;
	out[13][1]=0;
	out[13][2]=0;
	out[13][3]=0;
	out[13][4]=0;
	out[13][5]=0;
	out[13][6]=0;
	out[13][7]=0;
	out[13][8]=0;
	out[13][9]=0;
	out[13][10]=0;
	out[13][11]=0;
	out[13][12]=0;
	out[13][13]=0;
	out[13][14]=0;
	out[14][0]=0;
	out[14][1]=0;
	out[14][2]=0;
	out[14][3]=0;
	out[14][4]=0;
	out[14][5]=0;
	out[14][6]=0;
	out[14][7]=0;
	out[14][8]=0;
	out[14][9]=0;
	out[14][10]=0;
	out[14][11]=0;
	out[14][12]=0;
	out[14][13]=0;
	out[14][14]=0;
}

void ekf_L(const float X[EKF_NUM_STATES],__attribute__((unused))  const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES][EKF_NUM_INPUTS])
{
	float x0=cos(X[7]);
	float x1=cos(X[8]);
	float x2=sin(X[8]);
	float x3=cos(X[6]);
	float x4=x2*x3;
	float x5=sin(X[7]);
	float x6=sin(X[6]);
	float x7=x1*x6;
	float x8=x2*x6;
	float x9=x1*x3;
	float x10=tan(X[7]);
	float x11=1.0/x0;
	out[0][0]=0;
	out[0][1]=0;
	out[0][2]=0;
	out[0][3]=0;
	out[0][4]=0;
	out[0][5]=0;
	out[1][0]=0;
	out[1][1]=0;
	out[1][2]=0;
	out[1][3]=0;
	out[1][4]=0;
	out[1][5]=0;
	out[2][0]=0;
	out[2][1]=0;
	out[2][2]=0;
	out[2][3]=0;
	out[2][4]=0;
	out[2][5]=0;
	out[3][0]=-x0*x1;
	out[3][1]=x4 - x5*x7;
	out[3][2]=-x5*x9 - x8;
	out[3][3]=0;
	out[3][4]=0;
	out[3][5]=0;
	out[4][0]=-x0*x2;
	out[4][1]=-x5*x8 - x9;
	out[4][2]=-x4*x5 + x7;
	out[4][3]=0;
	out[4][4]=0;
	out[4][5]=0;
	out[5][0]=x5;
	out[5][1]=-x0*x6;
	out[5][2]=-x0*x3;
	out[5][3]=0;
	out[5][4]=0;
	out[5][5]=0;
	out[6][0]=0;
	out[6][1]=0;
	out[6][2]=0;
	out[6][3]=-1;
	out[6][4]=-x10*x6;
	out[6][5]=-x10*x3;
	out[7][0]=0;
	out[7][1]=0;
	out[7][2]=0;
	out[7][3]=0;
	out[7][4]=-x3;
	out[7][5]=x6;
	out[8][0]=0;
	out[8][1]=0;
	out[8][2]=0;
	out[8][3]=0;
	out[8][4]=-x11*x6;
	out[8][5]=-x11*x3;
	out[9][0]=0;
	out[9][1]=0;
	out[9][2]=0;
	out[9][3]=0;
	out[9][4]=0;
	out[9][5]=0;
	out[10][0]=0;
	out[10][1]=0;
	out[10][2]=0;
	out[10][3]=0;
	out[10][4]=0;
	out[10][5]=0;
	out[11][0]=0;
	out[11][1]=0;
	out[11][2]=0;
	out[11][3]=0;
	out[11][4]=0;
	out[11][5]=0;
	out[12][0]=0;
	out[12][1]=0;
	out[12][2]=0;
	out[12][3]=0;
	out[12][4]=0;
	out[12][5]=0;
	out[13][0]=0;
	out[13][1]=0;
	out[13][2]=0;
	out[13][3]=0;
	out[13][4]=0;
	out[13][5]=0;
	out[14][0]=0;
	out[14][1]=0;
	out[14][2]=0;
	out[14][3]=0;
	out[14][4]=0;
	out[14][5]=0;
}



void ekf_f_rk4(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], const float dt, float out[EKF_NUM_STATES])
{
	float k1[EKF_NUM_STATES];
	float k2[EKF_NUM_STATES];
	float k3[EKF_NUM_STATES];
	float k4[EKF_NUM_STATES];

	float Xtmp[EKF_NUM_STATES];

	// k1   = f(X,U)
	ekf_f(X,U,k1);
	
	// Xtmp = X+dt*k1/2
	float_vect_smul(Xtmp, k1, dt/2, EKF_NUM_STATES);
	float_vect_add(Xtmp, X, EKF_NUM_STATES);
	
	// k2   = f(Xtmp,U)
	ekf_f(Xtmp,U,k2);
	
	// Xtmp = X+dt*k2/2
	float_vect_smul(Xtmp, k2, dt/2, EKF_NUM_STATES);
	float_vect_add(Xtmp, X, EKF_NUM_STATES);

	// k3   = f(Xtmp,U)
	ekf_f(Xtmp,U,k3);

	// Xtmp = X+dt*k3
	float_vect_smul(Xtmp, k3, dt, EKF_NUM_STATES);
	float_vect_add(Xtmp, X, EKF_NUM_STATES);

	// k4   = f(Xtmp,U)
	ekf_f(Xtmp,U,k4);	
	
	// out = k2+k3
	float_vect_sum(out, k2, k3, EKF_NUM_STATES);
	// out *= 2
	float_vect_scale(out, 2, EKF_NUM_STATES);
	// out += k1
	float_vect_add(out, k1, EKF_NUM_STATES);
	// out += k4
	float_vect_add(out, k4, EKF_NUM_STATES);
	// out *= dt/6
	float_vect_scale(out, dt/6, EKF_NUM_STATES);
	// out += X
	float_vect_add(out, X, EKF_NUM_STATES);	
}


void ekf_step(const float U[EKF_NUM_INPUTS], const float Z[EKF_NUM_OUTPUTS], const float dt)
{
	// [1] Predicted (a priori) state estimate:
	float Xkk_1[EKF_NUM_STATES];
	ekf_f_rk4(ekf_X, U, dt, Xkk_1);


	// [2] Get matrices
	float F[EKF_NUM_STATES][EKF_NUM_STATES];
	float L[EKF_NUM_STATES][EKF_NUM_INPUTS];
	ekf_F(ekf_X, U, F);
	ekf_L(ekf_X, U, L);


	// [3] Continuous to discrete
	// Fd = eye(N) + F*dt + F*F*dt**2/2 = I + [I+F*dt/2]*F*dt
	// Ld = L*dt+F*L*dt**2/2            = [I+F*dt/2]*L*dt
	float Fd[EKF_NUM_STATES][EKF_NUM_STATES];
	float Ld[EKF_NUM_STATES][EKF_NUM_INPUTS];
	float tmp[EKF_NUM_STATES][EKF_NUM_STATES];
	
	MAKE_MATRIX_PTR(F_, F, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(L_, L, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(Fd_, Fd, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(Ld_, Ld, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(tmp_, tmp, EKF_NUM_STATES);
	
	// tmp = I+F*dt/2
	float_mat_diagonal_scal(tmp_, 1, EKF_NUM_STATES);
	float_mat_sum_scaled(tmp_, F_, dt/2, EKF_NUM_STATES, EKF_NUM_STATES);

	// Ld = tmp*L*dt
	float_mat_mul(Ld_, tmp_, L_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_INPUTS);
	float_mat_scale(Ld_, dt, EKF_NUM_STATES, EKF_NUM_INPUTS);
	
	// Fd = tmp*F*dt
	float_mat_mul(Fd_, tmp_, F_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);
	float_mat_scale(Fd_, dt, EKF_NUM_STATES, EKF_NUM_STATES);

	// Fd += I
	int i;
	for (i = 0; i < EKF_NUM_STATES; i++) {
		Fd[i][i] += 1;
	}
	
	
	// [4] Predicted covariance estimate:
	// Pkk_1 = Fd*P*Fd.T + Ld*Q*Ld.T
	float Pkk_1[EKF_NUM_STATES][EKF_NUM_STATES];
	float LdT[EKF_NUM_INPUTS][EKF_NUM_STATES];
	float QLdT[EKF_NUM_INPUTS][EKF_NUM_STATES];

	MAKE_MATRIX_PTR(Pkk_1_, Pkk_1, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_P_, ekf_P, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_Q_, ekf_Q, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(LdT_, LdT, EKF_NUM_INPUTS);
	MAKE_MATRIX_PTR(QLdT_, QLdT, EKF_NUM_INPUTS);

	// Fd = Fd.T
	float_mat_transpose_square(Fd_, EKF_NUM_STATES);

	// tmp = P*Fd
	float_mat_mul(tmp_, ekf_P_, Fd_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);

	// Fd = Fd.T
	float_mat_transpose_square(Fd_, EKF_NUM_STATES);

	// Pkk_1 = Fd*tmp
	float_mat_mul(Pkk_1_, Fd_, tmp_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);

	// LdT = Ld.T
	float_mat_transpose(LdT_, Ld_, EKF_NUM_STATES, EKF_NUM_INPUTS);
	
	// QLdT = Q*LdT
	float_mat_mul(QLdT_, ekf_Q_, LdT_, EKF_NUM_INPUTS, EKF_NUM_INPUTS, EKF_NUM_STATES);

	// tmp = Ld*QLdT
	float_mat_mul(tmp_, Ld_, QLdT_, EKF_NUM_STATES, EKF_NUM_INPUTS, EKF_NUM_STATES);

	// Pkk_1 += tmp
	float_mat_sum_scaled(Pkk_1_, tmp_, 1, EKF_NUM_STATES, EKF_NUM_STATES);


	// [5] Measurement residual:
	// yk = Z - H*Xkk_1
	float yk[EKF_NUM_OUTPUTS];

	MAKE_MATRIX_PTR(ekf_H_, ekf_H, EKF_NUM_OUTPUTS);

	float_mat_vect_mul(yk, ekf_H_, Xkk_1, EKF_NUM_OUTPUTS, EKF_NUM_STATES);
	float_vect_scale(yk, -1, EKF_NUM_OUTPUTS);
	float_vect_add(yk, Z, EKF_NUM_OUTPUTS);
	

	// [6] Residual covariance:
	// Sk = H*Pkk_1*H.T + R
	float Sk[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];
	float PHT[EKF_NUM_STATES][EKF_NUM_OUTPUTS];
	
	MAKE_MATRIX_PTR(Sk_, Sk, EKF_NUM_OUTPUTS);
	MAKE_MATRIX_PTR(PHT_, PHT, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_R_, ekf_R, EKF_NUM_OUTPUTS);
	
	// PHT = Pkk_1*H.T
	float_mat_transpose(PHT_, ekf_H_, EKF_NUM_OUTPUTS, EKF_NUM_STATES);
	float_mat_mul_copy(PHT_, Pkk_1_, PHT_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_OUTPUTS);

	// Sk = H*PHT
	float_mat_mul(Sk_, ekf_H_, PHT_, EKF_NUM_OUTPUTS, EKF_NUM_STATES, EKF_NUM_OUTPUTS);
	
	// Sk += R
	float_mat_sum_scaled(Sk_, ekf_R_, 1, EKF_NUM_OUTPUTS, EKF_NUM_OUTPUTS);


	// [7] Near-optimal Kalman gain:
	// K = Pkk_1*H.T*inv(Sk)
	float Sk_inv[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];
	float K[EKF_NUM_STATES][EKF_NUM_OUTPUTS];

	MAKE_MATRIX_PTR(Sk_inv_, Sk_inv, EKF_NUM_OUTPUTS);
	MAKE_MATRIX_PTR(K_, K, EKF_NUM_STATES);
	
	// Sk_inv = inv(Sk)
	float_mat_invert(Sk_inv_, Sk_, EKF_NUM_OUTPUTS);

	// K = PHT*Sk_inv
	float_mat_mul(K_, PHT_, Sk_inv_, EKF_NUM_STATES, EKF_NUM_OUTPUTS, EKF_NUM_OUTPUTS);


	// [8] Updated state estimate
	// Xkk = Xkk_1 + K*yk
	float_mat_vect_mul(ekf_X, K_, yk, EKF_NUM_STATES, EKF_NUM_OUTPUTS);
	float_vect_add(ekf_X, Xkk_1, EKF_NUM_STATES);

	
	// [9] Updated covariance estimate:
    	// Pkk = (I - K*H)*Pkk_1

	// tmp = K*H
	float_mat_mul(tmp_, K_, ekf_H_, EKF_NUM_STATES, EKF_NUM_OUTPUTS, EKF_NUM_STATES);

	// tmp *= -1
	float_mat_scale(tmp_, -1, EKF_NUM_STATES, EKF_NUM_STATES);

	// tmp += I
	for (i = 0; i < EKF_NUM_STATES; i++) {
		tmp_[i][i] += 1;
	}
	// P = tmp*Pkk_1
	float_mat_mul(ekf_P_, tmp_, Pkk_1_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);
}

void ekf_prediction_step(const float U[EKF_NUM_INPUTS], const float dt) {
	// [1] Predicted (a priori) state estimate:
	float Xkk_1[EKF_NUM_STATES];
	// Xkk_1 = f(X,U)
	ekf_f(ekf_X, U, Xkk_1);
	// Xkk_1 *= dt
	float_vect_scale(Xkk_1, dt, EKF_NUM_STATES);
	// Xkk_1 += X
	float_vect_add(Xkk_1, ekf_X, EKF_NUM_STATES);	


	// [2] Get matrices
	float F[EKF_NUM_STATES][EKF_NUM_STATES];
	float Ld[EKF_NUM_STATES][EKF_NUM_INPUTS];
	ekf_F(ekf_X, U, F);
	ekf_L(ekf_X, U, Ld);


	// [3] Continuous to discrete
	// Fd = eye(N) + F*dt
	// Ld = L*dt
	float Fd[EKF_NUM_STATES][EKF_NUM_STATES];
	
	MAKE_MATRIX_PTR(F_, F, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(Fd_, Fd, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(Ld_, Ld, EKF_NUM_STATES);
	
	// Fd = I+F*dt/2
	float_mat_diagonal_scal(Fd_, 1, EKF_NUM_STATES);
	float_mat_sum_scaled(Fd_, F_, dt, EKF_NUM_STATES, EKF_NUM_STATES);

	// Ld = Ld*dt
	float_mat_scale(Ld_, dt, EKF_NUM_STATES, EKF_NUM_INPUTS);


	// [4] Predicted covariance estimate:
	// Pkk_1 = Fd*P*Fd.T + Ld*Q*Ld.T
	float Pkk_1[EKF_NUM_STATES][EKF_NUM_STATES];
	float LdT[EKF_NUM_INPUTS][EKF_NUM_STATES];
	float QLdT[EKF_NUM_INPUTS][EKF_NUM_STATES];
	float tmp[EKF_NUM_STATES][EKF_NUM_STATES];

	MAKE_MATRIX_PTR(Pkk_1_, Pkk_1, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_P_, ekf_P, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_Q_, ekf_Q, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(LdT_, LdT, EKF_NUM_INPUTS);
	MAKE_MATRIX_PTR(QLdT_, QLdT, EKF_NUM_INPUTS);
	MAKE_MATRIX_PTR(tmp_, tmp, EKF_NUM_STATES);

	// Fd = Fd.T
	float_mat_transpose_square(Fd_, EKF_NUM_STATES);

	// tmp = P*Fd
	float_mat_mul(tmp_, ekf_P_, Fd_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);

	// Fd = Fd.T
	float_mat_transpose_square(Fd_, EKF_NUM_STATES);

	// Pkk_1 = Fd*tmp
	float_mat_mul(Pkk_1_, Fd_, tmp_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);

	// LdT = Ld.T
	float_mat_transpose(LdT_, Ld_, EKF_NUM_STATES, EKF_NUM_INPUTS);
	
	// QLdT = Q*LdT
	float_mat_mul(QLdT_, ekf_Q_, LdT_, EKF_NUM_INPUTS, EKF_NUM_INPUTS, EKF_NUM_STATES);

	// tmp = Ld*QLdT
	float_mat_mul(tmp_, Ld_, QLdT_, EKF_NUM_STATES, EKF_NUM_INPUTS, EKF_NUM_STATES);

	// Pkk_1 += tmp
	float_mat_sum_scaled(Pkk_1_, tmp_, 1, EKF_NUM_STATES, EKF_NUM_STATES);

	// X = Xkk_1
	float_vect_copy(ekf_X, Xkk_1, EKF_NUM_STATES);

	// P = Pkk_1
	float_mat_copy(ekf_P_, Pkk_1_, EKF_NUM_STATES, EKF_NUM_STATES);	
}

void ekf_measurement_step(const float Z[EKF_NUM_OUTPUTS]) {
	// Xkk_1 = X
	float Xkk_1[EKF_NUM_STATES];
	float_vect_copy(Xkk_1, ekf_X, EKF_NUM_STATES);

	// Pkk_1 = P
	float Pkk_1[EKF_NUM_STATES][EKF_NUM_STATES];
	MAKE_MATRIX_PTR(Pkk_1_, Pkk_1, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_P_, ekf_P, EKF_NUM_STATES);
	float_mat_copy(Pkk_1_, ekf_P_, EKF_NUM_STATES, EKF_NUM_STATES);	

	// [5] Measurement residual:
	// yk = Z - H*Xkk_1
	float yk[EKF_NUM_OUTPUTS];

	MAKE_MATRIX_PTR(ekf_H_, ekf_H, EKF_NUM_OUTPUTS);

	float_mat_vect_mul(yk, ekf_H_, Xkk_1, EKF_NUM_OUTPUTS, EKF_NUM_STATES);
	float_vect_scale(yk, -1, EKF_NUM_OUTPUTS);
	float_vect_add(yk, Z, EKF_NUM_OUTPUTS);
	

	// [6] Residual covariance:
	// Sk = H*Pkk_1*H.T + R
	float Sk[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];
	float PHT[EKF_NUM_STATES][EKF_NUM_OUTPUTS];
	
	MAKE_MATRIX_PTR(Sk_, Sk, EKF_NUM_OUTPUTS);
	MAKE_MATRIX_PTR(PHT_, PHT, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_R_, ekf_R, EKF_NUM_OUTPUTS);
	
	// PHT = Pkk_1*H.T
	float_mat_transpose(PHT_, ekf_H_, EKF_NUM_OUTPUTS, EKF_NUM_STATES);
	float_mat_mul_copy(PHT_, Pkk_1_, PHT_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_OUTPUTS);

	// Sk = H*PHT
	float_mat_mul(Sk_, ekf_H_, PHT_, EKF_NUM_OUTPUTS, EKF_NUM_STATES, EKF_NUM_OUTPUTS);
	
	// Sk += R
	float_mat_sum_scaled(Sk_, ekf_R_, 1, EKF_NUM_OUTPUTS, EKF_NUM_OUTPUTS);


	// [7] Near-optimal Kalman gain:
	// K = Pkk_1*H.T*inv(Sk)
	float Sk_inv[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];
	float K[EKF_NUM_STATES][EKF_NUM_OUTPUTS];

	MAKE_MATRIX_PTR(Sk_inv_, Sk_inv, EKF_NUM_OUTPUTS);
	MAKE_MATRIX_PTR(K_, K, EKF_NUM_STATES);
	
	// Sk_inv = inv(Sk)
	float_mat_invert(Sk_inv_, Sk_, EKF_NUM_OUTPUTS);

	// K = PHT*Sk_inv
	float_mat_mul(K_, PHT_, Sk_inv_, EKF_NUM_STATES, EKF_NUM_OUTPUTS, EKF_NUM_OUTPUTS);


	// [8] Updated state estimate
	// Xkk = Xkk_1 + K*yk
	float_mat_vect_mul(ekf_X, K_, yk, EKF_NUM_STATES, EKF_NUM_OUTPUTS);
	float_vect_add(ekf_X, Xkk_1, EKF_NUM_STATES);

	
	// [9] Updated covariance estimate:
    	// Pkk = (I - K*H)*Pkk_1
	float tmp[EKF_NUM_STATES][EKF_NUM_STATES];
	MAKE_MATRIX_PTR(tmp_, tmp, EKF_NUM_STATES);

	// tmp = K*H
	float_mat_mul(tmp_, K_, ekf_H_, EKF_NUM_STATES, EKF_NUM_OUTPUTS, EKF_NUM_STATES);

	// tmp *= -1
	float_mat_scale(tmp_, -1, EKF_NUM_STATES, EKF_NUM_STATES);

	// tmp += I
	int i;
	for (i = 0; i < EKF_NUM_STATES; i++) {
		tmp_[i][i] += 1;
	}
	// P = tmp*Pkk_1
	float_mat_mul(ekf_P_, tmp_, Pkk_1_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);
}


void ekf_run(void)
{
	static bool initialized = false;


	t1 = get_sys_time_float();
	float dt = t1-t0;
	t0 = t1;

#define ROBIN_IMU	0
	
	// set input values
	ekf_U[0] = ACCEL_FLOAT_OF_BFP(imu.accels[ROBIN_IMU].scaled.x);
	ekf_U[1] = ACCEL_FLOAT_OF_BFP(imu.accels[ROBIN_IMU].scaled.y);
	ekf_U[2] = ACCEL_FLOAT_OF_BFP(imu.accels[ROBIN_IMU].scaled.z);
	ekf_U[3] = RATE_FLOAT_OF_BFP(imu.gyros[ROBIN_IMU].scaled.p);
	ekf_U[4] = RATE_FLOAT_OF_BFP(imu.gyros[ROBIN_IMU].scaled.q);
	ekf_U[5] = RATE_FLOAT_OF_BFP(imu.gyros[ROBIN_IMU].scaled.r);

	if (start) {

		if (!initialized) {
			ins_reset_local_origin();
			initialized = true;
		}

		// prediction step
		DEBUG_PRINT("ekf prediction step U = %f, %f, %f, %f, %f, %f dt = %f \n", ekf_U[0], ekf_U[1], ekf_U[2], ekf_U[3], ekf_U[4], ekf_U[5], dt);
		ekf_prediction_step(ekf_U, dt);

		// measurement step
		if (measurement_update) {
			DEBUG_PRINT("ekf measurement step Z = %f, %f, %f, %f \n", ekf_Z[0], ekf_Z[1], ekf_Z[2], ekf_Z[3]);
			ekf_measurement_step(ekf_Z);
			measurement_update = false;
		}
	}

	// initial guess
	struct NedCoor_f ned_pos;
	ned_pos.x = ekf_X[0];
	ned_pos.y = ekf_X[1];
	ned_pos.z = ekf_X[2];

	struct NedCoor_f ned_speed;
  ned_speed.x  = ekf_X[3];
  ned_speed.y = ekf_X[4];
  ned_speed.z  = ekf_X[5];

	struct FloatEulers ned_to_body_eulers;
	ned_to_body_eulers.phi = ekf_X[6];
	ned_to_body_eulers.theta = ekf_X[7];
	ned_to_body_eulers.psi = ekf_X[8];

  struct FloatRates rates = { ekf_U[3]-ekf_X[12], ekf_U[4]-ekf_X[13], ekf_U[5]-ekf_X[14] };


  // Export RAW IMU
	stateSetAccelBody_i(&imu.accels[ROBIN_IMU].scaled);


  struct FloatVect3 accel;
  struct FloatVect3 accel_ned_f;
	accel.x = ekf_U[0]-ekf_X[9];
	accel.y = ekf_U[1]-ekf_X[10];
	accel.z = ekf_U[2]-ekf_X[11];
  struct FloatRMat *ned_to_body_rmat_f = stateGetNedToBodyRMat_f();
	float_rmat_transp_vmult(&accel_ned_f, ned_to_body_rmat_f, &accel);
	accel_ned_f.z += 9.81;

	stateSetPositionNed_f(&ned_pos);
	stateSetSpeedNed_f(&ned_speed);
	stateSetNedToBodyEulers_f(&ned_to_body_eulers);
  stateSetBodyRates_f(&rates);
  stateSetAccelNed_f((struct NedCoor_f *)&accel_ned_f);

}


void external_pose_update(uint8_t *buf)
{
  if (DL_EXTERNAL_POSE_ac_id(buf) != AC_ID) { return; } // not for this aircraft
  
  float enu_x = DL_EXTERNAL_POSE_enu_x(buf);
  float enu_y = DL_EXTERNAL_POSE_enu_y(buf);
  float enu_z = DL_EXTERNAL_POSE_enu_z(buf);

  float quat_i = DL_EXTERNAL_POSE_body_qi(buf);
  float quat_x = DL_EXTERNAL_POSE_body_qx(buf);
  float quat_y = DL_EXTERNAL_POSE_body_qy(buf);
  float quat_z = DL_EXTERNAL_POSE_body_qz(buf);

	printf("EXT_UPDATE\n");

  struct FloatQuat orient;
  struct FloatEulers orient_eulers;

  orient.qi = quat_i;
  orient.qx = quat_y;   //north
  orient.qy = -quat_x;  //east
  orient.qz = -quat_z;  //down

  float_eulers_of_quat(&orient_eulers, &orient);
  //orient_eulers.psi -= 90.0/57.6;
	orient_eulers.theta = -orient_eulers.theta;

/*
  orient.qi = quat_i;
  orient.qx = quat_x;   //north
  orient.qy = quat_y;  //east
  orient.qz = quat_z;  //down

  float_eulers_of_quat(&orient_eulers, &orient);
*/

//  struct FloatEulers body_e;
//  float_eulers_of_quat(&body_e, &body_q);

  //fix psi
  float delta_psi = orient_eulers.psi - ev_att[2];
  if (delta_psi > M_PI) {
  	delta_psi -= 2*M_PI;
  } else if (delta_psi < -M_PI) {
  	delta_psi += 2*M_PI;
  }

  ev_pos[0] = enu_y;
  ev_pos[1] = enu_x;
  ev_pos[2] = -enu_z;
  ev_att[0] = orient_eulers.phi;
  ev_att[1] = orient_eulers.theta;
  ev_att[2] = orient_eulers.psi;


	DEBUG_PRINT("Att = %f %f %f \n", ev_att[0], ev_att[1], ev_att[2]);

  // ekf starts at the first ev update
  if (start == false) {
  	start = true;

	// initial guess
	ekf_X[0] = ev_pos[0];
	ekf_X[1] = ev_pos[1];
	ekf_X[2] = ev_pos[2];
	ekf_X[6] = ev_att[0];
	ekf_X[7] = ev_att[1];
	ekf_X[8] = ev_att[2];
  }
  
  measurement_update = true;
  ekf_Z[0] = ev_pos[0];
  ekf_Z[1] = ev_pos[1];
  ekf_Z[2] = ev_pos[2];
  ekf_Z[3] = ev_att[0];
  ekf_Z[4] = ev_att[1];
  ekf_Z[5] += delta_psi;  
}

void ins_reset_local_origin(void)
{
	// Ext pos does not allow geoinit: FP origin only
}

void ins_reset_altitude_ref(void)
{
	// Ext pos does not allow geoinit: FP origin only
}

// Logging
void ins_ext_pos_log_header(FILE *file) {
  fprintf(file, "ekf_X1,ekf_X2,ekf_X3,ekf_X4,ekf_X5,ekf_X6,ekf_X7,ekf_X8,ekf_X9,ekf_X10,ekf_X11,ekf_X12,ekf_X13,ekf_X14,ekf_X15,");
  fprintf(file, "ekf_U1,ekf_U2,ekf_U3,ekf_U4,ekf_U5,ekf_U6,");
  fprintf(file, "ekf_Z1,ekf_Z2,ekf_Z3,ekf_Z4,");
}

void ins_ext_pos_log_data(FILE *file) {
  fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,", ekf_X[0], ekf_X[1], ekf_X[2], ekf_X[3], ekf_X[4], ekf_X[5], ekf_X[6], ekf_X[7], ekf_X[8], ekf_X[9], ekf_X[10], ekf_X[11], ekf_X[12], ekf_X[13], ekf_X[14]);
  fprintf(file, "%f,%f,%f,%f,%f,%f,", ekf_U[0], ekf_U[1], ekf_U[2], ekf_U[3], ekf_U[4], ekf_U[5]);
  fprintf(file, "%f,%f,%f,%f,", ekf_Z[0], ekf_Z[1], ekf_Z[2], ekf_Z[3]);
}
