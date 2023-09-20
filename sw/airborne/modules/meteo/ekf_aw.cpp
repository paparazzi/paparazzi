#include "modules/meteo/ekf_aw.h"
#include <iostream>
#include <stdio.h>
#include "std.h"
#include <math.h>

#include "mcu_periph/sys_time.h" // FOR DEBUG

#include <matrix/math.hpp>

// Covariance matrix elements and size
enum ekfAwCovVar {
  EKF_AW_u_index, EKF_AW_v_index, EKF_AW_w_index,
   EKF_AW_mu_x_index, EKF_AW_mu_y_index, EKF_AW_mu_z_index,
   EKF_AW_k_x_index, EKF_AW_k_y_index, EKF_AW_k_z_index,
   EKF_AW_COV_SIZE
};

typedef matrix::SquareMatrix<float, EKF_AW_COV_SIZE> EKF_Aw_Cov;

// Process noise elements and size
enum ekfAwQVar {
  EKF_AW_Q_accel_x_index, EKF_AW_Q_accel_y_index, EKF_AW_Q_accel_z_index,
  EKF_AW_Q_gyro_x_index,  EKF_AW_Q_gyro_y_index,  EKF_AW_Q_gyro_z_index,
  EKF_AW_Q_mu_x_index,    EKF_AW_Q_mu_y_index,    EKF_AW_Q_mu_z_index,
  EKF_AW_Q_k_x_index,     EKF_AW_Q_k_y_index,     EKF_AW_Q_k_z_index,
  EKF_AW_Q_SIZE
};

typedef matrix::SquareMatrix<float, EKF_AW_Q_SIZE> EKF_Aw_Q;

// Measurement noise elements and size
enum ekfAwRVar {
  EKF_AW_R_V_gnd_x_index,  EKF_AW_R_V_gnd_y_index, EKF_AW_R_V_gnd_z_index,
  EKF_AW_R_a_x_filt_index, EKF_AW_R_a_y_filt_index, EKF_AW_R_a_z_filt_index,
  EKF_AW_R_V_pitot_index, 
  EKF_AW_R_SIZE
};

typedef matrix::SquareMatrix<float, EKF_AW_R_SIZE> EKF_Aw_R;

// filter state vector
struct ekfAwState {
	matrix::Vector3f V_body;
	matrix::Vector3f wind;
  matrix::Vector3f offset;
};

// filter command vector
struct ekfAwInputs {
	matrix::Vector3f accel;
  matrix::Vector3f rates;
  matrix::Vector3f euler;
  float RPM_pusher;
  matrix::Vector<float,4> RPM_hover;
  float skew;
  float elevator_angle;	
};

// filter measurement vector
struct ekfAwMeasurements {
	matrix::Vector3f V_gnd;
	matrix::Vector3f accel_filt;
	float V_pitot;
};

// forces vector
struct ekfAwForces {
  matrix::Vector3f fuselage;
  matrix::Vector3f wing;
  matrix::Vector3f elevator;
  matrix::Vector3f hover;
  matrix::Vector3f pusher;
};

// private filter structure
struct ekfAwPrivate {
  struct ekfAwState state;
  struct ekfAwInputs inputs;
  struct ekfAwMeasurements measurements;
  struct ekfAwMeasurements innovations;
  struct ekfAwForces forces;

  EKF_Aw_Cov P;
  EKF_Aw_Q Q;
  EKF_Aw_R R;

  struct ekfHealth health;
};

// Parameters Process Noise
#ifndef EKF_AW_Q_ACCEL
#define EKF_AW_Q_ACCEL   1.0E-4f
#endif
#ifndef EKF_AW_Q_GYRO
#define EKF_AW_Q_GYRO    1.0E-09f
#endif
#ifndef EKF_AW_Q_MU
#define EKF_AW_Q_MU      1.0E-6f
#endif
#ifndef EKF_AW_Q_OFFSET
#define EKF_AW_Q_OFFSET  1.E-8f
#endif

// Parameters Initial Covariance Matrix
#ifndef EKF_AW_P0_V_BODY
#define EKF_AW_P0_V_BODY   1.E-2f
#endif
#ifndef EKF_AW_P0_MU
#define EKF_AW_P0_MU       EKF_AW_Q_MU*1.E1f
#endif
#ifndef EKF_AW_P0_OFFSET
#define EKF_AW_P0_OFFSET   EKF_AW_Q_OFFSET
#endif

// Parameters Measurement Noise
#ifndef EKF_AW_R_V_GND
#define EKF_AW_R_V_GND        1.E-5f
#endif
#ifndef EKF_AW_R_ACCEL_FILT_X
#define EKF_AW_R_ACCEL_FILT_X   1.E-5f
#endif
#ifndef EKF_AW_R_ACCEL_FILT_Y
#define EKF_AW_R_ACCEL_FILT_Y   1.E-5f
#endif
#ifndef EKF_AW_R_ACCEL_FILT_Z
#define EKF_AW_R_ACCEL_FILT_Z   1.E-5f
#endif
#ifndef EKF_AW_R_V_PITOT
#define EKF_AW_R_V_PITOT      1.E-7f
#endif

// Other options
#ifndef EKF_AW_WING_INSTALLED
#define EKF_AW_WING_INSTALLED false
#endif
#ifndef EKF_AW_USE_MODEL_BASED_X
#define EKF_AW_USE_MODEL_BASED_X false
#endif
#ifndef EKF_AW_USE_MODEL_BASED_Y
#define EKF_AW_USE_MODEL_BASED_Y false
#endif
#ifndef EKF_AW_USE_MODEL_BASED_Z
#define EKF_AW_USE_MODEL_BASED_Z false
#endif
#ifndef EKF_AW_USE_BETA
#define EKF_AW_USE_BETA false
#endif
#ifndef EKF_AW_PROPAGATE_OFFSET
#define EKF_AW_PROPAGATE_OFFSET false
#endif
#ifndef EKF_AW_USE_PITOT
#define EKF_AW_USE_PITOT false
#endif

// Model Based Parameters
#ifndef EKF_AW_VEHICLE_MASS
#define EKF_AW_VEHICLE_MASS 6.5f
#endif

// Fx
#ifndef EKF_AW_K1_FX_DRAG
#define EKF_AW_K1_FX_DRAG -3.0E-1f
#endif
#ifndef EKF_AW_K2_FX_DRAG
#define EKF_AW_K2_FX_DRAG -4.0E-2f
#endif

#ifndef EKF_AW_K1_FX_FUSELAGE
#define EKF_AW_K1_FX_FUSELAGE 0.0f
#endif
#ifndef EKF_AW_K2_FX_FUSELAGE
#define EKF_AW_K2_FX_FUSELAGE -0.04f
#endif
#ifndef EKF_AW_K3_FX_FUSELAGE
#define EKF_AW_K3_FX_FUSELAGE 0.0f
#endif
#ifndef EKF_AW_K4_FX_FUSELAGE
#define EKF_AW_K4_FX_FUSELAGE 0.0f
#endif

#ifndef EKF_AW_K1_FX_HOVER
#define EKF_AW_K1_FX_HOVER 0.0f
#endif
#ifndef EKF_AW_K2_FX_HOVER
#define EKF_AW_K2_FX_HOVER 0.0f
#endif
#ifndef EKF_AW_K3_FX_HOVER
#define EKF_AW_K3_FX_HOVER -0.3f
#endif

#ifndef EKF_AW_K1_FX_WING
#define EKF_AW_K1_FX_WING -6.428638953316000e-03f
#endif
#ifndef EKF_AW_K2_FX_WING
#define EKF_AW_K2_FX_WING 1.671952644901720e-01f
#endif
#ifndef EKF_AW_K3_FX_WING
#define EKF_AW_K3_FX_WING 5.944103706458780e-01f
#endif
#ifndef EKF_AW_K4_FX_WING
#define EKF_AW_K4_FX_WING 3.983889380919000e-03f
#endif
#ifndef EKF_AW_K5_FX_WING
#define EKF_AW_K5_FX_WING 3.532085496834000e-03f
#endif

#ifndef EKF_AW_K1_FX_PUSH
#define EKF_AW_K1_FX_PUSH 3.96222948E-07f
#endif
#ifndef EKF_AW_K2_FX_PUSH
#define EKF_AW_K2_FX_PUSH -5.2930351318E-05f
#endif
#ifndef EKF_AW_K3_FX_PUSH
#define EKF_AW_K3_FX_PUSH -2.68843366027904E-01f
#endif

#ifndef EKF_AW_K1_FX_ELEV
#define EKF_AW_K1_FX_ELEV 0.0f
#endif
#ifndef EKF_AW_K2_FX_ELEV
#define EKF_AW_K2_FX_ELEV 0.0f
#endif
#ifndef EKF_AW_K3_FX_ELEV
#define EKF_AW_K3_FX_ELEV 0.0f
#endif

// Fy
#ifndef EKF_AW_K_FY_BETA
#define EKF_AW_K_FY_BETA -2.19E-1f
#endif
#ifndef EKF_AW_K_FY_V
#define EKF_AW_K_FY_V -3.2E-1f
#endif
#ifndef EKF_AW_K1_FY_WING
#define EKF_AW_K1_FY_WING 0.0f
#endif
#ifndef EKF_AW_K2_FY_WING
#define EKF_AW_K2_FY_WING 0.0f
#endif
#ifndef EKF_AW_K3_FY_WING
#define EKF_AW_K3_FY_WING 0.0f
#endif
#ifndef EKF_AW_K4_FY_WING
#define EKF_AW_K4_FY_WING 0.0f
#endif
#ifndef EKF_AW_K5_FY_WING
#define EKF_AW_K5_FY_WING 0.0f
#endif

// Fz
#ifndef EKF_AW_K1_FZ_FUSELAGE
#define EKF_AW_K1_FZ_FUSELAGE 0.0f
#endif
#ifndef EKF_AW_K2_FZ_FUSELAGE
#define EKF_AW_K2_FZ_FUSELAGE 0.0f
#endif
#ifndef EKF_AW_K3_FZ_FUSELAGE
#define EKF_AW_K3_FZ_FUSELAGE 0.0f
#endif
#ifndef EKF_AW_K4_FZ_FUSELAGE
#define EKF_AW_K4_FZ_FUSELAGE 0.0f
#endif

#ifndef EKF_AW_K1_FZ_WING
#define EKF_AW_K1_FZ_WING -1.000778727574050e-01f
#endif
#ifndef EKF_AW_K2_FZ_WING
#define EKF_AW_K2_FZ_WING -8.696479964371250e-01f
#endif
#ifndef EKF_AW_K3_FZ_WING
#define EKF_AW_K3_FZ_WING 1.457831456377660e-01f
#endif
#ifndef EKF_AW_K4_FZ_WING
#define EKF_AW_K4_FZ_WING 2.185394878246410e-01f
#endif

#ifndef EKF_AW_K1_FZ_HOVER
#define EKF_AW_K1_FZ_HOVER -8.738705811080210e-07f
#endif
#ifndef EKF_AW_K2_FZ_HOVER
#define EKF_AW_K2_FZ_HOVER -9.517409386179890e-07f
#endif
#ifndef EKF_AW_K3_FZ_HOVER
#define EKF_AW_K3_FZ_HOVER -8.946217883362630e-07f
#endif
#ifndef EKF_AW_K4_FZ_HOVER
#define EKF_AW_K4_FZ_HOVER -8.520556416144729e-07f
#endif
#ifndef EKF_AW_K5_FZ_HOVER
#define EKF_AW_K5_FZ_HOVER 0.0f
#endif

#ifndef EKF_AW_K1_FZ_ELEV
#define EKF_AW_K1_FZ_ELEV 0.0f
#endif
#ifndef EKF_AW_K2_FZ_ELEV
#define EKF_AW_K2_FZ_ELEV 0.0f
#endif

// Covariance Schedule
#ifndef EKF_AW_AZ_SCHED_GAIN
#define EKF_AW_AZ_SCHED_GAIN 0
#endif
#ifndef EKF_AW_AZ_SCHED_START_DEG
#define EKF_AW_AZ_SCHED_START_DEG 60
#endif
#ifndef EKF_AW_AZ_SCHED_END_DEG
#define EKF_AW_AZ_SCHED_END_DEG 70
#endif
#ifndef EKF_AW_AX_SCHED_GAIN
#define EKF_AW_AX_SCHED_GAIN 0
#endif
#ifndef EKF_AW_AX_SCHED_START_DEG
#define EKF_AW_AX_SCHED_START_DEG 40
#endif
#ifndef EKF_AW_AX_SCHED_END_DEG
#define EKF_AW_AX_SCHED_END_DEG 60
#endif

// Quick Convergence
#ifndef EKF_AW_AZ_QUICK_CONV_MU_GAIN
#define EKF_AW_AZ_QUICK_CONV_MU_GAIN -2
#endif
#ifndef EKF_AW_AZ_QUICK_CONV_ACCEL_GAIN
#define EKF_AW_AZ_QUICK_CONV_ACCEL_GAIN 0
#endif

// Other
#ifndef EKF_AW_ELEV_MAX_ANGLE
#define EKF_AW_ELEV_MAX_ANGLE 37.0f
#endif
#ifndef EKF_AW_ELEV_MIN_ANGLE
#define EKF_AW_ELEV_MIN_ANGLE -10.0f
#endif
#ifndef EKF_AW_AOA_MAX_ANGLE
#define EKF_AW_AOA_MAX_ANGLE 15.0f
#endif
#ifndef EKF_AW_AOA_MIN_ANGLE
#define EKF_AW_AOA_MIN_ANGLE -15.0f
#endif

// Measured skew value to real skew 
#ifndef EKF_AW_SKEW_POLY_0
#define EKF_AW_SKEW_POLY_0 0.0f
#endif
#ifndef EKF_AW_SKEW_POLY_1
#define EKF_AW_SKEW_POLY_1 1.0f
#endif
#ifndef EKF_AW_SKEW_POLY_2
#define EKF_AW_SKEW_POLY_2 0.0f
#endif

// FOR DEBUG
#ifndef EKF_AW_DEBUG
#define EKF_AW_DEBUG false
#endif

/*
// TIMING INFO (for future optimization)
Part of code                | Time to run (micro sec)
------------------------------------------------------
Var declaration             | 2
Trig fn                     | 3
Aoa, Beta                   | 5
F+L declaration             | 2
P = F*P*F^T+L*Q*L^T         | 3
Accel calc                  | 1
G                           | 2
S calculation G*P*G^T+R     | 28
Kalman gain calc P*G^T*S^-1 | 48
State update                | 7
Cov Update P=(I-K*G)*P      | 36
*/

// Parameters
struct ekfAwParameters ekf_aw_params;

// Internal structure
static struct ekfAwPrivate ekf_aw_private;
// Short name
#define eawp ekf_aw_private

// Earth Gravity
static const matrix::Vector3f gravity( 0.f, 0.f, 9.81f );

// Constants
float deg2rad = M_PI / 180.0;
float rad2deg = 180.0 / M_PI;

// Forces functions
float fx_fuselage(float *skew,float *aoa,float *u);
float fx_elevator(float *elevator_angle, float *V_a);
float fx_wing(float *skew,float *aoa,float *u);
float fy_wing(float *skew,float *aoa,float *u);
float fx_fy_hover(float *RPM_hover_mean, float *V);
float fx_pusher(float *RPM_pusher, float *u);
float fz_fuselage(float *skew,float *aoa,float *V_a);
float fz_elevator(float *elevator_angle, float *V_a);
float fz_wing(float *skew,float *aoa,float *V_a);
float fz_hover(matrix::Vector<float,4> RPM_hover,float *V_a);

/* init state and measurements */
static void init_ekf_aw_state(void)
{
  // Init State
  eawp.state.V_body.setZero();
	eawp.state.wind.setZero();
  eawp.state.offset.setZero();

  // Init Measures
  eawp.measurements.V_gnd.setZero();
  eawp.measurements.accel_filt.setZero();
  eawp.measurements.V_pitot = 0.f;

  // Init Input
  eawp.inputs.accel.setZero();
  eawp.inputs.rates.setZero();
  eawp.inputs.euler.setZero();
  eawp.inputs.RPM_pusher = 0.f;
  eawp.inputs.RPM_hover.setZero();
  eawp.inputs.skew = 0.f;
  eawp.inputs.elevator_angle = 0.f;

  // Init Innovation
  eawp.innovations.V_gnd.setZero();
  eawp.innovations.accel_filt.setZero();
  eawp.innovations.V_pitot = 0.f;

  // Init Forces
  eawp.forces.fuselage.setZero();
  eawp.forces.wing.setZero();
  eawp.forces.elevator.setZero();
  eawp.forces.hover.setZero();
  eawp.forces.pusher.setZero();

  // Init State Covariance
  eawp.P.setZero();
  eawp.P(EKF_AW_u_index,EKF_AW_u_index) = EKF_AW_P0_V_BODY;
  eawp.P(EKF_AW_v_index,EKF_AW_v_index) = EKF_AW_P0_V_BODY;
  eawp.P(EKF_AW_w_index,EKF_AW_w_index) = EKF_AW_P0_V_BODY;
  eawp.P(EKF_AW_mu_x_index,EKF_AW_mu_x_index) = EKF_AW_P0_MU;
  eawp.P(EKF_AW_mu_y_index,EKF_AW_mu_y_index) = EKF_AW_P0_MU;
  eawp.P(EKF_AW_mu_z_index,EKF_AW_mu_z_index) = EKF_AW_P0_MU;
  eawp.P(EKF_AW_k_x_index,EKF_AW_k_x_index) = EKF_AW_P0_OFFSET;
  eawp.P(EKF_AW_k_y_index,EKF_AW_k_y_index) = EKF_AW_P0_OFFSET;
  eawp.P(EKF_AW_k_z_index,EKF_AW_k_z_index) = EKF_AW_P0_OFFSET;

  // Init Process and Measurements Noise Matrix
  ekf_aw_update_params();

  // Init Filter Health
  eawp.health.healthy = true;
}

// Init function
void ekf_aw_init(void)
{
  // Process noise
  ekf_aw_params.Q_accel = EKF_AW_Q_ACCEL;    ///< accel process noise
  ekf_aw_params.Q_gyro = EKF_AW_Q_GYRO;      ///< gyro process noise
  ekf_aw_params.Q_mu = EKF_AW_Q_MU;          ///< wind process noise
  ekf_aw_params.Q_k = EKF_AW_Q_OFFSET;       ///< offset process noise

  // Measurement noise
  ekf_aw_params.R_V_gnd = EKF_AW_R_V_GND;      ///< speed measurement noise

  ekf_aw_params.R_accel_filt[0] = EKF_AW_R_ACCEL_FILT_X; 
  ekf_aw_params.R_accel_filt[1] = EKF_AW_R_ACCEL_FILT_Y; 
  ekf_aw_params.R_accel_filt[2] = EKF_AW_R_ACCEL_FILT_Z; ///< filtered accel measurement noise

  ekf_aw_params.R_V_pitot = EKF_AW_R_V_PITOT;      ///< airspeed measurement noise
  
  // Other options
  ekf_aw_params.use_pitot = EKF_AW_USE_PITOT;

  ekf_aw_params.use_model[0] = EKF_AW_USE_MODEL_BASED_X;
  ekf_aw_params.use_model[1] = EKF_AW_USE_MODEL_BASED_Y;
  ekf_aw_params.use_model[2] = EKF_AW_USE_MODEL_BASED_Z;

  ekf_aw_params.propagate_offset = EKF_AW_PROPAGATE_OFFSET;

  // Model based parameters 
    ekf_aw_params.vehicle_mass = EKF_AW_VEHICLE_MASS;
    // X Axis
    ekf_aw_params.k_fx_drag[0] = EKF_AW_K1_FX_DRAG; 
    ekf_aw_params.k_fx_drag[1] = EKF_AW_K2_FX_DRAG;

    ekf_aw_params.k_fx_fuselage[0] = EKF_AW_K1_FX_FUSELAGE;
    ekf_aw_params.k_fx_fuselage[1] = EKF_AW_K2_FX_FUSELAGE;
    ekf_aw_params.k_fx_fuselage[2] = EKF_AW_K3_FX_FUSELAGE;
    ekf_aw_params.k_fx_fuselage[3] = EKF_AW_K4_FX_FUSELAGE;
    ekf_aw_params.k_fx_hover[0] = EKF_AW_K1_FX_HOVER;
    ekf_aw_params.k_fx_hover[1] = EKF_AW_K2_FX_HOVER;
    ekf_aw_params.k_fx_hover[2] = EKF_AW_K3_FX_HOVER;

    ekf_aw_params.k_fx_wing[0] = EKF_AW_K1_FX_WING;
    ekf_aw_params.k_fx_wing[1] = EKF_AW_K2_FX_WING;
    ekf_aw_params.k_fx_wing[2] = EKF_AW_K3_FX_WING;
    ekf_aw_params.k_fx_wing[3] = EKF_AW_K4_FX_WING;
    ekf_aw_params.k_fx_wing[4] = EKF_AW_K5_FX_WING;

    ekf_aw_params.k_fx_push[0] = EKF_AW_K1_FX_PUSH;
    ekf_aw_params.k_fx_push[1] = EKF_AW_K2_FX_PUSH;
    ekf_aw_params.k_fx_push[2] = EKF_AW_K3_FX_PUSH;

    ekf_aw_params.k_fx_elev[0] = EKF_AW_K1_FX_ELEV;
    ekf_aw_params.k_fx_elev[1] = EKF_AW_K2_FX_ELEV;
    ekf_aw_params.k_fx_elev[2] = EKF_AW_K3_FX_ELEV;

    // Y Axis
    ekf_aw_params.k_fy_beta = EKF_AW_K_FY_BETA;
    ekf_aw_params.k_fy_v = EKF_AW_K_FY_V;

    ekf_aw_params.k_fy_wing[0] = EKF_AW_K1_FY_WING;
    ekf_aw_params.k_fy_wing[1] = EKF_AW_K2_FY_WING;
    ekf_aw_params.k_fy_wing[2] = EKF_AW_K3_FY_WING;
    ekf_aw_params.k_fy_wing[3] = EKF_AW_K4_FY_WING;
    ekf_aw_params.k_fy_wing[4] = EKF_AW_K5_FY_WING;

    // Z Axis
    ekf_aw_params.k_fz_fuselage[0] = EKF_AW_K1_FZ_FUSELAGE;
    ekf_aw_params.k_fz_fuselage[1] = EKF_AW_K2_FZ_FUSELAGE;
    ekf_aw_params.k_fz_fuselage[2] = EKF_AW_K3_FZ_FUSELAGE;
    ekf_aw_params.k_fz_fuselage[3] = EKF_AW_K4_FZ_FUSELAGE;

    ekf_aw_params.k_fz_wing[0] = EKF_AW_K1_FZ_WING;
    ekf_aw_params.k_fz_wing[1] = EKF_AW_K2_FZ_WING;
    ekf_aw_params.k_fz_wing[2] = EKF_AW_K3_FZ_WING;
    ekf_aw_params.k_fz_wing[3] = EKF_AW_K4_FZ_WING;

    ekf_aw_params.k_fz_hover[0] = EKF_AW_K1_FZ_HOVER;
    ekf_aw_params.k_fz_hover[1] = EKF_AW_K2_FZ_HOVER;
    ekf_aw_params.k_fz_hover[2] = EKF_AW_K3_FZ_HOVER;
    ekf_aw_params.k_fz_hover[3] = EKF_AW_K4_FZ_HOVER;
    ekf_aw_params.k_fz_hover[4] = EKF_AW_K5_FZ_HOVER;

    ekf_aw_params.k_fz_elev[0] = EKF_AW_K1_FZ_ELEV;
    ekf_aw_params.k_fz_elev[1] = EKF_AW_K2_FZ_ELEV;
    
  // Init state and measurements
  init_ekf_aw_state();

  // Init crashes number
  eawp.health.crashes_n = 0;

  // Init quick convergence
  ekf_aw_params.quick_convergence = false;

}

void ekf_aw_update_params(void)
{
  // Update Process Noise Q Matrix
  eawp.Q(EKF_AW_Q_accel_x_index,EKF_AW_Q_accel_x_index) = ekf_aw_params.Q_accel;
  eawp.Q(EKF_AW_Q_accel_y_index,EKF_AW_Q_accel_y_index) = ekf_aw_params.Q_accel;
  eawp.Q(EKF_AW_Q_accel_z_index,EKF_AW_Q_accel_z_index) = ekf_aw_params.Q_accel;

  eawp.Q(EKF_AW_Q_gyro_x_index,EKF_AW_Q_gyro_x_index) = ekf_aw_params.Q_gyro;
  eawp.Q(EKF_AW_Q_gyro_y_index,EKF_AW_Q_gyro_y_index) = ekf_aw_params.Q_gyro;
  eawp.Q(EKF_AW_Q_gyro_z_index,EKF_AW_Q_gyro_z_index) = ekf_aw_params.Q_gyro;

  eawp.Q(EKF_AW_Q_mu_x_index,EKF_AW_Q_mu_x_index) = ekf_aw_params.Q_mu;
  eawp.Q(EKF_AW_Q_mu_y_index,EKF_AW_Q_mu_y_index) = ekf_aw_params.Q_mu;
  eawp.Q(EKF_AW_Q_mu_z_index,EKF_AW_Q_mu_z_index) = 1E-1f*ekf_aw_params.Q_mu;

  eawp.Q(EKF_AW_Q_k_x_index,EKF_AW_Q_k_x_index) = ekf_aw_params.Q_k;
  eawp.Q(EKF_AW_Q_k_y_index,EKF_AW_Q_k_y_index) = ekf_aw_params.Q_k;
  eawp.Q(EKF_AW_Q_k_z_index,EKF_AW_Q_k_z_index) = ekf_aw_params.Q_k;

  // Update Measurement Noise R Matrix
  eawp.R(EKF_AW_R_V_gnd_x_index,EKF_AW_R_V_gnd_x_index) =  ekf_aw_params.R_V_gnd;
  eawp.R(EKF_AW_R_V_gnd_y_index,EKF_AW_R_V_gnd_y_index) =  ekf_aw_params.R_V_gnd;
  eawp.R(EKF_AW_R_V_gnd_z_index,EKF_AW_R_V_gnd_z_index) =  ekf_aw_params.R_V_gnd;

  eawp.R(EKF_AW_R_a_x_filt_index,EKF_AW_R_a_x_filt_index) =  ekf_aw_params.R_accel_filt[0];
  eawp.R(EKF_AW_R_a_y_filt_index,EKF_AW_R_a_y_filt_index) =  ekf_aw_params.R_accel_filt[1];
  eawp.R(EKF_AW_R_a_z_filt_index,EKF_AW_R_a_z_filt_index) =  ekf_aw_params.R_accel_filt[2];

  eawp.R(EKF_AW_R_V_pitot_index,EKF_AW_R_V_pitot_index) =  ekf_aw_params.R_V_pitot;
}

// Reset function
void ekf_aw_reset(void)
{
  // Only reset state, measurement and innovation
  init_ekf_aw_state();
}

// Full propagation
void ekf_aw_propagate(struct FloatVect3 *acc,struct FloatRates *gyro, struct FloatEulers *euler, float *pusher_RPM,float *hover_RPM_array, float *skew, float *elevator_angle, FloatVect3 * V_gnd, FloatVect3 *acc_filt, float *V_pitot,float dt)
{
  /*
  x = [u v w mu_x mu_y mu_z k_x k_y k_z];
  u = [a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_angle];
  z = [V_x V_y V_z a_x a_y a_z];
  */
  
  // FOR DEBUG
  uint32_t tic = get_sys_time_usec(); uint32_t duration_1;uint32_t duration_2;uint32_t duration_3;uint32_t duration_4;uint32_t duration_5;uint32_t duration_6;uint32_t duration_7;uint32_t duration_8;uint32_t duration_9;uint32_t duration_10;uint32_t duration_11;

  // Exit filter if the filter crashed for more than 5s
  if (eawp.health.crashes_n > floor(5.0f/dt)){
    eawp.health.healthy = false;
    return;
  }

  /////////////////////////////////
  //     Preparing inputs        //
  /////////////////////////////////

  // Inputs
  eawp.inputs.accel(0) = acc->x;     eawp.inputs.accel(1) = acc->y;       eawp.inputs.accel(2) = acc->z;
  eawp.inputs.rates(0) = gyro->p;    eawp.inputs.rates(1) = gyro->q;      eawp.inputs.rates(2) = gyro->r;
  eawp.inputs.euler(0) = euler->phi; eawp.inputs.euler(1) = euler->theta; eawp.inputs.euler(2) = euler->psi;

  eawp.inputs.RPM_pusher = *pusher_RPM;
  
  eawp.inputs.RPM_hover(0) = hover_RPM_array[0]; eawp.inputs.RPM_hover(1) = hover_RPM_array[1]; eawp.inputs.RPM_hover(2) = hover_RPM_array[2]; eawp.inputs.RPM_hover(3) = hover_RPM_array[3];
  //std::cout << "Hover prop:\n" << eawp.inputs.RPM_hover << std::endl;

  eawp.inputs.skew = *skew;
  // Polyval from measured skew to real skew // TO DO: complementary filter between SP (low pass) and measured skew (high pass)
  eawp.inputs.skew = EKF_AW_SKEW_POLY_0 + EKF_AW_SKEW_POLY_1 * eawp.inputs.skew + EKF_AW_SKEW_POLY_2 * eawp.inputs.skew * eawp.inputs.skew;

  Bound(eawp.inputs.skew,0.0f,deg2rad*90.0f); // Saturate 0-90 deg

  eawp.inputs.elevator_angle = *elevator_angle;
  eawp.inputs.elevator_angle = eawp.inputs.elevator_angle < deg2rad*EKF_AW_ELEV_MIN_ANGLE ? deg2rad*EKF_AW_ELEV_MIN_ANGLE : eawp.inputs.elevator_angle > deg2rad*EKF_AW_ELEV_MAX_ANGLE ? deg2rad*EKF_AW_ELEV_MAX_ANGLE : eawp.inputs.elevator_angle; // Saturate

  // Measurements
  eawp.measurements.V_gnd(0) = V_gnd->x;eawp.measurements.V_gnd(1) = V_gnd->y;eawp.measurements.V_gnd(2) = V_gnd->z;
  eawp.measurements.accel_filt(0) = acc_filt->x; eawp.measurements.accel_filt(1) = acc_filt->y; eawp.measurements.accel_filt(2) = acc_filt->z;
  eawp.measurements.V_pitot = *V_pitot;

  // Variables used in matrices and precomputed values
  float V_a = eawp.state.V_body.norm(); //airspeed
  float u = eawp.state.V_body(0);
  float v = eawp.state.V_body(1);
  float w = eawp.state.V_body(2);
  float sign_u = u < 0.0f ? -1.0f : u > 0.0f ? 1.0f : 0.0f;
  float sign_v = v < 0.0f ? -1.0f : v > 0.0f ? 1.0f : 0.0f;

  float p = eawp.inputs.rates(0);
  float q = eawp.inputs.rates(1);
  float r = eawp.inputs.rates(2);

  float phi = eawp.inputs.euler(0);
  float theta = eawp.inputs.euler(1);
  float psi = eawp.inputs.euler(2);

  // FOR DEBUG
  if (EKF_AW_DEBUG){
  duration_1 = get_sys_time_usec()-tic;
  }

  float cos_phi = cosf(phi);
  float sin_phi = sinf(phi);
  float cos_theta = cosf(theta);
  float sin_theta = sinf(theta);
  float cos_psi = cosf(psi);
  float sin_psi = sinf(psi);

  float cos_skew = cosf(eawp.inputs.skew);
  float sin_skew = sinf(eawp.inputs.skew);

  // FOR DEBUG
  #if EKF_AW_DEBUG
  duration_2 = get_sys_time_usec()-tic;
  #endif

  // DCM from Euler Angles
  matrix::Matrix3f dcm;
  dcm(0, 0) = cos_theta * cos_psi;
  dcm(0, 1) = -cos_phi * sin_psi + sin_phi * sin_theta * cos_psi;
  dcm(0, 2) = sin_phi * sin_psi + cos_phi * sin_theta * cos_psi;

  dcm(1, 0) = cos_theta * sin_psi;
  dcm(1, 1) = cos_phi * cos_psi + sin_phi * sin_theta * sin_psi;
  dcm(1, 2) = -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi;

  dcm(2, 0) = -sin_theta;
  dcm(2, 1) = sin_phi * cos_theta;
  dcm(2, 2) = cos_phi * cos_theta;

  // Calculate Angle of Attack
  float aoa = atan2f(w, fabsf(u)<1.E-5 ? 1E-5 : u); // Protected alpha calculation
  float sat_aoa;
  Bound(sat_aoa,deg2rad*EKF_AW_AOA_MIN_ANGLE, deg2rad*EKF_AW_AOA_MAX_ANGLE); // Saturate alpha
  float tan_aoa = tanf(aoa);

  // Derivative of Angle of Attack
  float diff_alpha_u = 0.0f;
  float diff_alpha_w = 0.0f;
  if ((fabsf(u) > 1E-5) & (fabsf(w)>1E-5)){
    diff_alpha_u = -w/(u*u + w*w);
    diff_alpha_w = u/(u*u  +w*w);
  } 

  // Calculate Sideslip
  float beta = 0.0f;
  if (V_a > 1E-5){
    beta = asinf(v/V_a < -1.0f ? -1.0f : v/V_a > 1.0f ? 1.0f : v/V_a); // Ratio is kept between -1 and 1
  }

  float hover_RPM_mean = (eawp.inputs.RPM_hover(0)+eawp.inputs.RPM_hover(1)+eawp.inputs.RPM_hover(2)+eawp.inputs.RPM_hover(3))/4.0f;

  // Verify vehicle mass is not 0
  ekf_aw_params.vehicle_mass = fabsf(ekf_aw_params.vehicle_mass)<1E-1 ? 1E-1 : ekf_aw_params.vehicle_mass;

  // FOR DEBUG
  #if EKF_AW_DEBUG
  duration_3 = get_sys_time_usec()-tic;
  #endif

  /////////////////////////////////
  //    Special Conditions       //
  /////////////////////////////////

  // A_z covariance Scheduling
  /*
              Cov
              ▲
      10^Gain │ ────────
              │         \
              │          \
              │           \
              │            \
          1   │             ────────
              │
              └─────────┬───┬──────► Skew Angle
                        │   │
                     Start  End  
  
  */
  float exp_az = 0.0f;
  if (eawp.inputs.skew<deg2rad*EKF_AW_AZ_SCHED_START_DEG){
    exp_az = EKF_AW_AZ_SCHED_GAIN;
  }
  else if (eawp.inputs.skew>deg2rad*EKF_AW_AZ_SCHED_END_DEG){
    exp_az = 0.0f;
  }
  else{
    // Make sure Start and End points are not the same
    if (EKF_AW_AZ_SCHED_START_DEG!=EKF_AW_AZ_SCHED_END_DEG){
    exp_az = EKF_AW_AZ_SCHED_GAIN+(eawp.inputs.skew-deg2rad*EKF_AW_AZ_SCHED_START_DEG)*((EKF_AW_AZ_SCHED_GAIN)/(deg2rad*(EKF_AW_AZ_SCHED_START_DEG-EKF_AW_AZ_SCHED_END_DEG)));
    }
  }
  Bound(exp_az,0,EKF_AW_AZ_SCHED_GAIN); // Saturate
  eawp.R(EKF_AW_R_a_z_filt_index,EKF_AW_R_a_z_filt_index) = ekf_aw_params.R_accel_filt[2]*powf(10.0f,exp_az);

  // A_x covariance Scheduling
  /*
              Cov
              ▲
      10^Gain │              /────────
              │             /
              │            /
              │           / 
              │          /  
          1   │ ────────/   
              │
              └─────────┬───┬──────► Skew Angle
                        │   │
                     Start  End  
  
  */
  float exp_ax = 0.0f;
  if (eawp.inputs.skew<deg2rad*EKF_AW_AX_SCHED_START_DEG){
    exp_ax = 0.0f;
  }
  else if (eawp.inputs.skew>deg2rad*EKF_AW_AX_SCHED_END_DEG){
    exp_ax = EKF_AW_AX_SCHED_GAIN;
  }
  else{
    // Make sure Start and End points are not the same
    if (EKF_AW_AX_SCHED_START_DEG!=EKF_AW_AX_SCHED_END_DEG){
    exp_ax = (eawp.inputs.skew-deg2rad*EKF_AW_AX_SCHED_START_DEG)*((EKF_AW_AX_SCHED_GAIN)/(deg2rad*(EKF_AW_AX_SCHED_END_DEG-EKF_AW_AX_SCHED_START_DEG)));
    }
  }
  Bound(exp_ax,0,EKF_AW_AX_SCHED_GAIN); // Saturate
  eawp.R(EKF_AW_R_a_x_filt_index,EKF_AW_R_a_x_filt_index) = ekf_aw_params.R_accel_filt[0]*powf(10.0f,exp_ax);

  // Quick Convergence Mode
  // Increases wind covariance and decreases accel covariance
  if (ekf_aw_params.quick_convergence){
    eawp.Q(EKF_AW_Q_mu_x_index,EKF_AW_Q_mu_x_index) = ekf_aw_params.Q_mu*powf(10.0f,EKF_AW_AZ_QUICK_CONV_MU_GAIN);
    eawp.Q(EKF_AW_Q_mu_y_index,EKF_AW_Q_mu_y_index) = ekf_aw_params.Q_mu*powf(10.0f,EKF_AW_AZ_QUICK_CONV_MU_GAIN);

    eawp.R(EKF_AW_R_a_x_filt_index,EKF_AW_R_a_x_filt_index) = eawp.R(EKF_AW_R_a_x_filt_index,EKF_AW_R_a_x_filt_index)*powf(10.0f,EKF_AW_AZ_QUICK_CONV_ACCEL_GAIN);
    eawp.R(EKF_AW_R_a_y_filt_index,EKF_AW_R_a_y_filt_index) = eawp.R(EKF_AW_R_a_y_filt_index,EKF_AW_R_a_y_filt_index)*powf(10.0f,EKF_AW_AZ_QUICK_CONV_ACCEL_GAIN);
    eawp.R(EKF_AW_R_a_z_filt_index,EKF_AW_R_a_z_filt_index) = eawp.R(EKF_AW_R_a_z_filt_index,EKF_AW_R_a_z_filt_index)*powf(10.0f,EKF_AW_AZ_QUICK_CONV_ACCEL_GAIN);
  }
  else{
    eawp.Q(EKF_AW_Q_mu_x_index,EKF_AW_Q_mu_x_index) = ekf_aw_params.Q_mu;
    eawp.Q(EKF_AW_Q_mu_y_index,EKF_AW_Q_mu_y_index) = ekf_aw_params.Q_mu;

    eawp.R(EKF_AW_R_a_x_filt_index,EKF_AW_R_a_x_filt_index) = eawp.R(EKF_AW_R_a_x_filt_index,EKF_AW_R_a_x_filt_index);
    eawp.R(EKF_AW_R_a_y_filt_index,EKF_AW_R_a_y_filt_index) = eawp.R(EKF_AW_R_a_y_filt_index,EKF_AW_R_a_y_filt_index);
    eawp.R(EKF_AW_R_a_z_filt_index,EKF_AW_R_a_z_filt_index) = eawp.R(EKF_AW_R_a_z_filt_index,EKF_AW_R_a_z_filt_index);
  }

  /////////////////////////////////
  //      Propagate States       //
  /////////////////////////////////

  // Propagate state by Euler Integration
  matrix::Vector3f state_dev = -eawp.inputs.rates.cross(eawp.state.V_body)+dcm.transpose() * gravity + eawp.inputs.accel; // Verified and compared to Matlab output
  
  // Check if state derivative is a NAN and only propagate if its not
  bool anyNan = false;
  for(int i = 0; i < 3; i++) {
    if(std::isnan(state_dev(i))){
      anyNan = true;
      break;
    }
  }

  if (anyNan){
    eawp.health.healthy = false;
    eawp.health.crashes_n += 1;
  }
  else{
    eawp.state.V_body += state_dev * dt;
  }

  /////////////////////////////////
  //    Propagate Covariance     //
  /////////////////////////////////

  // FOR DEBUG
  #if EKF_AW_DEBUG
  duration_4 = get_sys_time_usec()-tic;
  #endif

  // This is an optimized version using code generation from Matlab, as L and F are sparse matrix
  // Goes from approx 4000 operation to 120 operations
  eawp.P.setZero();
  eawp.P(0,0) = eawp.Q(0, 0) + q * (q * eawp.P(2, 2) - r * eawp.P(1, 2)) - r * (q * eawp.P(2, 1) - r * eawp.P(1, 1)) + v*v * eawp.Q(5, 5) + w*w * eawp.Q(4, 4);
  eawp.P(0,1) = r * (q * eawp.P(2, 0) - r * eawp.P(1, 0)) - p * (q * eawp.P(2, 2) - r * eawp.P(1, 2)) - u * v*eawp.Q(5, 5);
  eawp.P(0,2) = p * (q * eawp.P(2, 1) - r * eawp.P(1, 1)) - q * (q * eawp.P(2, 0) - r * eawp.P(1, 0)) - u * w*eawp.Q(4, 4);
  eawp.P(1,0) = r * (p * eawp.P(2, 1) - r * eawp.P(0, 1)) - q * (p * eawp.P(2, 2) - r * eawp.P(0, 2)) - u * v*eawp.Q(5, 5);
  eawp.P(1,1) = eawp.Q(1, 1) + p * (p * eawp.P(2, 2) - r * eawp.P(0, 2)) - r * (p * eawp.P(2, 0) - r * eawp.P(0, 0)) + u*u * eawp.Q(5, 5) + w*w * eawp.Q(3, 3);
  eawp.P(1,2) = q * (p * eawp.P(2, 0) - r * eawp.P(0, 0)) - p * (p * eawp.P(2, 1) - r * eawp.P(0, 1)) - v * w*eawp.Q(3, 3);
  eawp.P(2,0) = q * (p * eawp.P(1, 2) - q * eawp.P(0, 2)) - r * (p * eawp.P(1, 1) - q * eawp.P(0, 1)) - u * w*eawp.Q(4, 4);
  eawp.P(2,1) = r * (p * eawp.P(1, 0) - q * eawp.P(0, 0)) - p * (p * eawp.P(1, 2) - q * eawp.P(0, 2)) - v * w*eawp.Q(3, 3);
  eawp.P(2,2) = eawp.Q(2, 2) + p * (p * eawp.P(1, 1) - q * eawp.P(0, 1)) - q * (p * eawp.P(1, 0) - q * eawp.P(0, 0)) + u*u * eawp.Q(4, 4) + v*v * eawp.Q(3, 3);
  eawp.P(3,3) = eawp.Q(6, 6);
  eawp.P(4,4) = eawp.Q(7, 7);
  eawp.P(5,5) = eawp.Q(8, 8);
  eawp.P(6,6) = eawp.Q(9, 9);
  eawp.P(7,7) = eawp.Q(10, 10);
  eawp.P(8,8) = eawp.Q(11, 11);
  
  // Other way of calculating covariance, but involves more operations:
  /*
  // F Matrix
  //F = [ dx_1/dx_1 dx_1/dx_2 ... dx_1/dx_n ;
  //      dx_2/dx_1 dx_2/dx_2 ....dx_2/dx_n ;
  //      ...                               ;
  //      dx_n/dx_1 dx_n/dx_2 ....dx_n/dx_n ];
  //
  //Validated and compared to Matlab output for F
  //Generated with Matlab script
  
  EKF_Aw_Cov F;
  F.setZero();
  F(0,1) = r;
  F(0,2) = -q;
  F(1,0) = -r;
  F(1,2) = p;
  F(2,0) = q;
  F(2,1) = -p;

  // L Matrix
  //
  //L = [ dx_1/dw_1 dx_1/dw_2 ... dx_1/dw_m ;
  //      dx_2/dw_1 dx_2/dw_2 ....dx_2/dw_m ;
  //      ...                               ;
  //      dx_n/dw_1 dx_n/dw_2 ....dx_n/dw_m ];
  //
  //Validated and compared to Matlab output for L
  //Generated with Matlab script
  
  matrix::Matrix<float, EKF_AW_COV_SIZE, EKF_AW_Q_SIZE> L;
  L.setZero();
  L(0,0) =1;
  L(0,4) =-w;
  L(0,5) =v;
  L(1,1) =1;
  L(1,3) =w;
  L(1,5) =-u;
  L(2,2) =1;
  L(2,3) =-v;
  L(2,4) =u;
  L(3,6) =1;
  L(4,7) =1;
  L(5,8) =1;
  L(6,9) =1;
  L(7,10) =1;
  L(8,11) =1;
  
  eawp.P = F * eawp.P * F.transpose() + L * eawp.Q * L.transpose();
  */

  // FOR DEBUG
  #if EKF_AW_DEBUG
  duration_5 = get_sys_time_usec()-tic;
  #endif
  ///////////////////////////////////////////
  //  Measurement estimation from state    //
  ///////////////////////////////////////////

  // Calculate acceleration estimation from filter states
  float a_x;
  float a_y; 
  float a_z;

  // A_x
  if (ekf_aw_params.use_model[0]){
    // Calculate forces in x axis
    eawp.forces.fuselage(0) = fx_fuselage(&eawp.inputs.skew,&sat_aoa,&u);
    eawp.forces.wing(0) = EKF_AW_WING_INSTALLED ? fx_wing(&eawp.inputs.skew,&sat_aoa,&u) : 0;
    eawp.forces.elevator(0) = fx_elevator(&eawp.inputs.elevator_angle, &V_a);
    eawp.forces.hover(0)    = fx_fy_hover(&hover_RPM_mean, &u);
    eawp.forces.pusher(0)   = fx_pusher(&eawp.inputs.RPM_pusher, &u);

    // Acceleration is sum of forces
    a_x = (eawp.forces.fuselage(0) +
           eawp.forces.wing(0) +
           eawp.forces.elevator(0) +
           eawp.forces.hover(0) +
           eawp.forces.pusher(0) +
           eawp.state.offset(0)*u*u*sign_u)/ekf_aw_params.vehicle_mass;
  }
  else{
    a_x = (ekf_aw_params.k_fx_drag[0]*u +
           ekf_aw_params.k_fx_drag[1]*u*u*sign_u +
           eawp.state.offset(0)*u*u*sign_u)/ekf_aw_params.vehicle_mass;

    // Pusher prop
    eawp.forces.pusher(0)   = fx_pusher(&eawp.inputs.RPM_pusher, &u);
    a_x += eawp.forces.pusher(0)/ekf_aw_params.vehicle_mass;
  }

  // A_y
  #if EKF_AW_USE_BETA
    a_y = beta*ekf_aw_params.k_fy_beta*(V_a*V_a)/ekf_aw_params.vehicle_mass + eawp.state.offset(1); // No need to bound beta equation as beta is generated by asin, which is bounded between -pi/2 and pi/2
  #else
    a_y = sign_v*v*v*ekf_aw_params.k_fy_v/ekf_aw_params.vehicle_mass + eawp.state.offset(1);
  #endif
  if (ekf_aw_params.use_model[1]){
    // Wing sideforce
    eawp.forces.wing(1) = EKF_AW_WING_INSTALLED ? fy_wing(&eawp.inputs.skew,&sat_aoa,&u) : 0;
    a_y += eawp.forces.wing(1)/ekf_aw_params.vehicle_mass;
  }

  // A_z
  if (ekf_aw_params.use_model[2]){
    // Calculate forces in x axis
    eawp.forces.fuselage(2) = fz_fuselage(&eawp.inputs.skew,&sat_aoa,&V_a);    
    eawp.forces.elevator(2) = fz_elevator(&eawp.inputs.elevator_angle, &V_a);
    eawp.forces.wing(2)     = EKF_AW_WING_INSTALLED ? fz_wing(&eawp.inputs.skew,&sat_aoa,&V_a) : 0;    
    eawp.forces.hover(2)    = fz_hover(eawp.inputs.RPM_hover, &V_a);

    // Acceleration is sum of forces
    a_z = (eawp.forces.fuselage(2) +
           eawp.forces.elevator(2) + 
           eawp.forces.wing(2) +
           eawp.forces.hover(2))/ekf_aw_params.vehicle_mass + eawp.state.offset(1);    
  }
  else{
    a_z = eawp.measurements.accel_filt(2);
  }

  matrix::Vector3f accel_est = {a_x, a_y, a_z};

  // Innovation
  eawp.innovations.V_gnd = eawp.measurements.V_gnd - (dcm * eawp.state.V_body + eawp.state.wind); // Ground speed in NED Frame
  eawp.innovations.accel_filt = eawp.measurements.accel_filt - accel_est;
  eawp.innovations.V_pitot = eawp.measurements.V_pitot - eawp.state.V_body(0);

  //std::cout << "State wind:\n" << eawp.state.wind << std::endl;
  //std::cout << "V_body:\n" << eawp.state.V_body << std::endl;
  //std::cout << "Q :\n" << eawp.Q(EKF_AW_Q_k_x_index,EKF_AW_Q_k_x_index) << std::endl;
  //std::cout << "V_body_gnd:\n" << quat.toRotationMatrix() * eawp.state.V_body << std::endl;
  //std::cout << "V_gnd:\n" << eawp.measurements.V_gnd << std::endl;
  //std::cout << "Innov V_gnd:\n" << eawp.innovations.V_gnd << std::endl;
  //std::cout << "Innov accel_filt:\n" << eawp.innovations.accel_filt << std::endl;
  //std::cout << "Euler:\n" << eawp.inputs.euler << std::endl;

  // FOR DEBUG
  #if EKF_AW_DEBUG
  duration_6 = get_sys_time_usec()-tic;
  #endif

  /////////////////////////////////
  //         State Update        //
  /////////////////////////////////

  // G Matrix Calculation
  /*
  G = [ dg_1/dx_1 dg_1/dx_2 ... dg_1/dx_m ;
       dg_2/dx_1 dg_2/dx_2 ....dg_2/dx_m ;
       ...                               ;
       dg_n/dx_1 dg_n/dx_2 ....dg_n/dx_m ];
  
  Generated with Matlab script
  */
  matrix::Matrix<float, EKF_AW_R_SIZE, EKF_AW_COV_SIZE> G;
  G.setZero();
  // V_gnd related lines (verified and compared to Matlab output)
  //Generated with Matlab script
  G(0,0) = cos_psi*cos_theta;
  G(0,1) = cos_psi*sin_phi*sin_theta - cos_phi*sin_psi;
  G(0,2) = sin_phi*sin_psi + cos_phi*cos_psi*sin_theta;
  G(0,3) = 1;
  G(1,0) = cos_theta*sin_psi;
  G(1,1) = cos_phi*cos_psi + sin_phi*sin_psi*sin_theta;
  G(1,2) = cos_phi*sin_psi*sin_theta - cos_psi*sin_phi;
  G(1,4) = 1;
  G(2,0) = -sin_theta;
  G(2,1) = cos_theta*sin_phi;
  G(2,2) = cos_phi*cos_theta;
  G(2,5) = 1;

  // A_x_filt related lines
  if (ekf_aw_params.use_model[0]){ 
    // Validated and compared to Matlab output for G
    //Generated with Matlab script

    // Pusher contribution
    G(3,0) += (ekf_aw_params.k_fx_push[2] + eawp.inputs.RPM_pusher*ekf_aw_params.k_fx_push[1])/ekf_aw_params.vehicle_mass; 
    
    // Fuselage contribution
    float temp_1 = ekf_aw_params.k_fx_fuselage[1] + ekf_aw_params.k_fx_fuselage[3]*aoa*aoa + ekf_aw_params.k_fx_fuselage[0]*cos_skew + ekf_aw_params.k_fx_fuselage[2]*aoa;
    G(3,0) += (2*u*(temp_1) + (ekf_aw_params.k_fx_fuselage[2]*diff_alpha_u + 2*ekf_aw_params.k_fx_fuselage[3]*aoa*diff_alpha_u)*V_a*V_a)/ekf_aw_params.vehicle_mass; 
    G(3,1) += (2*v*(temp_1)/ekf_aw_params.vehicle_mass);
    G(3,2) += (2*w*(temp_1) + (ekf_aw_params.k_fx_fuselage[2]*diff_alpha_w + 2*ekf_aw_params.k_fx_fuselage[3]*aoa*diff_alpha_w)*V_a*V_a)/ekf_aw_params.vehicle_mass;
    
    // Hover prop contribution
    G(3,0) += (2*ekf_aw_params.k_fx_hover[0]*sign_u*u + (0.5*ekf_aw_params.k_fx_hover[1]*sign_u*hover_RPM_mean*hover_RPM_mean)/sqrtf(fabsf(fabsf(u)<1E-5 ? 1E-5 : u))+ ekf_aw_params.k_fx_hover[2] * sign_u)/ekf_aw_params.vehicle_mass; // TO DO: Verify the abs()

    #if EKF_AW_WING_INSTALLED
    // Wing contribution
    float temp_2 = ekf_aw_params.k_fx_wing[0] + ekf_aw_params.k_fx_wing[4]*eawp.inputs.skew + (sin_skew*sin_skew + ekf_aw_params.k_fx_wing[3])*(ekf_aw_params.k_fx_wing[2]*aoa*aoa + ekf_aw_params.k_fx_wing[1]*aoa);
    float temp_3 = sin_skew*sin_skew + ekf_aw_params.k_fx_wing[3];
    G(3,0) += (2*u*temp_2 + temp_3*(ekf_aw_params.k_fx_wing[1]*diff_alpha_u + 2*ekf_aw_params.k_fx_wing[2]*aoa*diff_alpha_u)*V_a*V_a)/ekf_aw_params.vehicle_mass;
    G(3,1) += (2*v*temp_2)/ekf_aw_params.vehicle_mass;
    G(3,2) += (2*w*temp_2 + temp_3*(ekf_aw_params.k_fx_wing[1]*diff_alpha_w + 2*ekf_aw_params.k_fx_wing[2]*aoa*diff_alpha_w)*V_a*V_a)/ekf_aw_params.vehicle_mass;
    #endif
    // Elevator contribution
    float temp_4 = ekf_aw_params.k_fx_elev[2]*eawp.inputs.elevator_angle*eawp.inputs.elevator_angle + ekf_aw_params.k_fx_elev[1]*eawp.inputs.elevator_angle + ekf_aw_params.k_fx_elev[0];
    G(3,0) += (2*u*temp_4)/ekf_aw_params.vehicle_mass;
    G(3,1) += (2*v*temp_4)/ekf_aw_params.vehicle_mass;
    G(3,2) += (2*w*temp_4)/ekf_aw_params.vehicle_mass;
        
  }
  else{
    G(3,0) = (ekf_aw_params.k_fx_drag[0] + 2*ekf_aw_params.k_fx_drag[1]*u*sign_u)/ekf_aw_params.vehicle_mass; // Simplified version (using u instead of V_a)

    // Pusher contribution
    G(3,0) += (ekf_aw_params.k_fx_push[2] + eawp.inputs.RPM_pusher*ekf_aw_params.k_fx_push[1])/ekf_aw_params.vehicle_mass; 
  }
    // Offset contribution
    G(3,0) += (2*eawp.state.offset(0)*sign_u*u)/ekf_aw_params.vehicle_mass;
    G(3,6) = (u*u*sign_u)/ekf_aw_params.vehicle_mass;

  // A_y_filt related lines
  #if EKF_AW_USE_BETA
    float protected_V_a = fabsf(V_a)<1E-5 ? 1E-5 : V_a;
    float cos_beta = fabsf(cosf(beta))<1E-5 ? 1E-5 : cosf(beta);
    G(4,0) = 2*ekf_aw_params.k_fy_beta*u*beta - (ekf_aw_params.k_fy_beta*u*v)/(cos_beta*protected_V_a);
    G(4,1) = 2*ekf_aw_params.k_fy_beta*v*beta + (ekf_aw_params.k_fy_beta*(V_a - v*sinf(beta)))/cos_beta;
    G(4,2) = 2*ekf_aw_params.k_fy_beta*w*beta - (ekf_aw_params.k_fy_beta*v*w)/(cos_beta*protected_V_a);
  #else
    G(4,1) = 2*ekf_aw_params.k_fy_v*v*sign_v;
  #endif

  #if EKF_AW_WING_INSTALLED
    G(4,0) += (ekf_aw_params.k_fy_wing[0] * u * sinf(2 * eawp.inputs.skew))/ekf_aw_params.vehicle_mass;
  #endif

  G(4,7) = 1;

  // A_z_filt related lines
  if (ekf_aw_params.use_model[2]){ 
    // Validated and compared to Matlab output for G
    //Generated with Matlab script

    // Fuselage contribution
    float temp_5 = ekf_aw_params.k_fz_fuselage[1] + ekf_aw_params.k_fz_fuselage[3]*aoa*aoa + ekf_aw_params.k_fz_fuselage[0]*cos_skew + ekf_aw_params.k_fz_fuselage[2]*aoa;
    G(5,0) += (2*u*temp_5 + (ekf_aw_params.k_fz_fuselage[2]*diff_alpha_u + 2*ekf_aw_params.k_fz_fuselage[3]*aoa*diff_alpha_u)*V_a*V_a)/ekf_aw_params.vehicle_mass;
    G(5,1) += (2*v*temp_5)/ekf_aw_params.vehicle_mass;
    G(5,2) += (2*w*temp_5 + (ekf_aw_params.k_fz_fuselage[2]*diff_alpha_w + 2*ekf_aw_params.k_fz_fuselage[3]*aoa*diff_alpha_w)*V_a*V_a)/ekf_aw_params.vehicle_mass;

    #if EKF_AW_WING_INSTALLED
    // Wing contribution
    G(5,0) += (2*u*(sin_skew*sin_skew + ekf_aw_params.k_fz_wing[3])*(ekf_aw_params.k_fz_wing[0] + ekf_aw_params.k_fz_wing[2]*aoa*aoa + ekf_aw_params.k_fz_wing[1]*aoa) - (sin_skew*sin_skew + ekf_aw_params.k_fz_wing[3])*(-ekf_aw_params.k_fz_wing[1]*diff_alpha_u + -2*ekf_aw_params.k_fz_wing[2]*aoa*diff_alpha_u)*V_a*V_a)/ekf_aw_params.vehicle_mass;
    G(5,1) += (2*v*(sin_skew*sin_skew + ekf_aw_params.k_fz_wing[3])*(ekf_aw_params.k_fz_wing[0] + ekf_aw_params.k_fz_wing[2]*aoa*aoa + ekf_aw_params.k_fz_wing[1]*aoa)/ekf_aw_params.vehicle_mass);
    G(5,2) += ((sin_skew*sin_skew + ekf_aw_params.k_fz_wing[3])*(ekf_aw_params.k_fz_wing[1]*diff_alpha_w + 2*ekf_aw_params.k_fz_wing[2]*aoa*diff_alpha_w)*V_a*V_a + 2*w*(sin_skew*sin_skew + ekf_aw_params.k_fz_wing[3])*(ekf_aw_params.k_fz_wing[0] + ekf_aw_params.k_fz_wing[2]*aoa*aoa + ekf_aw_params.k_fz_wing[1]*aoa))/ekf_aw_params.vehicle_mass;
    #endif

    // Elevator contribution
    float temp_6 = ekf_aw_params.k_fz_elev[1]*eawp.inputs.elevator_angle + ekf_aw_params.k_fz_elev[0];
    G(5,0) += (2*u*temp_6)/ekf_aw_params.vehicle_mass;
    G(5,1) += (2*v*temp_6)/ekf_aw_params.vehicle_mass;
    G(5,2) += (2*w*temp_6)/ekf_aw_params.vehicle_mass;

    // Hover prop contribution
    G(5,0) += (ekf_aw_params.k_fz_hover[4] * sign_u)/ekf_aw_params.vehicle_mass;    
  }
    // Offset contribution
    G(5,8) = 1;

  // V_pitot related lines 
  G(6,0) = 1;

  // FOR DEBUG
  #if EKF_AW_DEBUG
  duration_7 = get_sys_time_usec()-tic;
  #endif

  // Innovation S Matrix Calculation
  matrix::SquareMatrix<float, EKF_AW_R_SIZE> S = G * eawp.P * G.transpose() + eawp.R; // M = identity

  // FOR DEBUG
  #if EKF_AW_DEBUG
  duration_8 = get_sys_time_usec()-tic;
  #endif

  // Kalman Gain Calculation
  matrix::Matrix<float, EKF_AW_COV_SIZE, EKF_AW_R_SIZE> K = eawp.P * G.transpose() * S.I();

  // FOR DEBUG
  #if EKF_AW_DEBUG
  duration_9 = get_sys_time_usec()-tic;
  #endif

  // Check if Kalman Gain contains any nan. Only update state if it is not NAN
  anyNan = false;
  for(int i = 0; i < EKF_AW_COV_SIZE; i++) {
    for(int j = 0; j<EKF_AW_R_SIZE;j++){
      if(std::isnan(K(i,j))){
        anyNan = true;
        break;
      }   
    }
  }

  if (anyNan){
    eawp.health.healthy = false;
    eawp.health.crashes_n += 1;
  }
  else{
    eawp.health.healthy = true;

    // Temp variable to slice K
    matrix::Matrix<float,3,3> K_slice_3;
    matrix::Matrix<float,3,1> K_slice_1;

    // State update using V_gnd
    K_slice_3 = K.slice<3,3>(0,0); eawp.state.V_body  += K_slice_3 * eawp.innovations.V_gnd; 
    K_slice_3 = K.slice<3,3>(3,0); eawp.state.wind    += K_slice_3 * eawp.innovations.V_gnd;
    if (ekf_aw_params.propagate_offset){
      K_slice_3 = K.slice<3,3>(6,0); eawp.state.offset  += K_slice_3 * eawp.innovations.V_gnd;
    } 
    
    // State update using a_filt
    K_slice_3 = K.slice<3,3>(0,3); eawp.state.V_body  += K_slice_3 * eawp.innovations.accel_filt; 
    K_slice_3 = K.slice<3,3>(3,3);eawp.state.wind    += K_slice_3 * eawp.innovations.accel_filt;
    if (ekf_aw_params.propagate_offset){
      K_slice_3 = K.slice<3,3>(6,3);eawp.state.offset  += K_slice_3 * eawp.innovations.accel_filt;
    }
    
    // State update using V_pitot (if available)
    if (ekf_aw_params.use_pitot){
      K_slice_1 = K.slice<3,1>(0,6); eawp.state.V_body  += K_slice_1 * eawp.innovations.V_pitot; 
      K_slice_1 = K.slice<3,1>(3,6); eawp.state.wind    += K_slice_1 * eawp.innovations.V_pitot; 
      if (ekf_aw_params.propagate_offset){
        K_slice_1 = K.slice<3,1>(6,6); eawp.state.offset  += K_slice_1 * eawp.innovations.V_pitot;
      }
    }
    // FOR DEBUG
    #if EKF_AW_DEBUG
    duration_10 = get_sys_time_usec()-tic;
    #endif

    // Covariance update
    matrix::SquareMatrix<float,EKF_AW_COV_SIZE> eye;
    eye.setIdentity();
    eawp.P = (eye - K * G) * eawp.P;
  }
    
  // FOR DEBUG
  #if EKF_AW_DEBUG
    duration_11 = get_sys_time_usec()-tic;

    eawp.forces.fuselage(0) = duration_1*1.0f;
    eawp.forces.fuselage(1) = duration_2*1.0f;
    eawp.forces.fuselage(2) = duration_3*1.0f;

    eawp.forces.wing(0) = duration_4*1.0f;
    eawp.forces.wing(1) = duration_5*1.0f;
    eawp.forces.wing(2) = duration_6*1.0f;

    eawp.forces.elevator(0) = duration_7*1.0f;
    eawp.forces.elevator(1) = duration_8*1.0f;
    eawp.forces.elevator(2) = duration_9*1.0f;

    eawp.forces.hover(0) = duration_10*1.0f;
    eawp.forces.hover(1) = duration_11*1.0f;
  #endif

  //std::cout << "subs:\n" << EKF_Aw_Cov::Identity() - K * G  << std::endl;
  //std::cout << "Cov matrix:\n" << eawp.P << std::endl;
  //std::cout << "S inverse:\n" << S.inverse() << std::endl;
  //std::cout << "K V_body V_gnd:\n" << K.block<3,3>(0,0) * eawp.innovations.V_gnd << std::endl;
  //std::cout << "K V_body Accel_filt:\n" << K.block<3,3>(0,3) * eawp.innovations.accel_filt << std::endl;
  //std::cout << "K:\n" << K << std::endl;

}

// Getter Functions
struct NedCoor_f ekf_aw_get_speed_body(void)
{
  const struct NedCoor_f s = {
    .x = eawp.state.V_body(0),
    .y = eawp.state.V_body(1),
    .z = eawp.state.V_body(2)
  };
  return s;
}

struct NedCoor_f ekf_aw_get_wind_ned(void)
{
  const struct NedCoor_f w = {
    .x = eawp.state.wind(0),
    .y = eawp.state.wind(1),
    .z = eawp.state.wind(2)
  };
  return w;
}

struct NedCoor_f ekf_aw_get_offset(void) // TO DO: use right type instead of NEDCOORD_f
{
  const struct NedCoor_f w = {
    .x = eawp.state.offset(0),
    .y = eawp.state.offset(1),
    .z = eawp.state.offset(2)
  };
  return w;
}

struct ekfHealth ekf_aw_get_health(void)
{
  const struct ekfHealth w = {
    .healthy = eawp.health.healthy,
    .crashes_n = eawp.health.crashes_n
  };
  return w;
}

struct FloatVect3 ekf_aw_get_innov_V_gnd(void)
{
  const struct FloatVect3 w = {
    .x = eawp.innovations.V_gnd(0),
    .y = eawp.innovations.V_gnd(1),
    .z = eawp.innovations.V_gnd(2)
  };
  return w;
}
struct FloatVect3 ekf_aw_get_innov_accel_filt(void)
{
  const struct FloatVect3 w = {
    .x = eawp.innovations.accel_filt(0),
    .y = eawp.innovations.accel_filt(1),
    .z = eawp.innovations.accel_filt(2)
  };
  return w;
}

float ekf_aw_get_innov_V_pitot(void)
{
  const float w = eawp.innovations.V_pitot;
  return w;
}

void ekf_aw_get_meas_cov(float meas_cov[7])
{
  float diagonal[EKF_AW_R_SIZE];
  for(int8_t i=0; i<EKF_AW_R_SIZE; i++) {
    // Protect log10 against 0 and negative values
    if (eawp.R(i,i)<=0.0f){
      diagonal[i] = eawp.R(i,i);
    }
    else{
      diagonal[i] = log10(eawp.R(i,i));
    }
  }

  memcpy(meas_cov, diagonal, 7 * sizeof(float));
}

void ekf_aw_get_state_cov(float state_cov[9])
{
  float diagonal[EKF_AW_COV_SIZE];
  for(int8_t i=0; i<EKF_AW_COV_SIZE; i++) {
    // Protect log10 against 0 and negative values
    if (eawp.P(i,i)<=0.0f){
      diagonal[i] = eawp.P(i,i);
    }
    else{
      diagonal[i] = log10(eawp.P(i,i));
    }
  }

  memcpy(state_cov, diagonal, 9 * sizeof(float));
}

void ekf_aw_get_process_cov(float process_cov[12])
{
  float diagonal[EKF_AW_Q_SIZE];
  for(int8_t i=0; i<EKF_AW_Q_SIZE; i++) {
    // Protect log10 against 0 and negative values
    if (eawp.Q(i,i)<=0.0f){
      diagonal[i] = eawp.Q(i,i);
    }
    else{
      diagonal[i] = log10(eawp.Q(i,i));
    }
  }

  memcpy(process_cov, diagonal, 9 * sizeof(float));
}

void ekf_aw_get_fuselage_force(float force[3])
{
  float temp[3];
  for(int8_t i=0; i<3;i++){
    temp[i] = eawp.forces.fuselage(i);
  }
  memcpy(force, temp, 3 * sizeof(float));
}

void ekf_aw_get_wing_force(float force[3])
{
  float temp[3];
  for(int8_t i=0; i<3;i++){
    temp[i] = eawp.forces.wing(i);
  }
  memcpy(force, temp, 3 * sizeof(float));
}

void ekf_aw_get_elevator_force(float force[3])
{
  float temp[3];
  for(int8_t i=0; i<3;i++){
    temp[i] = eawp.forces.elevator(i);
  }
  memcpy(force, temp, 3 * sizeof(float));
}

void ekf_aw_get_hover_force(float force[3])
{
  float temp[3];
  for(int8_t i=0; i<3;i++){
    temp[i] = eawp.forces.hover(i);
  }
  memcpy(force, temp, 3 * sizeof(float));
}

void ekf_aw_get_pusher_force(float force[3])
{
  float temp[3];
  for(int8_t i=0; i<3;i++){
    temp[i] = eawp.forces.pusher(i);
  }
  memcpy(force, temp, 3 * sizeof(float));
}

struct ekfAwParameters *ekf_aw_get_param_handle(void){
  return &ekf_aw_params;
}


// Setter Functions
void ekf_aw_set_speed_body(struct NedCoor_f *s)
{
  eawp.state.V_body(0) = s->x;
  eawp.state.V_body(1) = s->y;
  eawp.state.V_body(2) = s->z;
}

void ekf_aw_set_wind(struct NedCoor_f *s)
{
  eawp.state.wind(0) = s->x;
  eawp.state.wind(1) = s->y;
  eawp.state.wind(2) = s->z;
}

void ekf_aw_set_offset(struct NedCoor_f *s)
{
  eawp.state.offset(0) = s->x;
  eawp.state.offset(1) = s->y;
  eawp.state.offset(2) = s->z;
}

void ekf_aw_reset_health(void)
{
  eawp.health.healthy = true;
  eawp.health.crashes_n = 0;
}

// Forces functions

// Fx Forces functions
float fx_fuselage(float *skew,float *aoa,float *u){

  float sign = *u < 0.0f ? -1.0f : *u > 0.0f ? 1.0f : 0.0f;
  // Fx = (k1*cos(skew)+k2+k3*alpha+k4*alpha^2)*V^2
  float Fx = (ekf_aw_params.k_fx_fuselage[0]*cosf(*skew)+
                       ekf_aw_params.k_fx_fuselage[1] + 
                       ekf_aw_params.k_fx_fuselage[2] * *aoa + 
                       ekf_aw_params.k_fx_fuselage[3] * *aoa * *aoa)* *u * *u * sign;

  return Fx;
};

float fx_elevator(float *elevator_angle, float *V_a){

  // Fx = (k1+k2*alpha+k3*alpha^2)*V^2
  float Fx = (ekf_aw_params.k_fx_elev[0] +
              ekf_aw_params.k_fx_elev[1] * *elevator_angle +
              ekf_aw_params.k_fx_elev[2] * *elevator_angle * *elevator_angle) * *V_a * *V_a;

  return Fx;
};

float fx_wing(float *skew,float *aoa,float *u){

  float sign = *u < 0.0f ? -1.0f : *u > 0.0f ? 1.0f : 0.0f;

  float Fx = 0.0f;

  // Verify aircraft is flying forwards
  if (*u>0.0f){
    // Fx2 = (k1*(1+k5*skew)+(k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2 //TO DO: UPDATE WITH SIMPLER MODEL
    Fx = sign* (*u * *u) *(ekf_aw_params.k_fx_wing[0] + 
                              ekf_aw_params.k_fx_wing[4] *sinf(*skew) +
                                  (ekf_aw_params.k_fx_wing[1] * *aoa +
                                    ekf_aw_params.k_fx_wing[2] * (*aoa * *aoa)) *(sinf(*skew)*sinf(*skew)+ekf_aw_params.k_fx_wing[3]));
  }
  else{
    Fx = sign* (*u * *u) *(ekf_aw_params.k_fx_wing[0]);
  }
  

  return Fx;
};

float fx_fy_hover(float *RPM_hover_mean, float *V){

  float sign = *V < 0.0f ? -1.0f : *V > 0.0f ? 1.0f : 0.0f;

  // Fx = K1*V^2+K2*RPM^2*sqrt(V)     + K3*V
  float Fx = ekf_aw_params.k_fx_hover[0] * (*V * *V) * sign +
             ekf_aw_params.k_fx_hover[1] *sqrtf(fabsf(*V)) * sign * (*RPM_hover_mean * *RPM_hover_mean);

        Fx += ekf_aw_params.k_fx_hover[2] * *V *sign;

  return Fx;
};

float fx_pusher(float *RPM_pusher, float *u){
  float Fx = 0.0f;

  // Only generate a force if the pusher is turning
  if (*RPM_pusher>500){
    // Take care of case where drone is flying backwards with pusher (quite rare)
      if (*u>0.0f){
        // Fx_pusher = k1*RPM^2+k2*RPM*V+k3*V
        Fx = ekf_aw_params.k_fx_push[0] * (*RPM_pusher * *RPM_pusher) +
              ekf_aw_params.k_fx_push[1] * *RPM_pusher * *u +
              ekf_aw_params.k_fx_push[2] * *u;

      }
      else{
        Fx = ekf_aw_params.k_fx_push[0] * (*RPM_pusher * *RPM_pusher);
      };
  }
  return Fx;
}

// Fy Forces functions
float fy_wing(float *skew,float *aoa __attribute__((unused)),float *u){

  float sign = *u < 0.0f ? -1.0f : *u > 0.0f ? 1.0f : 0.0f;

  // Fy = K*cos(skew)*sin(skew)*u^2
  //TO DO: UPDATE MODEL
  float Fy = (ekf_aw_params.k_fy_wing[0]*cosf(*skew)*sinf(*skew))* *u * *u *sign;

  return Fy;
};

// Fz Forces functions
float fz_fuselage(float *skew,float *aoa,float *V_a){

  // Fz = (k1*cos(skew)+k2+k3*alpha+k4*alpha^2)*V^2
  float Fz = (ekf_aw_params.k_fz_fuselage[0]*cosf(*skew)+
                       ekf_aw_params.k_fz_fuselage[1] + 
                       ekf_aw_params.k_fz_fuselage[2] * *aoa + 
                       ekf_aw_params.k_fz_fuselage[3] * *aoa * *aoa)* *V_a * *V_a;

  return Fz;
};

float fz_elevator(float *elevator_angle, float *V_a){

  // Fz = (k1+k2*alpha+k3*alpha^2)*V^2  
  float Fz = (ekf_aw_params.k_fz_elev[0] +
              ekf_aw_params.k_fz_elev[1] * *elevator_angle) * *V_a * *V_a;

  return Fz;
};

float fz_wing(float *skew,float *aoa,float *u){

  float sign = *u < 0.0f ? -1.0f : *u > 0.0f ? 1.0f : 0.0f;

  //Fz1 = ((k1+k2*alpha+k3*alpha^2)*(sin(skew)^2+k4)*V^2 //TO DO: UPDATE WITH SIMPLER MODEL
  float Fz = 0.0f;
  if (*u>0.0f){
    float Fz = ((ekf_aw_params.k_fz_wing[0] + 
                ekf_aw_params.k_fz_wing[1] * *aoa +
                ekf_aw_params.k_fz_wing[2] * (*aoa * *aoa)) * (sinf(*skew) * sinf(*skew) + ekf_aw_params.k_fz_wing[3])) * (*u * *u) *sign;
  }
  else{
    Fz = 0.0f;
  }
  

  return Fz;
};

float fz_hover(matrix::Vector<float,4> RPM_hover, float *V_a){

  // Fz = RPM_1^2*K1 + RPM_2^2*K2 + RPM_3^2*K3 + RPM_4^2*K4 + K5*V
  float Fz = RPM_hover(0) * RPM_hover(0) * ekf_aw_params.k_fz_hover[0] +
             RPM_hover(1) * RPM_hover(1) * ekf_aw_params.k_fz_hover[1] +
             RPM_hover(2) * RPM_hover(2) * ekf_aw_params.k_fz_hover[2] +
             RPM_hover(3) * RPM_hover(3) * ekf_aw_params.k_fz_hover[3] ;
        
        Fz += ekf_aw_params.k_fz_hover[4] * *V_a;

  return Fz;

};
