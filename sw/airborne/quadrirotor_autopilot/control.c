#include <inttypes.h>


#include "control.h"
#include "kalman.h"
#include "imu.h"
#include "autopilot.h"

float control_desired_phi;
float control_desired_phi_dot;
float control_desired_theta;
float control_desired_theta_dot;

int16_t control_command_roll;
int16_t control_command_pitch;

#define phi_dot_p_gain   -2500.
#define theta_dot_p_gain  2500.

void control_run_rotational_speed_loop() {
  float err = control_desired_phi_dot - kalman_public.phi_dot;
  control_command_roll =  TRIM_PPRZ(phi_dot_p_gain * err);
  err = control_desired_theta_dot - kalman_public.theta_dot;
  control_command_pitch =  TRIM_PPRZ(theta_dot_p_gain * err);
}

void control_run_raw_rotational_speed_loop() {
  float err = control_desired_phi_dot - imu_sample.gyro_x;
  control_command_roll =  TRIM_PPRZ(phi_dot_p_gain * err);
  err = control_desired_theta_dot - imu_sample.gyro_y;
  control_command_pitch =  TRIM_PPRZ(theta_dot_p_gain * err);
}


#define TRIM(val, limit) ( val < -limit ? -limit : val > limit ? limit : val )

#define MAX_PHI_DOT  2.
#define phi_p_gain 3.
#define MAX_THETA_DOT  2.
#define theta_p_gain 3.

void control_run_attitude_loop() {
  float err = control_desired_phi - kalman_public.phi;
  control_desired_phi_dot = TRIM(phi_p_gain * err, MAX_PHI_DOT);
  err = control_desired_theta - kalman_public.theta;
  control_desired_theta_dot = TRIM(theta_p_gain * err, MAX_THETA_DOT);
}
