#ifndef KALMAN_H
#define KALMAN_H

void kalman_init( void );
void kalman_state_update( float gyro_phi_measure, float gyro_theta_measure );
void kalman_kalman_update( float ax_measure, float ay_measure, float az_measure );

struct KalmanPublic {
  float phi;
  float phi_dot;
  float gyro_phi_bias;
  float theta;
  float theta_dot;
  float gyro_theta_bias;
};

extern struct KalmanPublic kalman_public;

#endif /* KALMAN_H */
