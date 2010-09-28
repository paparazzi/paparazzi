

#include <iostream>
#include <iomanip>

#include <Eigen/Core>

#include "ins_qkf.hpp"

//#include "std.h"
#define RadOfDeg(x) ((x) * (M_PI/180.))

// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN

int main(int, char *[]) {

  std::cout << "test libeknav 1" << std::endl;

  /* initial state */
  Vector3d pos_0_ecef(1017.67e3, -5079.282e3, 3709.041e3);
  Vector3d speed_0_ecef(0., 0., 0.);
  Quaterniond orientation(1., 0., 0., 0.);
  Vector3d bias_0(0., 0., 0.);

  /* initial covariance */
  const double pos_cov_0 =  1e2*1e2;
  const double speed_cov_0 =  3.*3.;
  const double orientation_cov_0 =  RadOfDeg(5.)*RadOfDeg(5.);
  const double bias_cov_0 =  0.447;
  
  /* system noise      */
  const Vector3d gyro_white_noise = Vector3d::Ones()*0.1*0.1;
  const Vector3d gyro_stability_noise = Vector3d::Ones()*0.00001;
  const Vector3d accel_white_noise = Vector3d::Ones()* 0.04*0.04;

  /* measurement noise */
  const double mag_noise = std::pow(5 / 180.0 * M_PI, 2);
  const Vector3d gps_pos_noise = Vector3d::Ones()  *10*10;
  const Vector3d gps_speed_noise = Vector3d::Ones()*0.1*0.1;

  /* sensors */
  Vector3d gyro(0., 0., 0.);
  Vector3d accelerometer(0., 0., 9.81);
  Vector3d magnetometer = Vector3d::UnitZ();

  basic_ins_qkf ins(pos_0_ecef, pos_cov_0, bias_cov_0, speed_cov_0,
		    gyro_white_noise, gyro_stability_noise, accel_white_noise,orientation);

  const double dt = 1./512.; /* predict at 512Hz */
  for (int i=1; i<5000; i++) {
    ins.predict(gyro, accelerometer, dt);
    if (i % 10 == 0)  /* update mag at 50Hz */
      ins.obs_vector(magnetometer, magnetometer, mag_noise);
    if (i % 128 == 0) /* update gps at 4 Hz */
      ins.obs_gps_pv_report(pos_0_ecef, speed_0_ecef, gps_pos_noise, gps_speed_noise);
  }

  return 0;

}

