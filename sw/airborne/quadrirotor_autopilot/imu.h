#ifndef IMU_H
#define IMU_H

struct ImuSample {
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float accel_x;
  float accel_y;
  float accel_z;
};

extern struct ImuSample imu_sample;

//void imu_init( void );

void imu_update( void );

#endif /* IMU_H */
