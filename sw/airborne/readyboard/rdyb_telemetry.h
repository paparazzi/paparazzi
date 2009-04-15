#ifndef RDYB_TELEMETRY_H
#define RDYB_TELEMETRY_H

#include <stdio.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#define AC_ID 42

extern void telemetry_init(void);


#define TELEMETRY_SEND_ACCEL_RAW(_acc) {				\
    IvySendMsg("%d IMU_ACCEL_RAW %d %d %d\n", AC_ID, (_acc).x, (_acc).y, (_acc).z); \
  }

#define TELEMETRY_SEND_GYRO_RAW(_gyr) {					\
    IvySendMsg("%d IMU_GYRO_RAW %d %d %d\n", AC_ID, _gyr.x, _gyr.y, _gyr.z); \
  }

#define TELEMETRY_SEND_MAG_RAW(_mag) {					\
    IvySendMsg("%d IMU_MAG_RAW %d %d %d\n",AC_ID,  _mag.x, _mag.y, _mag.z); \
  }

#define TELEMETRY_SEND_ACCEL(_acc) {				\
    IvySendMsg("%d IMU_ACCEL %f %f %f\n", AC_ID,_acc.x, _acc.y, _acc.z); \
  }

#define TELEMETRY_SEND_GYRO(_gyro) {				\
    IvySendMsg("%d IMU_GYRO %f %f %f\n", AC_ID,_gyro.x, _gyro.y, _gyro.z); \
  }

#define TELEMETRY_SEND_MAG(_mag) {				\
    IvySendMsg("%d IMU_MAG %f %f %f\n", AC_ID,_mag.x, _mag.y, _mag.z); \
  }

#define TELEMETRY_SEND_AHRS_EULER(_eu) {				\
    IvySendMsg("%d AHRS_EULER %f %f %f\n", AC_ID,_eu.phi, _eu.theta, _eu.psi); \
  }

#define TELEMETRY_SEND_AHRS_MEASUREMENT_EULER(_eu) {				\
    IvySendMsg("%d AHRS_MEASUREMENT_EULER %f %f %f\n", AC_ID,_eu.phi, _eu.theta, _eu.psi); \
  }

#endif /* RDYB_TELEMETRY_H */
