#ifndef IMU_V3_H
#define IMU_V3_H

#include <inttypes.h>

#include "frames.h"

/* accelerometers in arbitrary unit */
extern float imu_accel[AXIS_NB];
/* gyros in rad/s                   */
extern float   imu_gyro[AXIS_NB];
/* magnetometer in arbitrary unit   */
extern int16_t imu_mag[AXIS_NB];
/* battery in volts                 */
extern float imu_bat;

extern void imu_init(void);


extern struct adc_buf buf_ax;
extern struct adc_buf buf_ay;
extern struct adc_buf buf_az;
extern struct adc_buf buf_bat;

#define IMU_ACCEL_X_NEUTRAL 538
#define IMU_ACCEL_Y_NEUTRAL 506
#define IMU_ACCEL_Z_NEUTRAL 506

#if 0
#define ImuUpdateAccels() {						\
    int_disable();							\
    imu_accel[AXIS_X]= buf_ax.sum;					\
    imu_accel[AXIS_Y]= buf_ay.sum;					\
    imu_accel[AXIS_Z]= buf_az.sum;					\
    int_enable();							\
    imu_accel[AXIS_X] = -((imu_accel[AXIS_X] / DEFAULT_AV_NB_SAMPLE) - IMU_ACCEL_X_NEUTRAL); \
    imu_accel[AXIS_Y] = (imu_accel[AXIS_Y] / DEFAULT_AV_NB_SAMPLE) - IMU_ACCEL_Y_NEUTRAL; \
    imu_accel[AXIS_Z] = (imu_accel[AXIS_Z] / DEFAULT_AV_NB_SAMPLE) - IMU_ACCEL_Z_NEUTRAL; \
  }
#else
#define ImuUpdateAccels() {             					   \
    imu_accel[AXIS_X]= -((buf_ax.sum / DEFAULT_AV_NB_SAMPLE) - IMU_ACCEL_X_NEUTRAL); \
    imu_accel[AXIS_Y]= (buf_ay.sum / DEFAULT_AV_NB_SAMPLE) - IMU_ACCEL_Y_NEUTRAL;  \
    imu_accel[AXIS_Z]= (buf_az.sum / DEFAULT_AV_NB_SAMPLE) - IMU_ACCEL_Z_NEUTRAL;  \
}
#endif


#define IMU_GYRO_X_NEUTRAL 40867
#define IMU_GYRO_Y_NEUTRAL 40852
#define IMU_GYRO_Z_NEUTRAL 38915

#define IMU_GYRO_X_GAIN -0.000220177991888642784
#define IMU_GYRO_Y_GAIN -0.000214915108532129801
#define IMU_GYRO_Z_GAIN  0.0002104

#define ImuUpdateGyros() {					                \
    									        \
    imu_gyro[AXIS_X] = (float)((int32_t)max1167_values[0] - IMU_GYRO_X_NEUTRAL) \
                        * IMU_GYRO_X_GAIN;				        \
    imu_gyro[AXIS_Y] = (float)((int32_t)max1167_values[1] - IMU_GYRO_Y_NEUTRAL) \
                        * IMU_GYRO_Y_GAIN;				        \
    imu_gyro[AXIS_Z] = (float)((int32_t)max1167_values[2] - IMU_GYRO_Z_NEUTRAL) \
                        * IMU_GYRO_Z_GAIN;				        \
  }

#define ImuUpdateMag() {		   \
    imu_mag[AXIS_X] = -micromag_values[0]; \
    imu_mag[AXIS_Y] = -micromag_values[1]; \
    imu_mag[AXIS_Z] = micromag_values[2];  \
  }


#define IMU_BAT_NEUTRAL 2
#define IMU_BAT_GAIN .010101

#define ImuUpdateBat() {					       \
    imu_bat = ((buf_bat.sum / DEFAULT_AV_NB_SAMPLE) - IMU_BAT_NEUTRAL) \
      * IMU_BAT_GAIN;						       \
  }

#endif /* IMU_V3_H */
