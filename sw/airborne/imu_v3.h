#ifndef IMU_V3_H
#define IMU_V3_H

#include "std.h"
#include "6dof.h"

#include "imu_v3_hw.h"

/* calibrated sensor readings */
extern float   imu_accel[AXIS_NB]; /* accelerometers in arbitrary unit */
extern float   imu_gyro[AXIS_NB];  /* gyros in rad/s                   */
extern int16_t imu_mag[AXIS_NB];   /* magnetometer in arbitrary unit   */
extern float   imu_bat;            /* battery in volts                 */

extern float imu_gyro_prev[AXIS_NB];  /* previous gyros in rad/s       */
#define IMU_GYRO_LP_ALPHA 0.6
extern float imu_gyro_lp[AXIS_NB];    /* low passed calibrated gyros in rad/s */

/* raw sensors readings */
extern uint16_t imu_accel_raw[AXIS_NB];
extern uint16_t imu_gyro_raw[AXIS_NB];
extern int16_t  imu_mag_raw[AXIS_NB];

extern void imu_v3_init(void);
extern void imu_v3_detect_vehicle_still(void);
extern bool_t imu_vehicle_still;
extern float imu_vs_gyro_initial_bias[AXIS_NB];
extern float imu_vs_gyro_unbiased[AXIS_NB]; /* unbiased gyros in rad/s */

/* sliding average for still vehicle detection */
extern uint32_t imu_vs_accel_raw_avg[AXIS_NB];
extern float    imu_vs_accel_raw_var[AXIS_NB];
extern uint32_t imu_vs_gyro_raw_avg[AXIS_NB];
extern float    imu_vs_gyro_raw_var[AXIS_NB];
extern int32_t  imu_vs_mag_raw_avg[AXIS_NB];
extern float    imu_vs_mag_raw_var[AXIS_NB];

extern struct adc_buf buf_ax;
extern struct adc_buf buf_ay;
extern struct adc_buf buf_az;
extern struct adc_buf buf_bat;

#define IMU_ACCEL_X_NEUTRAL (538 * 32)
#define IMU_ACCEL_Y_NEUTRAL (506 * 32)
#define IMU_ACCEL_Z_NEUTRAL (506 * 32)

#define IMU_ACCEL_X_GAIN -1.
#define IMU_ACCEL_Y_GAIN  1.
#define IMU_ACCEL_Z_GAIN  1.

#define ImuUpdateAccels() {						      \
    imu_accel_raw[AXIS_X]= buf_ax.sum;					      \
    imu_accel_raw[AXIS_Y]= buf_ay.sum;					      \
    imu_accel_raw[AXIS_Z]= buf_az.sum;					      \
    imu_accel[AXIS_X]= IMU_ACCEL_X_GAIN * (buf_ax.sum - IMU_ACCEL_X_NEUTRAL); \
    imu_accel[AXIS_Y]= IMU_ACCEL_Y_GAIN * (buf_ay.sum - IMU_ACCEL_Y_NEUTRAL); \
    imu_accel[AXIS_Z]= IMU_ACCEL_Y_GAIN * (buf_az.sum - IMU_ACCEL_Z_NEUTRAL); \
}

#define IMU_GYRO_P_NEUTRAL 40885
#define IMU_GYRO_Q_NEUTRAL 40910
#define IMU_GYRO_R_NEUTRAL 39552

#define IMU_GYRO_P_GAIN -0.0002202
#define IMU_GYRO_Q_GAIN -0.0002150
#define IMU_GYRO_R_GAIN  0.0002104

#define ImuUpdateGyros() {						                \
    imu_gyro_prev[AXIS_P] = imu_gyro[AXIS_P];				                \
    imu_gyro_prev[AXIS_Q] = imu_gyro[AXIS_Q];				                \
    imu_gyro_prev[AXIS_R] = imu_gyro[AXIS_R];				                \
    imu_gyro_raw[AXIS_P] = max1167_values[0];				                \
    imu_gyro_raw[AXIS_Q] = max1167_values[1];				                \
    imu_gyro_raw[AXIS_R] = max1167_values[2];				                \
    imu_gyro[AXIS_P] = (float)((int32_t)max1167_values[0] - IMU_GYRO_P_NEUTRAL)         \
                        * IMU_GYRO_P_GAIN;				                \
    imu_gyro[AXIS_Q] = (float)((int32_t)max1167_values[1] - IMU_GYRO_Q_NEUTRAL)         \
                        * IMU_GYRO_Q_GAIN;				                \
    imu_gyro[AXIS_R] = (float)((int32_t)max1167_values[2] - IMU_GYRO_R_NEUTRAL)         \
                        * IMU_GYRO_R_GAIN;						\
    imu_vs_gyro_unbiased[AXIS_P] = imu_gyro[AXIS_P] - imu_vs_gyro_initial_bias[AXIS_P]; \
    imu_vs_gyro_unbiased[AXIS_Q] = imu_gyro[AXIS_Q] - imu_vs_gyro_initial_bias[AXIS_Q]; \
    imu_vs_gyro_unbiased[AXIS_R] = imu_gyro[AXIS_R] - imu_vs_gyro_initial_bias[AXIS_R]; \
									                \
    imu_gyro_lp[AXIS_P] = IMU_GYRO_LP_ALPHA * imu_vs_gyro_unbiased[AXIS_P] +            \
                          ( 1. - IMU_GYRO_LP_ALPHA) * imu_gyro_lp[AXIS_P];              \
    imu_gyro_lp[AXIS_Q] = IMU_GYRO_LP_ALPHA * imu_vs_gyro_unbiased[AXIS_Q] +            \
                          ( 1. - IMU_GYRO_LP_ALPHA) * imu_gyro_lp[AXIS_Q];              \
    imu_gyro_lp[AXIS_R] = IMU_GYRO_LP_ALPHA * imu_vs_gyro_unbiased[AXIS_R] +	        \
                          ( 1. - IMU_GYRO_LP_ALPHA) * imu_gyro_lp[AXIS_R];              \
  }

#define ImuUpdateMag() {						\
    imu_mag_raw[AXIS_X] = micromag_values[0];				\
    imu_mag_raw[AXIS_Y] = micromag_values[1];				\
    imu_mag_raw[AXIS_Z] = micromag_values[2];				\
    imu_mag[AXIS_X] = -micromag_values[0];				\
    imu_mag[AXIS_Y] = -micromag_values[1];				\
    imu_mag[AXIS_Z] = micromag_values[2];				\
  }


#define IMU_BAT_NEUTRAL 2
#define IMU_BAT_GAIN .010101

#define ImuUpdateBat() {					       \
    imu_bat = ((buf_bat.sum / DEFAULT_AV_NB_SAMPLE) - IMU_BAT_NEUTRAL) \
      * IMU_BAT_GAIN;						       \
  }



#define ImuUpdateFromAvg() {						\
    imu_accel[AXIS_X] =	IMU_ACCEL_X_GAIN *((int32_t)imu_vs_accel_raw_avg[AXIS_X] - IMU_ACCEL_X_NEUTRAL); \
    imu_accel[AXIS_Y] =	IMU_ACCEL_Y_GAIN *((int32_t)imu_vs_accel_raw_avg[AXIS_Y] - IMU_ACCEL_Y_NEUTRAL); \
    imu_accel[AXIS_Z] =	IMU_ACCEL_Z_GAIN *((int32_t)imu_vs_accel_raw_avg[AXIS_Z] - IMU_ACCEL_Z_NEUTRAL); \
    imu_gyro[AXIS_P] = (float)((int32_t)imu_vs_gyro_raw_avg[AXIS_P] - IMU_GYRO_P_NEUTRAL)       \
                        * IMU_GYRO_P_GAIN;						        \
    imu_gyro[AXIS_Q] = (float)((int32_t)imu_vs_gyro_raw_avg[AXIS_Q] - IMU_GYRO_Q_NEUTRAL)       \
                        * IMU_GYRO_Q_GAIN;				                        \
    imu_gyro[AXIS_R] = (float)((int32_t)imu_vs_gyro_raw_avg[AXIS_R] - IMU_GYRO_R_NEUTRAL)       \
                        * IMU_GYRO_R_GAIN;				                        \
    imu_gyro_prev[AXIS_P] = imu_gyro[AXIS_P];				\
    imu_gyro_prev[AXIS_Q] = imu_gyro[AXIS_Q];				\
    imu_gyro_prev[AXIS_R] = imu_gyro[AXIS_R];				\
    imu_vs_gyro_initial_bias[AXIS_P] = imu_gyro[AXIS_P];		\
    imu_vs_gyro_initial_bias[AXIS_Q] = imu_gyro[AXIS_Q];		\
    imu_vs_gyro_initial_bias[AXIS_R] = imu_gyro[AXIS_R];		\
    imu_vs_gyro_unbiased[AXIS_P] = 0.;		                        \
    imu_vs_gyro_unbiased[AXIS_Q] = 0.;					\
    imu_vs_gyro_unbiased[AXIS_R] = 0.;					\
}




#endif /* IMU_V3_H */
