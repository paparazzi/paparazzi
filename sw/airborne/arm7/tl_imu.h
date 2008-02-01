#ifndef TL_IMU_H
#define TL_IMU_H

#define TL_IMU_USE_MICROMAG
#define TL_IMU_USE_BARO


#include "std.h"
#include "adc.h"
#include "spi.h"
#include "micromag.h"
#include "tl_baro.h"

/* Gyro */
#define GR_GAIN    -3.57e-4
#define GR_NEUTRAL 24894

/* Range Meter (http://www.maxbotix.com) VCC/512 per inch */
#define INCH 0.0254
#define RM_GAIN (3.3/5.5*512/1024 * INCH / DEFAULT_AV_NB_SAMPLE)
#define RM_ZERO 0

/* Z Accelerometer 4G, 300mV/g */
#define G 9.81
#define ACCEL_GAIN -(G * 3300/1024/300 / DEFAULT_AV_NB_SAMPLE)
#define ACCEL_NEUTRAL (DEFAULT_AV_NB_SAMPLE*512)

#define HX_GAIN    -1.
#define HX_NEUTRAL  0.
#define HX_CHAN     1
#define HY_GAIN    -1.
#define HY_NEUTRAL  0
#define HY_CHAN     0
#define HZ_GAIN    -1.
#define HZ_NEUTRAL  0
#define HZ_CHAN     2


#define TL_IMU_IDLE             0
#define TL_IMU_SENDING_BARO_REQ 1
#define TL_IMU_READING_MICROMAG 2
#define TL_IMU_READING_BARO     3
#define TL_IMU_DATA_AVAILABLE   4


extern void tl_imu_init(void);
extern void tl_imu_periodic(void);

extern volatile uint8_t tl_imu_status;
extern float   tl_imu_r;
extern float   tl_imu_rm; /* m */
extern float   tl_imu_accel; /* m/s^2 */
extern float   tl_imu_hx;
extern float   tl_imu_hy;
extern float   tl_imu_hz;
extern float   tl_imu_pressure;

extern struct adc_buf buf_gr; 


#define TlImuEventCheckAndHandle(user_callback) {			\
    if (tl_imu_status == TL_IMU_DATA_AVAILABLE) {			\
      tl_imu_hx = HX_GAIN * (float)( micromag_values[HX_CHAN] - HX_NEUTRAL); \
      tl_imu_hy = HY_GAIN * (float)( micromag_values[HY_CHAN] - HY_NEUTRAL); \
      tl_imu_hz = HZ_GAIN * (float)( micromag_values[HZ_CHAN] - HZ_NEUTRAL); \
      tl_baro_compute(); /* faire un truc avec les données baro */	\
      tl_imu_pressure = tl_baro_pressure;				\
      user_callback();							\
      micromag_status = MM_IDLE;					\
      tl_imu_status = TL_IMU_IDLE;					\
    }									\
  }

#endif /* TL_IMU_H */
