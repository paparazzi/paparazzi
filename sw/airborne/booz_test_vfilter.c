#include "std.h"
#include "init_hw.h"
#include "interrupt_hw.h"
#include "sys_time.h"

#include "uart.h"
#include "messages.h"
#include "downlink.h"

#include "booz_imu.h"
#include "tl_vfilter.h"

static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);

uint32_t t0, t1, diff;

int main( void ) {
  main_init();
  while (1) {
    if (sys_time_periodic())
      main_periodic();
    main_event();
  }
  return 0;
}


static inline void main_init(void) {
  hw_init();

  sys_time_init();

  uart1_init_tx();

  booz_imu_init();

  tl_vf_init(0., 0., 0.);

  int_enable();
}


static inline void main_periodic(void) {
  booz_imu_periodic();
  RunOnceEvery(250, DOWNLINK_SEND_BOOT(&cpu_time_sec)); 
}


static inline void on_imu_gyro_available(void) {
  RunOnceEvery(10, {							\
      DOWNLINK_SEND_IMU_ACCEL(&imu_accel[AXIS_X], &imu_accel[AXIS_Y], &imu_accel[AXIS_Z]); \
      DOWNLINK_SEND_TL_KALM_V_COV(&tl_vf_P[0][0], &tl_vf_P[1][1], &tl_vf_P[2][2]); \
    });
  //  t0 = T0TC;
  tl_vf_predict(imu_accel[AXIS_Z]);
  //  t1 = T0TC;
  //  diff = t1 - t0;
  //  DOWNLINK_SEND_TIME(&diff);
  DOWNLINK_SEND_TL_KALM_V_STATE(&tl_vf_z, &tl_vf_zdot, &tl_vf_bias, &tl_vf_z_meas);
}

static inline void on_imu_mag_available(void) {
}

static inline void on_imu_baro_available(void) {
  float pressure = 0.25 * imu_pressure_raw;
  static float ground_pressure = 0.;
  if (ground_pressure == 0) ground_pressure = pressure;
  float z_baro = (ground_pressure - pressure)*0.084;
  DOWNLINK_SEND_TL_IMU_PRESSURE(&pressure);
  DOWNLINK_SEND_TL_IMU_RANGEMETER(&z_baro);
  //  t0 = T0TC; 
  tl_vf_update(-z_baro);
  //  t1 = T0TC;
  //  diff = t1 - t0;
  //  DOWNLINK_SEND_TIME(&diff);
  //  t0 = t1;
}

static inline void main_event(void) {
  BoozImuEvent(on_imu_gyro_available, on_imu_mag_available, on_imu_baro_available);
}

