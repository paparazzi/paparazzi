
#include "std.h"
#include "sys_time.h"
#include "init_hw.h"
#include "interrupt_hw.h"

#include "uart.h"
#include "print.h"

#include "adc.h"
#include "imu_v3.h"
#include "multitilt.h"

#include "messages.h"
#include "downlink.h"

#include "link_imu.h"

//#define SEND_ACCEL 1
//#define SEND_GYRO  1
//#define SEND_ACCEL_RAW 1
//#define SEND_GYRO_RAW  1
#define SEND_ACCEL_RAW_AVG 1
#define SEND_GYRO_RAW_AVG  1
#define SEND_AHRS_STATE 1
#define SEND_AHRS_COV   1

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void);

static inline void ahrs_task( void );


int main( void ) {
  main_init();
  while (1) {
    if (sys_time_periodic())
      main_periodic_task();
    main_event_task();
  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
  uart1_init_tx();
  adc_init();
  imu_v3_init();

  multitilt_init();
  link_imu_init();
  int_enable();
}

static uint32_t t0, t1;

static inline void main_event_task( void ) {

  if (max1167_data_available) {
    max1167_data_available = FALSE;
    spi_cur_slave = SPI_SLAVE_NONE;
    ImuUpdateGyros();
#ifdef SEND_GYRO
    DOWNLINK_SEND_IMU_GYRO(&imu_gyro[AXIS_X], &imu_gyro[AXIS_Y], &imu_gyro[AXIS_Z]);
#endif
#ifdef SEND_GYRO_RAW
    DOWNLINK_SEND_IMU_GYRO_RAW(&imu_gyro_raw[AXIS_X], &imu_gyro_raw[AXIS_Y], &imu_gyro_raw[AXIS_Z]);
#endif
    ImuUpdateAccels();
#ifdef SEND_ACCEL
    DOWNLINK_SEND_IMU_ACCEL(&imu_accel[AXIS_X], &imu_accel[AXIS_Y], &imu_accel[AXIS_Z]);
#endif
#ifdef SEND_ACCEL_RAW
    DOWNLINK_SEND_IMU_ACCEL_RAW(&imu_accel_raw[AXIS_X], &imu_accel_raw[AXIS_Y], &imu_accel_raw[AXIS_Z]);
#endif

    ahrs_task(); 
    link_imu_send();
  }
}

static inline void main_periodic_task( void ) {
  if (spi_cur_slave != SPI_SLAVE_NONE) {
    DOWNLINK_SEND_AHRS_OVERRUN();
  }
  else {
    spi_cur_slave = SPI_SLAVE_MAX;
    max1167_read();
  }
}

static inline void ahrs_task( void ) {
  switch (mtt_status) {

  case MT_STATUS_UNINIT : {
    imu_v3_detect_vehicle_still();
#ifdef SEND_GYRO_RAW_AVG
    DOWNLINK_SEND_IMU_GYRO_RAW_AVG(&imu_vs_gyro_raw_avg[AXIS_X], &imu_vs_gyro_raw_avg[AXIS_Y], &imu_vs_gyro_raw_avg[AXIS_Z], \
				   &imu_vs_gyro_raw_var[AXIS_X], &imu_vs_gyro_raw_var[AXIS_Y], &imu_vs_gyro_raw_var[AXIS_Z]);
#endif
#ifdef SEND_ACCEL_RAW_AVG
    DOWNLINK_SEND_IMU_ACCEL_RAW_AVG(&imu_vs_accel_raw_avg[AXIS_X], &imu_vs_accel_raw_avg[AXIS_Y], &imu_vs_accel_raw_avg[AXIS_Z], \
				    &imu_vs_accel_raw_var[AXIS_X], &imu_vs_accel_raw_var[AXIS_Y], &imu_vs_accel_raw_var[AXIS_Z]);
#endif
    if (imu_vehicle_still) {
      ImuUpdateFromAvg();
      multitilt_start(imu_accel, imu_gyro);
    }
  }
    break;

  case MT_STATUS_RUNNING : {
    //    t0 = T0TC;
    multitilt_predict(imu_gyro_prev);
    multitilt_update(imu_accel);
    //    t1 = T0TC;
    //    uint32_t dif = t1 - t0;
    static uint32_t foo_cnt = 0;
    foo_cnt++;
    if (!(foo_cnt % 10)) {
      //      DOWNLINK_SEND_TIME(&dif);
#ifdef SEND_AHRS_STATE
    const float foo = 0.;
    DOWNLINK_SEND_AHRS_STATE(&mtt_phi, &mtt_theta, &foo, &foo, &mtt_bp, &mtt_bq, &mtt_br);
#endif
#ifdef SEND_AHRS_COV
    DOWNLINK_SEND_AHRS_COV(&mtt_P_phi[0][0], &mtt_P_phi[0][1], &mtt_P_phi[1][0], &mtt_P_phi[1][1], &mtt_P_theta[0][0], &mtt_P_theta[0][1], &mtt_P_theta[1][1]);
#endif
    }
  }
    break;
  }

}
