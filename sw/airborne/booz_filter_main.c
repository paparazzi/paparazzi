#include "booz_filter_main.h"

#include "std.h"
#include "init_hw.h"
#include "interrupt_hw.h"
#include "sys_time.h"

#include "adc.h"
#include "imu_v3.h"

#include "i2c.h"
#include "AMI601.h"

#include "booz_ahrs.h"

#include "uart.h"
#include "messages.h"
#include "downlink.h"
#include "booz_filter_telemetry.h"

#include "booz_link_mcu.h"

static inline void on_imu_event( void );


#ifndef SITL
int main( void ) {
  booz_filter_main_init();
  while (1) {
    if (sys_time_periodic())
      booz_filter_main_periodic_task();
    booz_filter_main_event_task();
  }
  return 0;
}
#endif

STATIC_INLINE void booz_filter_main_init( void ) {

  hw_init();
  sys_time_init();
  //FIXME
#ifndef SITL
  uart1_init_tx();
  adc_init();

  i2c_init();
  ami601_init();
#endif
  imu_v3_init();

  booz_ahrs_init();

  booz_link_mcu_init();

  int_enable();
}

// static uint32_t t0, t1, diff;

STATIC_INLINE void booz_filter_main_event_task( void ) {
  /* check if measurements are available */
  ImuEventCheckAndHandle(on_imu_event);

}

STATIC_INLINE void booz_filter_main_periodic_task( void ) {
  /* triger measurements */
  //  ami601_periodic();  
  //  DOWNLINK_SEND_IMU_MAG_RAW(&ami601_val[0], &ami601_val[4], &ami601_val[2]);
  ImuPeriodic();
  static uint8_t _62hz = 0;
  _62hz++;
  if (_62hz > 4) _62hz = 0;
  switch (_62hz) {
  case 0:
    booz_filter_telemetry_periodic_task();
    break;
  }

}

static inline void on_imu_event( void ) {
  switch (booz_ahrs_status) {

  case BOOZ_AHRS_STATUS_UNINIT :
    imu_v3_detect_vehicle_still();
    if (imu_vehicle_still) {
      ImuUpdateFromAvg();
      booz_ahrs_start(imu_accel, imu_gyro, imu_mag);
    }
    break;

  case BOOZ_AHRS_STATUS_RUNNING :
    // t0 = T0TC;
    booz_ahrs_run(imu_accel, imu_gyro_prev, imu_mag);
    // t1 = T0TC;
    // diff = t1 - t0;
    // DOWNLINK_SEND_TIME(&diff);
    break;
  }
  
  booz_link_mcu_send();
}
