#include "booz_filter_main.h"

#include "std.h"
#include "init_hw.h"
#include "interrupt_hw.h"
#include "sys_time.h"

#include "adc.h"
#include "imu_v3.h"

//#include "multitilt.h"
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
#endif
  imu_v3_init();

  multitilt_init();

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
  ImuPeriodic();
  static uint8_t _50hz = 0;
  _50hz++;
  if (_50hz > 5) _50hz = 0;
  switch (_50hz) {
  case 0:
    booz_filter_telemetry_periodic_task();
    break;
  }

}

static inline void on_imu_event( void ) {
  switch (mtt_status) {

  case MT_STATUS_UNINIT :
    imu_v3_detect_vehicle_still();
    if (imu_vehicle_still) {
      ImuUpdateFromAvg();
      multitilt_start(imu_accel, imu_gyro, imu_mag);
    }
    break;

  case MT_STATUS_RUNNING :
    // t0 = T0TC;
    multitilt_predict(imu_gyro_prev);
    multitilt_update(imu_accel, imu_mag);
    // t1 = T0TC;
    // diff = t1 - t0;
    // DOWNLINK_SEND_TIME(&diff);
    break;
  }
  
  booz_link_mcu_send();
}
