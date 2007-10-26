
#include "std.h"
#include "sys_time.h"
#include "init_hw.h"
#include "interrupt_hw.h"

#include "adc.h"
#include "imu_v3.h"
#include "multitilt.h"

#include "uart.h"
#include "messages.h"
#include "downlink.h"
#include "booz_imu_telemetry.h"

#include "link_imu.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void);

static inline void on_imu_event( void );


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

// static uint32_t t0, t1, diff;

static inline void main_event_task( void ) {
  
  ImuEventCheckAndHandle(on_imu_event);

}

static inline void main_periodic_task( void ) {
  ImuPeriodic();
}

static inline void on_imu_event( void ) {
  switch (mtt_status) {

  case MT_STATUS_UNINIT :
    imu_v3_detect_vehicle_still();
    if (imu_vehicle_still) {
      ImuUpdateFromAvg();
      multitilt_start(imu_accel, imu_gyro);
    }
    break;

  case MT_STATUS_RUNNING :
    // t0 = T0TC;
    multitilt_predict(imu_gyro_prev);
    multitilt_update(imu_accel);
    // t1 = T0TC;
    // diff = t1 - t0;
    // DOWNLINK_SEND_TIME(&dif);
    break;
  }
  
  link_imu_send();

}
