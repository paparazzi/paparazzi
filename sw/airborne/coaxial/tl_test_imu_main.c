#include "std.h"
#include "init_hw.h"
#include "interrupt_hw.h"
#include "sys_time.h"
#include "led.h"

#include "uart.h"
#include "messages.h"
#include "downlink.h"

#include "tl_imu.h"

static inline void tl_main_init( void );
static inline void tl_main_periodic_task( void );
static inline void tl_main_event_task( void );

int main( void ) {
  tl_main_init();
  while(1) {
    if (sys_time_periodic())
      tl_main_periodic_task();
    tl_main_event_task();
  }
  return 0;
}

static inline void tl_main_init( void ) {
  hw_init();
  led_init();
  sys_time_init();
  uart0_init_tx();

  adc_init();
  tl_imu_init();

  int_enable();
}

static inline void tl_main_periodic_task( void ) {
  DOWNLINK_SEND_BOOT(&cpu_time_sec);
  tl_imu_periodic();
}

static void imu_on_event(void) {
  DOWNLINK_SEND_IMU_MAG(&tl_imu_hx, &tl_imu_hy, &tl_imu_hz); 
  float psi = atan2(tl_imu_hy, tl_imu_hx);
  DOWNLINK_SEND_TL_ESTIMATOR(&tl_imu_r, &psi, &tl_imu_z_baro); 
}

static inline void tl_main_event_task( void ) {
  TlImuEventCheckAndHandle(imu_on_event);
}





