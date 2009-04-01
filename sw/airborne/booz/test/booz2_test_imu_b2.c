#include <inttypes.h>

#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "uart.h"
#include "messages.h"
#include "downlink.h"

#include "booz2_imu.h"
#include "booz2_imu_b2.h"

#include "interrupt_hw.h"


static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void on_imu_event(void);

int main( void ) {
  main_init();
  while(1) {
    if (sys_time_periodic())
      main_periodic_task();
    main_event_task();
  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
  led_init();
  uart1_init_tx();
  booz2_imu_impl_init();
  booz2_imu_init();

  int_enable();
}

static inline void main_periodic_task( void ) {
  //#if 0
  RunOnceEvery(100, {
    LED_TOGGLE(3);
    DOWNLINK_SEND_ALIVE(16, MD5SUM);
  });
  booz2_imu_periodic();

}

static inline void main_event_task( void ) {

  Booz2ImuEvent(on_imu_event);

}

static inline void on_imu_event(void) {
  Booz2ImuScaleGyro();
  Booz2ImuScaleAccel();

  LED_TOGGLE(2);
  static uint8_t cnt;
  cnt++;
  if (cnt > 15) cnt = 0;

  if (cnt == 0) {
    DOWNLINK_SEND_IMU_GYRO_RAW(&booz_imu.gyro_unscaled.p,
			       &booz_imu.gyro_unscaled.q,
			       &booz_imu.gyro_unscaled.r);
    
    DOWNLINK_SEND_IMU_ACCEL_RAW(&booz_imu.accel_unscaled.x,
				&booz_imu.accel_unscaled.y,
				&booz_imu.accel_unscaled.z);
  }
  else if (cnt == 7) {
    DOWNLINK_SEND_BOOZ2_GYRO(&booz_imu.gyro.p,
			     &booz_imu.gyro.q,
			     &booz_imu.gyro.r);
    
    DOWNLINK_SEND_BOOZ2_ACCEL(&booz_imu.accel.x,
			      &booz_imu.accel.y,
			      &booz_imu.accel.z);
  }  
}
