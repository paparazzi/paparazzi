#include <inttypes.h>

#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "uart.h"
#include "messages.h"
#include "downlink.h"

#include "booz2_imu.h"

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

/*   LED_ON(4); */
/*   LED_ON(5); */
/*   LED_ON(6); */
/*   LED_ON(7); */

  uart0_init();
  booz2_imu_impl_init();
  booz2_imu_init();

  int_enable();
}

static inline void main_periodic_task( void ) {
  //#if 0
  RunOnceEvery(100, {
      //    LED_TOGGLE(7);
    DOWNLINK_SEND_ALIVE(16, MD5SUM);
  });
  //  uint16_t foo = ami601_status;
  //  DOWNLINK_SEND_BOOT(&foo);
  //#endif
  //  if (cpu_time_sec > 2)
  
  booz2_imu_periodic();
}

static inline void main_event_task( void ) {

  Booz2ImuEvent(on_imu_event);

}

static inline void on_imu_event(void) {
  Booz2ImuScaleGyro();
  Booz2ImuScaleAccel();

  //  LED_TOGGLE(6);
  static uint8_t cnt;
  cnt++;
  if (cnt > 15) cnt = 0;

  if (cnt == 0) {
    DOWNLINK_SEND_IMU_GYRO_RAW(&booz2_imu_gyro_unscaled.x,
			       &booz2_imu_gyro_unscaled.y,
			       &booz2_imu_gyro_unscaled.z);
    
    DOWNLINK_SEND_IMU_ACCEL_RAW(&booz2_imu_accel_unscaled.x,
				&booz2_imu_accel_unscaled.y,
				&booz2_imu_accel_unscaled.z);
  }
  else if (cnt == 7) {
    DOWNLINK_SEND_BOOZ2_GYRO(&booz2_imu_gyro.x,
			     &booz2_imu_gyro.y,
			     &booz2_imu_gyro.z);
    
    DOWNLINK_SEND_BOOZ2_ACCEL(&booz2_imu_accel.x,
			      &booz2_imu_accel.y,
			      &booz2_imu_accel.z);
  }  
}
