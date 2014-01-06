#include "std.h"
#include "mcu.h"
#include "interrupt_hw.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "ADS8344.h"

static inline void main_init( void );
static inline void main_periodic( void );
static inline void main_event( void );

int main( void ) {
  main_init();
  while(1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic();
    main_event();
  }
  return 0;
}

static inline void main_init( void ) {
  mcu_init();

  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);

  led_init();

  uart1_init_tx();

  ADS8344_init();

  mcu_int_enable();

  ADS8344_start();
}


static inline void main_periodic( void ) {
  DOWNLINK_SEND_BOOT(&sys_time.nb_sec);

}

static inline void main_event( void ) {
  if (  ADS8344_available ) {
    float foo = ADS8344_values[0];
    float foo1 = ADS8344_values[1];
    float foo2 = ADS8344_values[2];
    DOWNLINK_SEND_IMU_GYRO(&foo, &foo1, &foo2);
    //    LED_TOGGLE(2);
    ADS8344_available = FALSE;
  }
}
