#include "std.h"
#include "mcu.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "mcu_periph/usb_serial.h"

#include "messages.h"
#include "downlink.h"

static inline void main_init( void );
static inline void main_periodic_task( void );

int main( void ) {
  main_init();
  while(1) {
    if (sys_time_periodic())
      main_periodic_task();
  }
  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_init();
  led_init();
  usb_serial_init();
  mcu_int_enable();
}

static inline void main_periodic_task( void ) {
  LED_TOGGLE(1);
  //  DOWNLINK_SEND_TAKEOFF(&cpu_time_sec);
  usb_serial_transmit( 'A' );
  usb_serial_transmit( '\n' );

}
