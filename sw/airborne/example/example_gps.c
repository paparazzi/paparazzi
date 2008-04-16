#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart.h"

#include "messages.h"
#include "downlink.h"

#include "gps.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );
static inline void on_gps( void );

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
  uart0_init_tx();
  uart1_init_tx();
  gps_init();
 int_enable();
}

static inline void main_periodic_task( void ) {
  LED_TOGGLE(1);
  DOWNLINK_SEND_TAKEOFF(&cpu_time_sec);
}

static inline void main_event_task( void ) {

  GpsEventCheckAndHandle(on_gps, FALSE);

}

static inline void on_gps( void ) {
  // do something clever
}
