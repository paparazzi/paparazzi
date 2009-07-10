#include <inttypes.h>

#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "uart.h"
#include "messages.h"
#include "downlink.h"
#include "booz2_gps.h"
#include "interrupt_hw.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static void on_gps_sol(void);

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
  uart0_init();
  uart1_init();
  booz2_gps_init();
  int_enable();
}

static inline void main_periodic_task( void ) {



}

static inline void main_event_task( void ) {
  Booz2GpsEvent(on_gps_sol);
  
}

static void on_gps_sol(void) {
  uint16_t foo = booz_gps_state.num_sv;
  DOWNLINK_SEND_BOOT(&foo);

}
