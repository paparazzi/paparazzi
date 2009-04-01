#include <inttypes.h>

#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart.h"

#include "messages.h"
#include "downlink.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

uint32_t t0, t1, diff;

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
  int_enable();
}

static inline void main_periodic_task( void ) {
  RunOnceEvery(10, {LED_TOGGLE(2); DOWNLINK_SEND_TIME(&cpu_time_sec);});
}

static inline void main_event_task( void ) {

}

