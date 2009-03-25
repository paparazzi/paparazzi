#include <inttypes.h>

#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "uart.h"
#include "messages.h"
#include "downlink.h"

#include "interrupt_hw.h"

#include "radio_control.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static void on_rc_event(void);

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

  ppm_init();
  radio_control_init();

  int_enable();
}

static inline void main_periodic_task( void ) {
  RunOnceEvery(100, {LED_TOGGLE(3); DOWNLINK_SEND_BOOT(&cpu_time_sec);});
  RunOnceEvery(10, { radio_control_periodic_task(); });
}

static inline void main_event_task( void ) {
  RadioControlEventCheckAndHandle(on_rc_event);
}

static void on_rc_event(void) {

}
