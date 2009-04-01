#include <inttypes.h>

#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

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
  int_enable();
  //  SetBit(IO0DIR, 18);
  //  SetBit(IO0SET, 18);
  //SetBit(IO0CLR, 18);
}

static inline void main_periodic_task( void ) {
  //#if 0
  RunOnceEvery(100, {
    LED_TOGGLE(1);
    LED_TOGGLE(2);
    LED_TOGGLE(3);
    LED_TOGGLE(4);
  });
  //#endif
}

static inline void main_event_task( void ) {

}
