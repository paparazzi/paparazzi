#include <inttypes.h>

#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "uart.h"
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
  uart0_init();
  uart1_init();
  int_enable();
}

static inline void main_periodic_task( void ) {
  RunOnceEvery(100, {
    LED_TOGGLE(4);
  });

}

static inline void main_event_task( void ) {

  while (Uart1ChAvailable())
    uart0_transmit(Uart1Getch());

  while (Uart0ChAvailable())
    uart1_transmit(Uart0Getch());

  
}
