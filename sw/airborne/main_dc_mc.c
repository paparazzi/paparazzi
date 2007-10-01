
#include "std.h"
#include "sys_time.h"
#include "led.h"
#include "uart.h"
#include "print.h"
#include "interrupt_hw.h"

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
  sys_time_init();
  led_init();
  uart0_init_tx();
  int_enable();
}

static inline void main_periodic_task( void ) {
  LED_TOGGLE(1);
  Uart0PrintString("demo3 running since ");
  Uart0PrintHex16(cpu_time_sec);
  Uart0PrintString(" seconds\n");
}
