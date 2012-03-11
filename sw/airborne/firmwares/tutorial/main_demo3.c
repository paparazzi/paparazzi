
#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "print.h"

static inline void main_init( void );
static inline void main_periodic_task( void );

int main( void ) {
  main_init();
  while(1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic_task();
  }
  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  uart0_init_tx();
  mcu_int_enable();
}

static inline void main_periodic_task( void ) {
  LED_TOGGLE(1);
  Uart0PrintString("demo3 running since ");
  Uart0PrintHex32(cpu_time_sec);
  Uart0PrintString(" seconds\n");
}
