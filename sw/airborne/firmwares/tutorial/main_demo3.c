
#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "modules/datalink/uart_print.h"

static inline void main_init(void);
static inline void main_periodic_task(void);

int main(void)
{
  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
  }
  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  uart0_init_tx();
}

static inline void main_periodic_task(void)
{
  LED_TOGGLE(1);
  UART0PrintString("demo3 running since ");
  UART0PrintHex32(sys_time.nb_sec);
  UART0PrintString(" seconds\n");
}
