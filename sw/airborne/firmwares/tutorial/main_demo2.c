
#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

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
}

static inline void main_periodic_task(void)
{
  LED_TOGGLE(1);
}
