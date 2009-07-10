#include <stm32/rcc.h>
#include <stm32/gpio.h>



#include <stm32/flash.h>
#include <stm32/misc.h>

#include CONFIG
#include "init_hw.h"
#include "sys_time.h"
#include "downlink.h"
static inline void main_init( void );
static inline void main_periodic( void );

int main(void) {

  main_init();

  while (1) {
    if (sys_time_periodic())
      main_periodic();
  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
}

static inline void main_periodic( void ) {
  RunOnceEvery(10, {DOWNLINK_SEND_TIME(&cpu_time_sec);});
}



