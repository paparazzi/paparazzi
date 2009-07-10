
#include <stm32/flash.h>
#include <stm32/misc.h>

#include CONFIG
#include "init_hw.h"
//#include "led.h"
#include "sys_time.h"

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
  //  led_init(); // handled by PERIPHERALS_AUTO_INIT
  sys_time_init();
}

static inline void main_periodic( void ) {
  //  LED_TOGGLE(1);
}


