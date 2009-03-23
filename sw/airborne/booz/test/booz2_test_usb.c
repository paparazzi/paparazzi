#include <inttypes.h>

#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"

//#include "uart.h"
#include "usb_serial.h"

#include "messages.h"
#include "downlink.h"

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

/*   LED_ON(4); */
/*   LED_ON(5); */
/*   LED_ON(6); */
/*   LED_ON(7); */

  //uart1_init_tx();
  VCOM_init();

  int_enable();
}

static inline void main_periodic_task( void ) {
  RunOnceEvery(100, {
      LED_TOGGLE(1);
      DOWNLINK_SEND_ALIVE(16, MD5SUM);
    });
}

static inline void main_event_task( void ) {

}

