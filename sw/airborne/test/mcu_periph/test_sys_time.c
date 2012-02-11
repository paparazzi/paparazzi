#include "std.h"
#include "mcu.h"
#include "led.h"
#include "mcu_periph/sys_time.h"

static inline void main_periodic_02( void );
static inline void main_periodic_03( void );
static inline void main_periodic_05( uint8_t id );
static inline void main_event( void );

int main(void) {

  mcu_init();
  unsigned int tmr_02 = sys_time_register_timer(0.2, NULL);
  unsigned int tmr_03 = sys_time_register_timer(0.3, NULL);
  sys_time_register_timer(0.5, main_periodic_05);

  while(1) {
    if (sys_time_check_and_ack_timer(tmr_02))
      main_periodic_02();
    if (sys_time_check_and_ack_timer(tmr_03))
      main_periodic_03();
    main_event();
  }

  return 0;
}

/*
   Called from main loop polling
*/
static inline void main_periodic_02( void ) {
#ifdef LED_GREEN
      LED_TOGGLE(LED_GREEN);
#endif
}

static inline void main_periodic_03( void ) {
#ifdef LED_BLUE
      LED_TOGGLE(LED_BLUE);
#endif
}

/*
   Called from the systime interrupt handler
*/
static inline void main_periodic_05( uint8_t id ) {
#ifdef LED_RED
      LED_TOGGLE(LED_RED);
#endif
}


static inline void main_event( void ) {
}
