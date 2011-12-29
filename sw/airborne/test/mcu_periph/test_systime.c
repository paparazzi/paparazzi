#include "std_pprz.h"
#include "mcu.h"
#include "subsystems/led.h"

static inline void main_periodic_02( void );
static inline void main_periodic_03( void );
static inline void main_periodic_05( uint8_t id );
static inline void main_event( void );

int main(void) {
  
  mcu_init();
  led_init();
  unsigned int tmr_02 = sys_time_register_timer(SYS_TIME_TIMER_S(0.2), NULL);
  unsigned int tmr_03 = sys_time_register_timer(SYS_TIME_TIMER_S(0.3), NULL);
  sys_time_register_timer(SYS_TIME_TIMER_S(0.5), main_periodic_05);

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
      LED_TOGGLE(LED_GREEN);
}

static inline void main_periodic_03( void ) {
      LED_TOGGLE(LED_BLUE);
}

/* 
   Called from the systime interrupt handler
*/
static inline void main_periodic_05( uint8_t id ) {
      LED_TOGGLE(LED_RED);
}


static inline void main_event( void ) {
}

