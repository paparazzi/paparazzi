
#include "std.h"
#include "sys_time.h"
#include "init_hw.h"
#include "interrupt_hw.h"

#include "uart.h"
#include "print.h"

#include "adc.h"

#include "icp_scale.h"

#include "tacho_mb.h"

#include "messages.h"
#include "downlink.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void);

int main( void ) {
  main_init();
  LED_ON(1);
  LED_ON(2);
  while (1) {
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
  uart0_init_tx();
  adc_init();
  icp_scale_init();
  tacho_mb_init();
  int_enable();
}

static uint32_t t0, t1;

static inline void main_event_task( void ) {

}

static inline void main_periodic_task( void ) {
  t0 = T0TC;
  static uint8_t cnt;
  cnt++;
  if (!(cnt%16)) {
    //    LED_TOGGLE(1);
    uart0_transmit('#');
    Uart0PrintHex32(pulse_len);
    uart0_transmit(' ');
    Uart0PrintHex32(t_duration);
    uart0_transmit('\n');
  }
}



