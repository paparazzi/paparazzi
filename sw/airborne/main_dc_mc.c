
#include "std.h"
#include "sys_time.h"
#include "led.h"
#include "uart.h"
#include "messages.h"
#include "downlink.h"

#include "interrupt_hw.h"

#include "dc_mc_link.h"
#include "dc_mc_power.h"

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
  sys_time_init();
  led_init();
  uart0_init_tx();
  dc_mc_link_init();
  dc_mc_power_init();
  int_enable();
}

static inline void main_periodic_task( void ) {
  //  LED_TOGGLE(1);
  dc_mc_link_periodic();
  dc_mc_power_set(dc_mc_link_command);
  DOWNLINK_SEND_DC_MC_STATUS(&cpu_time_sec, &cpu_time_ticks, &dc_mc_link_command);
  
}

static inline void main_event_task( void ) {
  if (dc_mc_link_event) {
    LED_TOGGLE(1);
    dc_mc_link_command = dc_mc_link_twi_rx_buf[0] | dc_mc_link_twi_rx_buf[1] << 8;
    dc_mc_link_event = FALSE;
    dc_mc_power_set(dc_mc_link_command);
  }
}
