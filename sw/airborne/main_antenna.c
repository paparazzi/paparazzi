
#include "std.h"
#include "sys_time.h"
#include "init_hw.h"
#include "interrupt_hw.h"

#include "led.h"

#include "uart.h"
#include "print.h"

#include "adc.h"

#include "ant_v2x.h"

#include "messages.h"
#include "downlink.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void);

uint8_t track_mode;

int main( void ) {
  main_init();
  LED_ON(1);
  LED_OFF(2);
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
  uart1_init_tx();
  adc_init();
  int_enable();
}

static inline void main_event_task( void ) {
 
  if (ant_v2x_data_available) {
    ant_v2x_read_data();
    DOWNLINK_SEND_ANTENNA_DEBUG(&ant_v2x_data.xraw, &ant_v2x_data.yraw, \
                                &ant_v2x_data.xcal, &ant_v2x_data.ycal, \
                                &ant_v2x_data.heading, &ant_v2x_data.magnitude, \
                                &ant_v2x_data.temp, &ant_v2x_data.distor, \
                                &ant_v2x_data.cal_status);
    ant_v2x_data_available = FALSE;
  }

  if (PprzBuffer()) {
    ReadPprzBuffer();
    if (pprz_msg_received) {
      pprz_parse_payload();
      pprz_msg_received = FALSE;
    }
  }
  if (dl_msg_available) {
    dl_parse_msg();
    dl_msg_available = FALSE;
  }

}

static inline void main_periodic_task( void ) {
  static uint8_t cnt;
  cnt++;
  if (!(cnt%16)) {
    LED_TOGGLE(2);
    //    uart1_transmit('#');
    //    Uart1PrintHex(cnt);
    //    uart1_transmit('\n');
  }
}



