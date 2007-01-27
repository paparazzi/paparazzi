
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

#include "motor_bench.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void);
static inline void main_dl_parse_msg( void );

int main( void ) {
  main_init();
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
  motor_bench_init();
  int_enable();
}

static inline void main_event_task( void ) {
  if (PprzBuffer()) {
    ReadPprzBuffer();
    if (pprz_msg_received) {
      pprz_parse_payload();
      pprz_msg_received = FALSE;
    }
  }
  if (dl_msg_available) {
    main_dl_parse_msg();
    dl_msg_available = FALSE;
    LED_TOGGLE(2);
  }
}

static inline void main_periodic_task( void ) {
  motor_bench_periodic();
  DOWNLINK_SEND_MOTOR_BENCH_STATUS(&cpu_time_ticks, &cpu_time_sec, &motor_bench_throttle, &motor_bench_mode);

  static uint8_t cnt;
  cnt++;
  if (!(cnt%16)) {
    LED_TOGGLE(1);
    //  DOWNLINK_SEND_MOTOR_BENCH_STATUS(&cpu_time_ticks, &cpu_time_sec, &throttle, &mode);
  }
}

bool_t dl_msg_available;
/** Flag provided to control calls to ::dl_parse_msg. NOT used in this module*/
#define MSG_SIZE 128
uint8_t dl_buffer[MSG_SIZE]  __attribute__ ((aligned));

#include "settings.h"

#define IdOfMsg(x) (x[1])

static inline void main_dl_parse_msg(void) {
  uint8_t msg_id = IdOfMsg(dl_buffer);
  if (msg_id == DL_SETTING) {
    uint8_t i = DL_SETTING_index(dl_buffer);
    float var = DL_SETTING_value(dl_buffer);
    DlSetting(i, var);
    DOWNLINK_SEND_DL_VALUE(&i, &var);
  }  
}



