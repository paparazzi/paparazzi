
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

#include "paparazzi.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void);

static inline void bench_periodic ( void );
static inline bool_t prbs_10( void ) ;

#define MODE_MANUAL 0
#define MODE_RAMP   1
#define MODE_STEP   2
#define MODE_PRBS   3

uint8_t mode;
uint16_t throttle;

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
  mode = MODE_PRBS;
  int_enable();
}

static inline void main_event_task( void ) {

}

static inline void main_periodic_task( void ) {
  bench_periodic();
  DOWNLINK_SEND_MOTOR_BENCH_STATUS(&cpu_time_ticks, &cpu_time_sec, &throttle, &mode);

  static uint8_t cnt;
  cnt++;
  if (!(cnt%16)) {
    LED_TOGGLE(1);
    //  DOWNLINK_SEND_MOTOR_BENCH_STATUS(&cpu_time_ticks, &cpu_time_sec, &throttle, &mode);
  }
}

#define PRBS_VAL1 (0.55*MAX_PPRZ)
#define PRBS_VAL2 (0.65*MAX_PPRZ)

static inline void bench_periodic ( void ) {
  switch (mode) {
  case MODE_MANUAL:
    throttle = 0;
    break;
  case MODE_PRBS:
    throttle = prbs_10() ? PRBS_VAL1 : PRBS_VAL2;
    break;
  }
}

static inline bool_t prbs_10( void ) {
  static uint16_t lsr = 0xFFFF;
  uint8_t feed =
    bit_is_set(lsr,6) ^
    bit_is_set(lsr,9);
  uint8_t val = bit_is_set(lsr,9);
  lsr = (lsr << 1) | feed;
  return val;
}





