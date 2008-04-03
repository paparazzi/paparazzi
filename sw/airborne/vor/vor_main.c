#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart.h"

#include "messages.h"
#include "downlink.h"

#include "vor_demod.h"

static inline void main_init( void );
static inline void main_periodic_task( void );

static uint32_t t0, t1;

int main( void ) {
  main_init();
  while(1) {
    if (sys_time_periodic())
      main_periodic_task();
  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
  led_init();
  uart0_init_tx();
  vor_demod_init();
  int_enable();
}

static inline void main_periodic_task( void ) {
  //  DOWNLINK_SEND_TAKEOFF(&cpu_time_sec);
  t0 = T0TC;

  const int16_t a0 =  0.0000294918571461433008684162315748977790 * (1<<14);
  const int16_t a1 =  0.0000884755714384299093815122727590960494 * (1<<14);
  const int16_t a2 =  0.0000884755714384299093815122727590960494 * (1<<14);
  const int16_t a3 =  0.0000294918571461433008684162315748977790 * (1<<14);
  const int16_t b1 =  1.0000000000000000000000000000000000000000 * (1<<14);
  const int16_t b2 = -2.8738524677701420273479016032069921493530 * (1<<14);
  const int16_t b3 =  2.7555363566291544152875303552718833088875 * (1<<14);
  const int16_t b4 = -0.8814479540018430592240861187747213989496 * (1<<14);

  //  vor_demod_periodic();
  t1 = T0TC;
  uint32_t dif =t1 - t0;
  //  t0 = t1;

  RunOnceEvery(255, {
      DOWNLINK_SEND_TIME(&dif);
    });
  
}
