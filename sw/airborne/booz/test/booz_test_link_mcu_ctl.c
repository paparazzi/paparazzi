#include "std.h"
#include "init_hw.h"
#include "interrupt_hw.h"
#include "sys_time.h"

#include "uart.h"
#include "messages.h"
#include "downlink.h"

#include "booz_link_mcu.h"

static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);

int main( void ) {
  main_init();
  while (1) {
    if (sys_time_periodic())
      main_periodic();
    main_event();
  }
  return 0;
}


static inline void main_init(void) {
  hw_init();

  sys_time_init();

  uart1_init_tx();

  booz_link_mcu_init();

  int_enable();
}

extern volatile uint32_t it_cnt;
extern volatile uint32_t rx_it_cnt;
extern volatile uint32_t ti_it_cnt;

static inline void main_periodic(void) {
  RunOnceEvery(250, {					\
      static uint32_t it_last;				\
      uint16_t it_nb = it_cnt - it_last;		\
      it_last = it_cnt;					\
      DOWNLINK_SEND_TAKEOFF(&it_nb);			\
      static float rx_it_last;				\      
      float diff_rx = rx_it_cnt - rx_it_last;		\
      rx_it_last = rx_it_cnt;				\
      static float ti_it_last;				\
      float diff_ti = ti_it_cnt - ti_it_last;		\
      ti_it_last = ti_it_cnt;				\
      DOWNLINK_SEND_BOOZ_DEBUG_BAR(&diff_rx, &diff_ti);	\
    });
}


static inline void on_link_event(void) {
  RunOnceEvery(250, {					\

      uint8_t f0 = link_mcu_rx_buf[0];			\
      uint8_t f1 = link_mcu_rx_buf[1];			\
      uint8_t f2 = link_mcu_rx_buf[2];			\
      uint8_t f3 = link_mcu_rx_buf[3];			\
      DOWNLINK_SEND_BOOZ_CMDS(&f0, &f1, &f2, &f3);	\
    });



}

static inline void main_event(void) {
  BoozLinkMcuEventCheckAndHandle(on_link_event);
}

