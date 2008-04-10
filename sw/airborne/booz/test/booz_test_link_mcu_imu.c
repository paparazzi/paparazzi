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

extern uint8_t foo_debug;
extern uint32_t it_count;

static inline void main_periodic(void) {
  RunOnceEvery(250, {				\
      static uint32_t it_last;			\
      uint16_t it_nb = it_count - it_last;	\
      it_last = it_count;			\
      DOWNLINK_SEND_BOOT(&it_nb)		\
	});
  //  RunOnceEvery(250,DOWNLINK_SEND_TAKEOFF(&foo_debug)); 
  booz_link_mcu_send();
}


static inline void main_event(void) {

}

