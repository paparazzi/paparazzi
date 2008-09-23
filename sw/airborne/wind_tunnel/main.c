#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart.h"

#include "messages.h"
#include "downlink.h"

#include "datalink.h"
#include "settings.h"
#include "dl_protocol.h"

#include "wt_servo.h"


static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );


uint16_t motor_power;
uint8_t dl_buffer[MSG_SIZE]  __attribute__ ((aligned));
bool_t dl_msg_available;
uint16_t datalink_time;

int main( void ) {
  main_init();
  while(1) {
    if (sys_time_periodic())
      main_periodic_task();
    main_event_task( );
  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
  led_init();
  uart0_init_tx();

  motor_power = 0;
  wt_servo_init();
  wt_servo_set(500);

  int_enable();
}

static inline void main_periodic_task( void ) {
  //  LED_TOGGLE(1);
  DOWNLINK_SEND_TAKEOFF(&motor_power);
}

static inline void main_event_task( void ) {
  DatalinkEvent();
}


#define IdOfMsg(x) (x[1])

void dl_parse_msg(void) {

  LED_TOGGLE(1);

  uint8_t msg_id = IdOfMsg(dl_buffer);
  switch (msg_id) {
  
  case  DL_PING: {
    DOWNLINK_SEND_PONG();
    break;
  }
    
  case DL_SETTING : {
    uint8_t i = DL_SETTING_index(dl_buffer);
    float var = DL_SETTING_value(dl_buffer);
    DlSetting(i, var);
    DOWNLINK_SEND_DL_VALUE(&i, &var);
    break;
  }

  }
}
