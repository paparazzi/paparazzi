
#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "mb_tacho.h"
#include "mb_servo.h"
#include "i2c.h"
#include "mb_twi_controller.h"
#include "mb_current.h"
#include "mb_scale.h"


#include "uart.h"
#include "messages.h"
#include "downlink.h"

#include "adc.h"

#include "mb_modes.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_dl_parse_msg( void );

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
  //initialisation 
  hw_init();
  led_init();
  sys_time_init();
  mb_tacho_init();
  mb_servo_init();
  mb_servo_set_range( 1275000, 1825000 );

  i2c_init();
  mb_twi_controller_init();

  adc_init();
  mb_current_init();
  mb_scale_init();

  uart0_init_tx();
  mb_mode_init();

  int_enable();
}

static inline void main_periodic_task( void ) {
  mb_mode_periodic();
  float throttle = mb_modes_throttle;
  mb_servo_set(throttle);

  mb_twi_controller_set(throttle);

  float rpm = mb_tacho_get_averaged();
  mb_current_periodic();
  float amps = mb_current_amp;
  mb_scale_periodic();
  float thrust = mb_scale_thrust;
  float torque = 0.;

  static uint8_t my_cnt = 0;
  my_cnt++;
  if (!(my_cnt%10)) {
    LED_TOGGLE(1);
  }
  DOWNLINK_SEND_MOTOR_BENCH_STATUS(&cpu_time_ticks, &throttle, &rpm, &amps , &thrust, &torque, &cpu_time_sec, &mb_modes_mode);
}

static inline  void main_event_task( void ) {
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
    LED_TOGGLE(1);
  }
}

bool_t dl_msg_available;

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
