
#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "mb_tacho.h"
#include "mb_servo.h"
#include "mb_current.h"


#include "uart.h"
#include "messages.h"
#include "downlink.h"

#include "adc.h"

#include "mb_modes.h"

static inline void main_init( void );
static inline void main_periodic_task( void );

int main( void ) {
  main_init();
  while(1) {
    if (sys_time_periodic())
      main_periodic_task();
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
  adc_init();
  mb_current_init();

  uart0_init_tx();
  mb_servo_arm();
  mb_mode_init();

  int_enable();
}

static inline void main_periodic_task( void ) {
  mb_mode_periodic();
  float throttle = mb_modes_throttle;
  mb_servo_set(throttle);
  float rpm = mb_tacho_get_averaged();
  mb_current_periodic();
  float amps = mb_current_amp;
  static uint8_t my_cnt = 0;
  my_cnt++;
  if (!(my_cnt%10)) {
    LED_TOGGLE(1);
    DOWNLINK_SEND_MOTOR_BENCH_STATUS(&cpu_time_ticks, &throttle, &rpm, &amps , &cpu_time_sec, &mb_modes_mode);
  }
}
