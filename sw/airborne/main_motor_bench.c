
#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "mb_tacho.h"
#include "mb_servo.h"


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

static struct adc_buf cur_sensor_buf;


static inline void main_init( void ) {
  //initialisation 
  hw_init();
  sys_time_init();
  led_init();
  adc_init();
  mb_tacho_init();
  mb_servo_init();
  uart0_init_tx();
  int_enable();
  mb_servo_arm();
  mb_servo_set(0.5);

  mb_mode_init();

  adc_buf_channel(0, &cur_sensor_buf, 16);

}

extern uint16_t adc0_val[];

static inline void main_periodic_task( void ) {

  mb_mode_periodic();
  mb_servo_set(mb_modes_throttle);
  float rpm = mb_tacho_get_averaged();
  //  LED_TOGGLE(1);
  //  uint16_t cur_int = cur_sensor_buf.sum / cur_sensor_buf.av_nb_sample;
  uint16_t cur_int = adc0_val[0];
  float foo2 = cur_int * 1.;

  DOWNLINK_SEND_MOTOR_BENCH_STATUS(&cpu_time_ticks, &mb_modes_throttle, &rpm, &foo2 , &cpu_time_sec, &mb_modes_mode);
}
