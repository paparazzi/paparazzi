#include "init_hw.h"
#include "interrupt_hw.h"
#include "sys_time.h"
#include "led.h"
#include "actuators.h"
#include "airframe.h"
#define DATALINK_C
#include "datalink.h"
#include "uart.h"
#include "pprz_transport.h"
#include "main_fbw.h"


#define IdOfMsg(x) (x[1])

#define SetServo(x, v) Actuator(x) = SERVOS_TICS_OF_USEC(ChopServo(v,700,2400));

void dl_parse_msg( void ) {
  uint8_t msg_id = IdOfMsg(dl_buffer);
  if (msg_id == DL_SET_ACTUATOR) {
    uint8_t servo_no = DL_SET_ACTUATOR_no(dl_buffer);
    uint16_t servo_value = DL_SET_ACTUATOR_value(dl_buffer);
    if (servo_no < SERVOS_NB)
      SetServo(servo_no, servo_value);
  }
}


void init_fbw( void ) {
  hw_init();
  sys_time_init();
  led_init();

  Uart0Init();

  actuators_init();

  uint8_t i;
  for(i = 0; i < SERVOS_NB; i++)
    SetServo(i, 1500);

  SetServo(SERVO_GAZ, SERVO_GAZ_MIN);

  int_enable();
}

void periodic_task_fbw(void) {
/*    static float t; */
/*    t += 1./60.; */
/*    uint16_t servo_value = 1500+ 500*sin(t); */
/*    SetServo(SERVO_AILEVON_LEFT, servo_value); */
}

void event_task_fbw(void) {
  if (PprzBuffer()) {
    ReadPprzBuffer();
  }
  if (pprz_msg_received) {
    pprz_msg_received = FALSE;
    LED_TOGGLE(2);
    pprz_parse_payload();
  } 
  if (dl_msg_available) {
    dl_parse_msg();
    dl_msg_available = FALSE;
  }
}
