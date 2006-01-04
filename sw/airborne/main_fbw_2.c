#include "main_fbw.h"

#include "low_level_hw.h"
#include "int.h"
#include "sys_time.h"
#include "int.h"
#include "led.h"
#include "ppm.h"
//#include "command.h"
#include "servos_hw.h"
#include "ppm.h"
#include "radio_control.h"

void init_fbw( void ) {
  low_level_init();
  sys_time_init();
#ifdef LED
  led_init();
  LED_OFF(1);
  LED_OFF(2);
  //  LED_OFF(3);
#endif /* LED */
#ifdef ACTUATORS
  servos_init();
#endif /* ACTUATORS */
#ifdef RADIO_CONTROL
  ppm_init();
  radio_control_init();
#endif /* RADIO_CONTROL */

  /* if FBW is running in a separate MCU */
#ifndef AP
#endif /* AP */
  int_enable();
}

void periodic_task_fbw( void ) {
#ifdef RADIO_CONTROL
  radio_control_periodic_task();
#endif /* RADIO_CONTROL */
  //LED_TOGGLE(1);
  //  LED_TOGGLE(2);
  //  LED_TOGGLE(3);
}

void event_task_fbw( void ) {
#ifdef RADIO_CONTROL
  radio_control_ppm_event();
#endif /* RADIO_CONTROL */
}

