#include "main_fbw.h"

#include "low_level_hw.h"
#include "sys_time.h"
#include "led.h"
//#include "command.h"
#include "servos.h"

void init_fbw( void ) {
  low_level_init();
  sys_time_init();
#ifdef LED
  led_init();
#endif /* LED */
#ifdef ACTUATORS
  servos_init();
#endif /* ACTUATORS */

  /* if FBW is running in a separate MCU */
#ifndef AP

#endif /* AP */
}

void periodic_task_fbw( void ) {
  //  LED_TOGGLE(1);
  LED_TOGGLE(2);
  //  LED_TOGGLE(3);
}

void event_task_fbw( void ) {

}

