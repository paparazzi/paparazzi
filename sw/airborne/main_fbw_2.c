#include "main_fbw.h"

#include "low_level_hw.h"
#include "int.h"
#include "sys_time.h"
#include "led.h"
#include "ppm.h"
#include "radio_control.h"
#include "command.h"
#include "control_2.h"

void init_fbw( void ) {
  low_level_init();
  sys_time_init();
#ifdef LED
  led_init();
#endif /* LED */
#ifdef ACTUATORS
  command_init();
#endif /* ACTUATORS */
#ifdef RADIO_CONTROL
  ppm_init();
  radio_control_init();
#endif /* RADIO_CONTROL */

  /* if FBW is running in a separate MCU */
#ifndef AP
#endif /* not AP */
  int_enable();
}

void periodic_task_fbw( void ) {
#ifdef RADIO_CONTROL
  radio_control_periodic_task();
#endif /* RADIO_CONTROL */
}

void event_task_fbw( void ) {
#ifdef RADIO_CONTROL
  if (radio_control_ppm_event())
    control_process_radio_control();
#endif /* RADIO_CONTROL */
}

