#include "main_fbw.h"

#include "low_level_hw.h"
#include "int.h"
#include "sys_time.h"
#include "led.h"
#include "ppm.h"
#include "radio_control.h"
#include "command.h"
#include "control.h"
#include "autopilot_fbw.h"

#include "traces.h"

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
#ifdef _3DMG

#endif /* _3DMG */

#ifdef TRACES
  TRACES_INIT();
  //  uart1_init();
#endif /* TRACES */

  /* if FBW is running in a separate MCU */
#ifndef AP
#endif /* not AP */
  int_enable();
}

void periodic_task_fbw( void ) {
#ifdef RADIO_CONTROL
  if (radio_control_periodic_task()) {
    if (rc_status == RC_REALLY_LOST)
      command_set(failsafe_values);
  }
#endif /* RADIO_CONTROL */
}

void event_task_fbw( void ) {
#ifdef RADIO_CONTROL
  if (radio_control_ppm_event()) {
#ifdef TRACES
      {
	static uint16_t foo;
	foo++;
	if (!(foo%8)) {
	  PRINT_RADIO_CONTROL();
	}
      }
#endif /* TRACES */
#ifdef AUTOPILOT
    autopilot_process_radio_control();
#ifdef CONTROL
    if (autopilot_mode == MODE_MANUAL) {
      control_process_radio_control_MANUAL();
      command_set(control_commands);
#ifdef TRACES
      {
	static uint16_t foo;
	foo++;
	if (!(foo%8)) {
	  PRINT_RADIO_CONTROL();
	  PRINT_CONTROL_COMMANDS();
	}
      }
#endif /* TRACES */
    }
#endif /* CONTROL */
#endif /* AUTOPILOT */
  }
#endif /* RADIO_CONTROL */
}

