#include "main_booz.h"

#include "std.h"
#include "init_hw.h"
#include "interrupt_hw.h"
#include "sys_time.h"
#include "led.h"

#include "commands.h"
#include "i2c.h"
#include "actuators.h"
#include "radio_control.h"

#include "spi.h"
#include "link_imu.h"
#include "booz_estimator.h"
#include "booz_control.h"
#include "booz_autopilot.h"

#include "uart.h"
#include "messages.h"
#include "downlink.h"
#include "booz_telemetry.h"
#include "datalink.h"



int16_t trim_p = 0;
int16_t trim_q = 0;
int16_t trim_r = 0;
uint8_t vbat = 0;

//FIXME
#ifdef SITL
uint32_t link_imu_nb_err;
uint8_t  link_imu_status;
#endif

#ifndef SITL
int main( void ) {
  booz_main_init();
  while(1) {
    if (sys_time_periodic())
      booz_main_periodic_task();
    booz_main_event_task();
  }
  return 0;
}
#endif

STATIC_INLINE void booz_main_init( void ) {

  hw_init();
  led_init();
  sys_time_init();

  i2c_init();
  actuators_init();
  SetCommands(commands_failsafe);

  ppm_init();
  radio_control_init();

#ifndef SITL
  spi_init();
  link_imu_init();
#endif
  
  booz_estimator_init();
  booz_control_init();
  booz_autopilot_init();

  //FIXME
#ifndef SITL
  uart1_init_tx();
#endif

  int_enable();

  DOWNLINK_SEND_BOOT(&cpu_time_sec);
}

STATIC_INLINE void booz_main_periodic_task( void ) {
  
  // FIXME
#ifndef SITL
  link_imu_periodic_task();
#endif
  
  booz_autopilot_periodic_task();

  SetActuatorsFromCommands(commands);

  static uint8_t _50hz = 0;
  _50hz++;
  if (_50hz > 5) _50hz = 0;
  switch (_50hz) {
  case 0:
    break;
  case 1:
    radio_control_periodic_task();
    if (rc_status != RC_OK)
      booz_autopilot_mode = BOOZ_AP_MODE_FAILSAFE;
    break;
  case 2:
    booz_controller_telemetry_periodic_task();
    break;
  case 3:
    break;
  case 4:
    break;
  }


}

STATIC_INLINE void booz_main_event_task( void ) {
  
  // FIXME
#ifndef SITL
  DlEventCheckAndHandle();

  LinkImuEventCheckAndHandle();
#endif

  RadioControlEventCheckAndHandle(booz_autopilot_on_rc_event);
 
}
