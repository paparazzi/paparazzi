#include "std.h"
#include "init_hw.h"
#include "interrupt_hw.h"
#include "sys_time.h"
#include "led.h"

#include "commands.h"
#include "actuators.h"
#include "radio_control.h"

#include "uart.h"
#include "messages.h"
#include "downlink.h"
#include "tl_telemetry.h"
#include "datalink.h"
#include "tl_autopilot.h"
#include "tl_imu.h"
#include "tl_estimator.h"
#include "adc.h"
#include "tl_bat.h"
#include "gps.h"
#include "tl_nav.h"

static inline void tl_main_init( void );
static inline void tl_main_periodic_task( void );
static inline void tl_main_event_task( void );


int main( void ) {
  tl_main_init();
  while(1) {
    if (sys_time_periodic())
      tl_main_periodic_task();
    tl_main_event_task();
  }
  return 0;
}


static inline void tl_main_init( void ) {
  hw_init();
  led_init();
  adc_init();
  tl_bat_init();
  sys_time_init();

  actuators_init();
  SetCommands(commands_failsafe);

  ppm_init();
  radio_control_init();

  tl_imu_init();

  tl_estimator_init();

  tl_control_init();

  tl_nav_init();

  uart0_init_tx();
  uart1_init_tx();

  int_enable();

  DOWNLINK_SEND_BOOT(&cpu_time_sec);
}


static inline void tl_main_periodic_task( void ) {

  tl_bat_periodic_task();

  tl_estimator_periodic_task();

  tl_imu_periodic();
  if (telemetry_mode_Ap == TELEMETRY_MODE_Ap_test)
    DOWNLINK_SEND_TL_GYRO_RAW(&(buf_gr.sum));

  tl_estimator_use_gyro();
  
  tl_autopilot_periodic_task();

  radio_control_periodic_task();

  if (rc_status != RC_OK)
     tl_autopilot_mode = TL_AP_MODE_FAILSAFE;

 
  tl_telemetry_periodic_task();

  SetActuatorsFromCommands(commands);
}


static inline void tl_main_event_task( void ) {
  RadioControlEventCheckAndHandle(tl_autopilot_on_rc_event);

  GpsEventCheckAndHandle(tl_estimator_use_gps, !estimator_in_flight);

  DlEventCheckAndHandle();

  TlImuEventCheckAndHandle(tl_estimator_use_imu);
}
