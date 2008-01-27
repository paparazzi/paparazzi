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
#include "tl_estimator.h"
#include "gps.h"

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
  sys_time_init();

  actuators_init();
  SetCommands(commands_failsafe);

  ppm_init();
  radio_control_init();

  tl_estimator_init();

  uart0_init_tx();
  uart1_init_tx();

  int_enable();

  DOWNLINK_SEND_BOOT(&cpu_time_sec);
}


static inline void tl_main_periodic_task( void ) {
  
  /* run control loops */
  tl_autopilot_periodic_task();

  SetActuatorsFromCommands(commands);

  radio_control_periodic_task();
  if (rc_status != RC_OK)
     tl_autopilot_mode = TL_AP_MODE_FAILSAFE;
  
  tl_telemetry_periodic_task();
}



static inline void tl_main_event_task( void ) {
  
  //DlEventCheckAndHandle();

  RadioControlEventCheckAndHandle(tl_autopilot_on_rc_event);

#ifdef GPS
  if (GpsBuffer()) {
    ReadGpsBuffer();
  }
  if (gps_msg_received) {
    /* parse and use GPS messages */
#ifdef GPS_CONFIGURE
    if (gps_status_config < GPS_CONFIG_DONE)		
      gps_configure();
    else
#endif
      parse_gps_msg();
    gps_msg_received = FALSE;
    if (gps_pos_available) {
      gps_verbose_downlink = !estimator_in_flight;
      UseGpsPos(tl_estimator_use_gps);
      gps_pos_available = FALSE;
    }
  }
#endif /** GPS */
}
