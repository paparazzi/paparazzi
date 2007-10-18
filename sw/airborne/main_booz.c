
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
#include "booz_telemetry.h"

#include "spi.h"
#include "link_imu.h"
#include "booz_estimator.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_dl_parse_msg( void );

int main( void ) {
  main_init();
  while(1) {
    if (sys_time_periodic())
      main_periodic_task();
    main_event_task();
  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  led_init();
  sys_time_init();

  actuators_init();
  SetCommands(commands_failsafe);

  ppm_init();
  radio_control_init();

  spi_init();
  link_imu_init();
  uart1_init_tx();
  int_enable();
}

static inline void main_periodic_task( void ) {
  link_imu_periodic_task();

  static uint8_t _50hz = 0;
  _50hz++;
  if (_50hz > 5) _50hz = 0;
  switch (_50hz) {
  case 0:
    LED_TOGGLE(1);
    break;
  case 1:
    radio_control_periodic_task();
    break;
  case 2:
    booz_telemetry_periodic_task();
    break;
  }


}

static inline  void main_event_task( void ) {
  if (PprzBuffer()) {
    ReadPprzBuffer();
    if (pprz_msg_received) {
      pprz_parse_payload();
      pprz_msg_received = FALSE;
    }
  }
  if (dl_msg_available) {
    main_dl_parse_msg();
    dl_msg_available = FALSE;
    LED_TOGGLE(2);
  }

  if (spi_message_received) {
    spi_message_received = FALSE;
    link_imu_event_task();
  }

  if (ppm_valid) {
    ppm_valid = FALSE;
    radio_control_event_task();
    //    if (rc_values_contains_avg_channels) {
    //      fbw_mode = FBW_MODE_OF_PPRZ(rc_values[RADIO_MODE]);
    //    }
    /* setpoints */

  }

}

bool_t dl_msg_available;

#define MSG_SIZE 128
uint8_t dl_buffer[MSG_SIZE]  __attribute__ ((aligned));

#include "settings.h"

#define IdOfMsg(x) (x[1])

static inline void main_dl_parse_msg(void) {
  uint8_t msg_id = IdOfMsg(dl_buffer);
  if (msg_id == DL_SETTING) {
    uint8_t i = DL_SETTING_index(dl_buffer);
    float var = DL_SETTING_value(dl_buffer);
    DlSetting(i, var);
    DOWNLINK_SEND_DL_VALUE(&i, &var);
  }  
}
