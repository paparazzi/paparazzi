#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart.h"

#include "main_ap.h"
#include "airframe.h"

#include "messages.h"
#include "downlink.h"
#include "spi.h"
#include "baro_MS5534A.h"

#include "estimator.h"

float ground_alt;

#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE

static bool_t dl_msg_available;
static inline void main_dl_parse_msg( void );

void init_ap( void ) {
  /** init done in main_fbw */
  spi_init();

  baro_MS5534A_init();

  int_enable();

  baro_MS5534A_init();
}

void periodic_task_ap( void ) {
  //  LED_TOGGLE(1);
  //  DOWNLINK_SEND_TAKEOFF(&cpu_time_sec);

  static uint8_t _20Hz   = 0;
  _20Hz++;
  if (_20Hz>=3) _20Hz=0;

  if (!_20Hz) {
    baro_MS5534A_send();
  }
}

void event_task_ap( void ) {
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
    LED_TOGGLE(1);
  }

  if (spi_message_received) {
    /* Got a message on SPI. */
    spi_message_received = FALSE;
    baro_MS5534A_event_task();
    if (baro_MS5534A_available) {
      baro_MS5534A_available = FALSE;

      baro_MS5534A_z = ground_alt +((float)baro_MS5534A_ground_pressure - baro_MS5534A_pressure)*0.084;
      if (alt_baro_enabled) {
	EstimatorSetAlt(baro_MS5534A_z);
      }
    }
  }
}

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
