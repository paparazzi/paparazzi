#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"

#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);

static inline void main_dl_parse_msg(void);

int main(void)
{
  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
    main_event_task();
  }
  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  uart0_init_tx();
}

static inline void main_periodic_task(void)
{
  //  LED_TOGGLE(1);
  uint16_t time_sec = sys_time.nb_sec;
  DOWNLINK_SEND_TAKEOFF(&time_sec);
}

static inline void main_event_task(void)
{
  if (PprzBuffer()) {
    ReadPprzBuffer();
    if (pprz_msg_received) {
      pprz_parse_payload();
      pprz_msg_received = false;
    }
  }
  if (dl_msg_available) {
    main_dl_parse_msg();
    dl_msg_available = false;
    LED_TOGGLE(1);
  }
}

uint16_t foo;

bool dl_msg_available;

#define MSG_SIZE 256
uint8_t dl_buffer[MSG_SIZE]  __attribute__((aligned));

#include "generated/settings.h"

#define IdOfMsg(x) (x[1])

static inline void main_dl_parse_msg(void)
{
  uint8_t msg_id = IdOfMsg(dl_buffer);
  if (msg_id == DL_SETTING) {
    uint8_t i = DL_SETTING_index(dl_buffer);
    float var = DL_SETTING_value(dl_buffer);
    DlSetting(i, var);
    DOWNLINK_SEND_DL_VALUE(&i, &var);
  }
}
