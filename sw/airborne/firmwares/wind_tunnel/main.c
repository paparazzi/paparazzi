#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"

#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#include "subsystems/datalink/datalink.h"
#include "generated/settings.h"
#include "pprzlink/dl_protocol.h"

#include "wt_servo.h"

#include "spi.h"
#include "wt_baro.h"

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);


uint16_t motor_power;
uint8_t dl_buffer[MSG_SIZE]  __attribute__((aligned));
bool dl_msg_available;
uint16_t datalink_time;

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
  led_init();
  uart0_init();

  motor_power = 0;
  wt_servo_init();
  wt_servo_set(500);

  spi_init();
  wt_baro_init();

  mcu_int_enable();
}

static inline void main_periodic_task(void)
{
  LED_TOGGLE(1);
  DOWNLINK_SEND_TAKEOFF(&motor_power);
  wt_baro_periodic();
  DOWNLINK_SEND_DEBUG(3, buf_input);
}

static inline void main_event_task(void)
{
  DatalinkEvent();

  // spi baro
  if (spi_message_received) {
    /* Got a message on SPI. */
    spi_message_received = false;
    wt_baro_event();
    uint16_t temp = 0;
    float alt = 0.;
    DOWNLINK_SEND_BARO_MS5534A(&wt_baro_pressure, &temp, &alt);
  }

}


#define IdOfMsg(x) (x[1])

void dl_parse_msg(void)
{

  LED_TOGGLE(1);

  uint8_t msg_id = IdOfMsg(dl_buffer);
  switch (msg_id) {

    case  DL_PING: {
      DOWNLINK_SEND_PONG();
      break;
    }

    case DL_SETTING : {
      uint8_t i = DL_SETTING_index(dl_buffer);
      float var = DL_SETTING_value(dl_buffer);
      DlSetting(i, var);
      DOWNLINK_SEND_DL_VALUE(&i, &var);
      break;
    }

  }
}


