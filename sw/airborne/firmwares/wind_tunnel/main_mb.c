#define MODULES_C

#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"

#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#include "subsystems/datalink/datalink.h"
#include "generated/settings.h"
#include "generated/modules.h"
#include "pprzlink/dl_protocol.h"

#include "i2c.h"
#include "mb_twi_controller_mkk.h"
#include "mb_tacho.h"

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);


//uint16_t motor_power;
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

  i2c_init();
  mb_twi_controller_init();

  mb_tacho_init();
  modules_init();

  //motor_power = 0;

  mcu_int_enable();
}

static inline void main_periodic_task(void)
{
  LED_TOGGLE(1);
  //  DOWNLINK_SEND_TAKEOFF(&wt_servo_motor_power);
  //  DOWNLINK_SEND_DEBUG(3,buf_input);

  float rpm = mb_tacho_get_averaged();
  DOWNLINK_SEND_WT(&rpm);

  float throttle = (float)wt_servo_motor_power / 1000.;
  mb_twi_controller_set(throttle);

  modules_periodic_task();
}

static inline void main_event_task(void)
{
  modules_event_task();
}


#define IdOfMsg(x) (x[1])

void dl_parse_msg(struct link_device *dev __attribute__((unused)), struct transport_tx *trans __attribute__((unused)), uint8_t *buf)
{

  LED_TOGGLE(1);

  uint8_t msg_id = IdOfMsg(buf);
  switch (msg_id) {

    case  DL_PING: {
      DOWNLINK_SEND_PONG();
      break;
    }

    case DL_SETTING : {
      uint8_t i = DL_SETTING_index(buf);
      float var = DL_SETTING_value(buf);
      DlSetting(i, var);
      DOWNLINK_SEND_DL_VALUE(&i, &var);
      break;
    }

  }
}


