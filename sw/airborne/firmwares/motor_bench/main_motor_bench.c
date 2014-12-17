
#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mb_tacho.h"
#include "mb_servo.h"
#include "i2c.h"
#include "mb_twi_controller_asctech.h"
//#include "mb_twi_controller_mkk.h"
#include "mb_current.h"
#include "mb_scale.h"


#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "generated/settings.h"


#include "mb_modes.h"
//#include "mb_static.h"

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
  mb_tacho_init();

#if defined USE_TWI_CONTROLLER
  i2c0_init();
  mb_twi_controller_init();
#endif

  mb_servo_init();
  mb_servo_set_range(1090000, 1910000);

  mb_current_init();
  mb_scale_init();

  uart0_init();
  mb_mode_init();

  mcu_int_enable();
}

static inline void main_periodic_task(void)
{
  float rpm = mb_tacho_get_averaged();
  mb_current_periodic();
  float amps = mb_current_amp;
  float thrust = mb_scale_thrust;
  float torque = 0.;

  mb_mode_periodic(rpm, thrust, amps);

  float throttle = mb_modes_throttle;

#if defined USE_TWI_CONTROLLER
  mb_twi_controller_set(throttle);
#endif
  mb_servo_set(throttle);


  RunOnceEvery(125, {
    DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
    PeriodicSendDlValue(DefaultChannel);
  });
  uint16_t time_secs = sys_time.nb_sec;
  DOWNLINK_SEND_MOTOR_BENCH_STATUS(DefaultChannel, DefaultDevice, &sys_time.nb_sec_rem, &throttle, &rpm, &amps , &thrust,
                                   &torque, &time_secs, &mb_modes_mode);




}

static inline  void main_event_task(void)
{
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
  }
}

bool_t dl_msg_available;

#define MSG_SIZE 128
uint8_t dl_buffer[MSG_SIZE]  __attribute__((aligned));


#define IdOfMsg(x) (x[1])

static inline void main_dl_parse_msg(void)
{
  uint8_t msg_id = IdOfMsg(dl_buffer);
  //  if (msg_id == DL_PING) {}
  if (msg_id == DL_SETTING) {
    LED_TOGGLE(1);
    uint8_t i = DL_SETTING_index(dl_buffer);
    float var = DL_SETTING_value(dl_buffer);
    DlSetting(i, var);
    DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &var);
  }
}
