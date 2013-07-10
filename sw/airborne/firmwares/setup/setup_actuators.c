#include "generated/airframe.h"

#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "subsystems/actuators.h"

#include "firmwares/fixedwing/main_fbw.h"


#define DATALINK_C
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/pprz_transport.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"

#include "generated/settings.h"

#define IdOfMsg(x) (x[1])


void dl_parse_msg( void ) {
  uint8_t msg_id = IdOfMsg(dl_buffer);
  if (msg_id == DL_SET_ACTUATOR) {
    uint8_t servo_no = DL_SET_ACTUATOR_no(dl_buffer);
    uint16_t servo_value = DL_SET_ACTUATOR_value(dl_buffer);
    LED_TOGGLE(2);
    if (servo_no < ACTUATORS_NB)
      //SetServo(servo_no, servo_value);
  }
#ifdef DlSetting
  else if (msg_id == DL_SETTING && DL_SETTING_ac_id(dl_buffer) == AC_ID) {
    uint8_t i = DL_SETTING_index(dl_buffer);
    float val = DL_SETTING_value(dl_buffer);
    DlSetting(i, val);
    LED_TOGGLE(2);
    for (int j=0 ; j<8 ; j++) {
      //SetServo(j,actuators[j]);
    }
    DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
  } else if (msg_id == DL_GET_SETTING && DL_GET_SETTING_ac_id(dl_buffer) == AC_ID) {
    uint8_t i = DL_GET_SETTING_index(dl_buffer);
    float val = settings_get_value(i);
    DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
  }
#endif
}


void init_fbw( void ) {
  mcu_init();

  actuators_init();

  uint8_t i;
  for(i = 0; i < ACTUATORS_NB; i++) {
    //SetServo(i, 1500);
  }

  //  SetServo(SERVO_GAZ, SERVO_GAZ_MIN);

  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);

  mcu_int_enable();
}

void handle_periodic_tasks_fbw(void) {

  if (sys_time_check_and_ack_timer(0))
    periodic_task_fbw();

}

void periodic_task_fbw(void) {
   /* static float t; */
   /* t += 1./60.; */
   /* uint16_t servo_value = 1500+ 500*sin(t); */
   /* SetServo(SERVO_THROTTLE, servo_value); */

  RunOnceEvery(300, DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM));
  RunOnceEvery(300, DOWNLINK_SEND_ACTUATORS(DefaultChannel, DefaultDevice, ACTUATORS_NB, actuators ));
}

void event_task_fbw(void) {
  DatalinkEvent();
}
