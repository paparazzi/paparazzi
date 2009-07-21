#include "nps_autopilot.h"

#include "booz2_main.h"
#include "nps_sensors.h"
#include "nps_radio_control.h"
#include "booz_radio_control.h"
#include "booz_imu.h"
#include "booz2_analog_baro.h"

#include "actuators.h"

struct NpsAutopilot autopilot;


void nps_autopilot_init(void) {

  nps_radio_control_init();

  booz2_main_init();

}

#include <stdio.h>

void nps_autopilot_run_step(double time __attribute__ ((unused))) {
  
  if (nps_radio_control_available(time)) {
    booz_radio_control_feed();
    booz2_main_event();
  }

  if (nps_sensors_gyro_available()) {
    booz_imu_feed_gyro_accel();
    booz2_main_event();
  }

  if (nps_sensors_mag_available()) {
    //    printf("in mag %f %f %f\n", sensors.mag.value.x, sensors.mag.value.y, sensors.mag.value.z);
    booz_imu_feed_mag();
    booz2_main_event();
 }

  if (nps_sensors_baro_available()) {
    Booz2BaroISRHandler(sensors.baro.value);
    booz2_main_event();
  }

  booz2_main_periodic();


  if (time < 5) {
    double hover = 0.25;
    //double hover = 0.2493;
    //   double hover = 0.23;
    //  double hover = 0.;
    //  if (time > 20) hover = 0.25;
    
    autopilot.commands[SERVO_FRONT] = hover;
    autopilot.commands[SERVO_BACK]  = hover;
    autopilot.commands[SERVO_RIGHT] = hover;
    autopilot.commands[SERVO_LEFT]  = hover;
  }
  else {
    int32_t ut_front = Actuator(SERVO_FRONT) - TRIM_FRONT;
    int32_t ut_back  = Actuator(SERVO_BACK)  - TRIM_BACK;
    int32_t ut_right = Actuator(SERVO_RIGHT) - TRIM_RIGHT;
    int32_t ut_left  = Actuator(SERVO_LEFT)  - TRIM_LEFT;
    autopilot.commands[SERVO_FRONT] = (double)ut_front / SUPERVISION_MAX_MOTOR;
    autopilot.commands[SERVO_BACK]  = (double)ut_back  / SUPERVISION_MAX_MOTOR;
    autopilot.commands[SERVO_RIGHT] = (double)ut_right / SUPERVISION_MAX_MOTOR;
    autopilot.commands[SERVO_LEFT]  = (double)ut_left  / SUPERVISION_MAX_MOTOR;
  }
  printf("%f %f %f %f\n", autopilot.commands[SERVO_FRONT], autopilot.commands[SERVO_BACK],
	 autopilot.commands[SERVO_RIGHT], autopilot.commands[SERVO_LEFT]);	 

}
