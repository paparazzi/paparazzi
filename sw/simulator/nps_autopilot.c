#include "nps_autopilot.h"

#include "booz2_main.h"
#include "nps_sensors.h"

struct NpsAutopilot autopilot;


void nps_autopilot_init(void) {

  /* Just for testing fdm */
  double hover = 0.2495;

  autopilot.commands[SERVO_FRONT] = hover;
  autopilot.commands[SERVO_BACK]  = hover;
  autopilot.commands[SERVO_RIGHT] = hover;
  autopilot.commands[SERVO_LEFT]  = hover;
  
  booz2_main_init();

}


void nps_autopilot_run_step(double time __attribute__ ((unused))) {
  double hover = 0.2493;
  autopilot.commands[SERVO_FRONT] = hover;
  autopilot.commands[SERVO_BACK]  = hover;
  autopilot.commands[SERVO_RIGHT] = hover;
  autopilot.commands[SERVO_LEFT]  = hover;
  
  if (nps_sensors_gyro_available()) {
    //    booz2_imu_feed_data();
    //    booz2_main_event();
  }


  booz2_main_periodic();


}
