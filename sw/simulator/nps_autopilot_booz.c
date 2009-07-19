#include "nps_autopilot.h"

#include "booz2_main.h"
#include "nps_sensors.h"
#include "nps_radio_control.h"

struct NpsAutopilot autopilot;


void nps_autopilot_init(void) {

  nps_radio_control_init();

  booz2_main_init();

}


void nps_autopilot_run_step(double time __attribute__ ((unused))) {
  
#if 0
  if (nps_radio_control_available(time)) {
    booz2_radio_control_feed();
    booz2_main_event();
  }
#endif

  if (nps_sensors_gyro_available()) {
    //    booz2_imu_feed_data();
    //    booz2_main_event();
  }


  booz2_main_periodic();

  //  double hover = 0.25;
  double hover = 0.2493;
  //   double hover = 0.23;
  //  double hover = 0.;
  //  if (time > 20) hover = 0.25;

  autopilot.commands[SERVO_FRONT] = hover;
  autopilot.commands[SERVO_BACK]  = hover;
  autopilot.commands[SERVO_RIGHT] = hover;
  autopilot.commands[SERVO_LEFT]  = hover;

}
