#include "nps_autopilot.h"

struct NpsAutopilot autopilot;

#include "nps_sensors.h"

void nps_autopilot_init(void) {

  /* Just for testing fdm */
  double hover = 0.247;

  autopilot.commands[0] = hover;
  autopilot.commands[1] = hover;
  autopilot.commands[2] = hover;
  autopilot.commands[3] = hover;

}


void nps_autopilot_run_step(void) {

  if (nps_sensors_gyro_available()) {
    //    booz2_imu_feed_data();
    //    booz2_main_event();
  }

}
