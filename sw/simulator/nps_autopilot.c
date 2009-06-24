#include "nps_autopilot.h"

struct NpsAutopilot autopilot;

#include "nps_sensors.h"

void nps_autopilot_init(void) {

  /* Just for testing fdm */

  autopilot.commands[0] = 0.5;
  autopilot.commands[1] = 0.5;
  autopilot.commands[2] = 0.5;
  autopilot.commands[3] = 0.5;

}


void nps_autopilot_run_step(void) {

  if (nps_sensors_gyro_available()) {
    //    booz2_imu_feed_data();
    //    booz2_main_event();
  }

}
