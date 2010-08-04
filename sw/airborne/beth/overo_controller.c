#include "overo_controller.h"

#include "overo_estimator.h"

struct OveroController controller;

void control_init(void) {
  controller.kp = 0.05;
  controller.kd = 0.01;

  controller.tilt_sp = 0.;

  controller.cmd = 0.;
}



void control_run(void) {
  
  float track_err = estimator.tilt - controller.tilt_sp;
  
  controller.cmd = controller.kp*track_err + controller.kd*estimator.tilt_dot;
  
}

