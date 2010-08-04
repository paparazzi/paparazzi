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
  static int foo=0;

  float track_err = estimator.tilt - controller.tilt_sp;
  float pcmd = controller.kp*track_err;
  float dcmd = controller.kd*estimator.tilt_dot;
  //controller.cmd = controller.kp*track_err + controller.kd*estimator.tilt_dot;
  controller.cmd = pcmd + dcmd;
  //if (!(foo%100)) printf("%f %f\n",pcmd,dcmd);
  foo++;
}

