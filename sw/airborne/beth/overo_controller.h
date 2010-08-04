#ifndef OVERO_CONTROLLER_H
#define OVERO_CONTROLLER_H

struct OveroController {
  float kp;
  float kd;
  
  float tilt_sp;

  float cmd;
};


extern struct OveroController controller;

extern void control_init(void);
extern void control_run(void);

#endif /* OVERO_CONTROLLER_H */
