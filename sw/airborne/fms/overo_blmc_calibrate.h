#ifndef OVERO_BLMC_CALIBRATE_H
#define OVERO_BLMC_CALIBRATE_H

#include "std.h"
#include "fms/fms_autopilot_msg.h"

struct OveroBLMCCalibrate {
  /* our actuators          */
  uint16_t servos_outputs_usecs[LISA_PWM_OUTPUT_NB]; /* FIXME */
};

extern struct OveroBLMCCalibrate blmc_calibrate;

#endif /* OVERO_BLMC_CALIBRATE_H */
