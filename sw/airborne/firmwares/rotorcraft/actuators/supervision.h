#ifndef BOOZ_SUPERVISION_H
#define BOOZ_SUPERVISION_H

#include "std.h"
#include "airframe.h"

struct BoozSupervision {
  int32_t commands[SUPERVISION_NB_MOTOR];
  int32_t trim[SUPERVISION_NB_MOTOR];
  bool_t override_enabled[SUPERVISION_NB_MOTOR];
  int32_t override_value[SUPERVISION_NB_MOTOR];
  uint32_t nb_failure;
};

extern struct BoozSupervision supervision;

extern void supervision_init(void);
extern void supervision_run(bool_t motors_on, bool_t override_on, int32_t in_cmd[]);
extern void supervision_run_spinup(uint32_t counter, uint32_t max_counter);

#endif /* BOOZ_SUPERVISION_H */
