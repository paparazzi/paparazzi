#ifndef PT_ANT_ESTIMATOR_H
#define PT_ANT_ESTIMATOR_H

#include "std.h"
#include "traffic_info.h"
#include "pt_ant_sensors.h"


struct PtAntEstimatorState {

  float self_phi;
  float self_theta;
  float self_psi;

  float self_east;
  float self_north;
  float self_alt;

  float target_east;
  float target_north;
  float target_alt;

};

extern struct PtAntEstimatorState pt_ant_estimator_state;

extern void pt_ant_estimator_update_target(uint8_t id, struct ac_info_ *ac);
extern void pt_ant_estimator_update_self_gps(void);
extern void pt_ant_estimator_update_self_attitude(struct PtAntSensorData* data);


#endif /* PT_ANT_ESTIMATOR_H */
