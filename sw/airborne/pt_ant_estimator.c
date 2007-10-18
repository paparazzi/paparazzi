#include "pt_ant_estimator.h"

struct PtAntEstimatorState pt_ant_estimator_state;

void pt_ant_estimator_update_target(uint8_t id, struct ac_info_ *ac) {
  pt_ant_estimator_state.target_east = ac->east;
  pt_ant_estimator_state.target_north = ac->north;
  pt_ant_estimator_state.target_alt = ac->alt;
}

void pt_ant_estimator_update_self_attitude(struct PtAntSensorData* data) {
  pt_ant_estimator_state.self_phi = atan2(data->accel_y, data->accel_z);
  float g2 = data->accel_x * data->accel_x +
             data->accel_y * data->accel_y +
             data->accel_z * data->accel_z;
  pt_ant_estimator_state.self_theta = -asin(data->accel_x / sqrt(g2));

}

void pt_ant_estimator_update_self_gps(void) {


}
