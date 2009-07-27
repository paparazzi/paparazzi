#include "booz_flight_model.h"
#include "booz_sensors_model.h"


#define DT_FDM  (1./250.)
#define NB_ITER 100

int main(int argc, char** argv) {

  double actuators_values[] = {0.6, 0.6, 0.6, 0.6};
  double time = 0.;

  booz_flight_model_init();
  booz_sensors_model_init(time);

  bfm.on_ground = FALSE;

  int i;
  for (i=0; i<NB_ITER; i++) {
    booz_flight_model_run(DT_FDM, actuators_values);
    time += DT_FDM;
    booz_sensors_model_run(time);
    //    if (booz_sensors_model_baro_available())
    //      printf("%f \t %f\n", time, bsm.baro);
    //    if (booz_sensors_model_gps_available())
    //      printf("%f \t %f \t %f\n", time, bsm.gps_pos->ve[AXIS_Z], bsm.gps_speed->ve[AXIS_Z]);
    if (booz_sensors_model_accel_available())
      printf("%f \t %f \t %f \t %f\n", time, bsm.accel->ve[AXIS_X], bsm.accel->ve[AXIS_Y], bsm.accel->ve[AXIS_Z]);
  }

  return 0;
}
