#include "booz_flight_model.h"


#define DT_FDM  (1./250.)
#define NB_ITER 100

int main(int argc, char** argv) {

  double actuators_values[] = {0.6, 0.6, 0.6, 0.6};
  double time = 0.;

  booz_flight_model_init();

  bfm.on_ground = FALSE;

  int i;
  for (i=0; i<NB_ITER; i++) {
    booz_flight_model_run(DT_FDM, actuators_values);
    time += DT_FDM;
    printf("%f \t %f\n", time, bfm.state->ve[BFMS_Z]);
  }

  return 0;
}
