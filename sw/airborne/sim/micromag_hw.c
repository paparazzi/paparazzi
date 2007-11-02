#include "micromag.h"

#include "../../simulator/booz_sensors_model.h"

void micromag_hw_init( void ) {}


void micromag_read( void ) {
  micromag_values[0] =  bsm.mag->ve[AXIS_X];
  micromag_values[1] =  bsm.mag->ve[AXIS_Y];
  micromag_values[2] =  bsm.mag->ve[AXIS_Z];
}
