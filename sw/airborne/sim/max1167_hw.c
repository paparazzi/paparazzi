#include "max1167.h"

#include "../../simulator/booz_sensors_model.h"

void max1167_hw_init( void ) {}

void max1167_read( void ) {
  if (max1167_status == STA_MAX1167_IDLE) {
    max1167_values[0] = bsm.gyro->ve[AXIS_P];
    max1167_values[1] = bsm.gyro->ve[AXIS_Q];
    max1167_values[2] = bsm.gyro->ve[AXIS_R];
    max1167_status = STA_MAX1167_DATA_AVAILABLE;
  }
  else {
    /* report overrun error */
  }
}
