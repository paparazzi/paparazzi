
#include "subsystems/sensors/baro.h"

struct Baro baro;

void baro_init(void) { }

void baro_periodic(void) {baro.status = BS_RUNNING;}

void baro_feed_value(double value) {
  baro.absolute = (int32_t) value;
}
