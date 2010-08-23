

#include "firmwares/rotorcraft/baro.h"

struct Baro baro;

bool_t baro_pc_available;

void baro_init(void) {baro_pc_available=FALSE;}

void baro_periodic(void) {}

void baro_feed_value(double value) {
  baro.absolute = value;
  baro_pc_available = TRUE;
}
