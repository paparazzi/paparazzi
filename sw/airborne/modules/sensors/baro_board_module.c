
//FIXME: this is just a kludge
// Gautier plz commit the real one

#include "baro_board_module.h"

struct Baro baro;

void baro_init(void) {
  baro.status = BS_UNINITIALIZED;
  baro.absolute     = 0;
  baro.differential = 0;
}

void baro_periodic(void) {
  if (baro.absolute != 0)
    baro.status = BS_RUNNING;
}
