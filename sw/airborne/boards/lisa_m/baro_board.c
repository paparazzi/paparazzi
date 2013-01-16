/**
 *  Generic barometer interface, assuming the barometer is read through Aspirin IMU directly
 *
 * Edit by: Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
 * Utah State University, http://aggieair.usu.edu/
 */

#include "subsystems/sensors/baro.h"
#include "baro_board.h"

struct Baro baro;

void baro_init(void) {
  baro.status = BS_UNINITIALIZED;
  baro.absolute     = 0;
  baro.differential = 0;
}

void baro_periodic(void) {}

void baro_event(void (*b_abs_handler)(void), void (*b_diff_handler)(void)){
  b_abs_handler();
  b_diff_handler();
}
