
/*
 * board specific fonctions for the KroozSD board
 *
 */

#ifndef BOARDS_KROOZ_BARO_H
#define BOARDS_KROOZ_BARO_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "modules/sensors/baro_ms5611_i2c.h"
#include "math/pprz_algebra_int.h"

//#include "led.h"

static inline void baro_event(void (*b_abs_handler)(void), void (*b_diff_handler)(void))
{
  baro_ms5611_event();
  if(baro_ms5611_valid) {
    baro.status = BS_RUNNING;
    baro.absolute = (int32_t)baroms;
    b_abs_handler();
    baro_ms5611_valid = FALSE;
  }
}

#define BaroEvent(_b_abs_handler, _b_diff_handler) baro_event(_b_abs_handler,_b_diff_handler)

#endif /* BOARDS_KROOZ_SD_BARO_H */
