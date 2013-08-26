
/*
 * board specific functions for the lisa_m board
 *
 */

#ifndef BOARDS_LISA_M_BARO_H
#define BOARDS_LISA_M_BARO_H

#include "std.h"

// for right now we abuse this file for the ms5611 baro on aspirin as well
#if !BARO_MS5611_I2C && !BARO_MS5611

#include "peripherals/bmp085.h"

extern struct Bmp085 baro_bmp085;


#endif  // !BARO_MS5611_xx

extern void baro_event(void (*b_abs_handler)(void));

#define BaroEvent(_b_abs_handler, _b_diff_handler) baro_event(_b_abs_handler)

#endif /* BOARDS_LISA_M_BARO_H */
