
/*
 * board specific functions for the lisa_m board
 *
 */

#ifndef BOARDS_LISA_M_BARO_H
#define BOARDS_LISA_M_BARO_H

#include "std.h"

extern void baro_event(void (*b_abs_handler)(void));

#define BaroEvent(_b_abs_handler, _b_diff_handler) baro_event(_b_abs_handler)

#endif /* BOARDS_LISA_M_BARO_H */
