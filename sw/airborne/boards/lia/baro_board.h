
/*
 * FIXME, fake baro board header for BaroEvent
 *
 */

#ifndef BOARDS_LIA_BARO_H
#define BOARDS_LIA_BARO_H

#include "std.h"

extern void baro_event(void (*b_abs_handler)(void));

#define BaroEvent(_b_abs_handler, _b_diff_handler) baro_event(_b_abs_handler)

#endif /* BOARDS_LIA_BARO_H */
