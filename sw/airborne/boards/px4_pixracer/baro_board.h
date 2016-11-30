
/*
 * board specific functions for the px4_Pixracer_v1.0 board
 *
 */

#ifndef BOARDS_PIXRACER_BARO_H
#define BOARDS_PIXRACER_BARO_H

#include "std.h"

// only for printing the baro type during compilation
#define BARO_BOARD BARO_MS5611_SPI

extern void baro_event(void);

#define BaroEvent baro_event

#endif /* BOARDS_PIXRACER_BARO_H */
