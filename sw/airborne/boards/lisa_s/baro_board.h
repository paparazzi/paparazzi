
/*
 * board specific functions for the lisa_s board
 *
 */

#ifndef BOARDS_LISA_S_BARO_H
#define BOARDS_LISA_S_BARO_H

#include "std.h"

// only for printing the baro type during compilation
#define BARO_BOARD BARO_MS5611_SPI

extern void baro_event(void);

#define BaroEvent baro_event

#endif /* BOARDS_LISA_S_BARO_H */
