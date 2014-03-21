
/*
 * board specific functions for the lisa_m board
 *
 */

#ifndef BOARDS_LISA_M_BARO_H
#define BOARDS_LISA_M_BARO_H

// only for printing the baro type during compilation
#ifndef BARO_BOARD
#define BARO_BOARD BARO_BOARD_BMP085
#endif

// for right now we abuse this file for the ms5611 baro on aspirin as well

extern void baro_event(void);
#define BaroEvent baro_event

#endif /* BOARDS_LISA_M_BARO_H */
