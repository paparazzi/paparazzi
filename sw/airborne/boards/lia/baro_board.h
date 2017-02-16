
/*
 * board specific functions for the lia board
 *
 */

#ifndef BOARDS_LIA_BARO_H
#define BOARDS_LIA_BARO_H

// only for printing the baro type during compilation
#ifndef BARO_BOARD
#define BARO_BOARD BARO_BOARD_BMP085
#endif

extern void baro_event(void);
#define BaroEvent baro_event

#endif /* BOARDS_LIA_BARO_H */
