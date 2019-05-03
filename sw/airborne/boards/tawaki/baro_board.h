
/*
 * board specific functions for the tawaki board
 *
 */

#ifndef BOARDS_TAWAKI_BARO_H
#define BOARDS_TAWAKI_BARO_H

// only for printing the baro type during compilation
#ifndef BARO_BOARD
#define BARO_BOARD BARO_BMP3_I2C
#endif

extern void baro_event(void);
#define BaroEvent baro_event

#endif /* BOARDS_TAWAKI_BARO_H */

