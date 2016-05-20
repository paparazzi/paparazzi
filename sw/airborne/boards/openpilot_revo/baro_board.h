/*
 * board specific functions for the openpilot_revo board
 *
 */

#ifndef BOARDS_OPENPILOT_REVO_BARO_H
#define BOARDS_OPENPILOT_REVO_BARO_H

// only for printing the baro type during compilation
#ifndef BARO_BOARD
#define BARO_BOARD BARO_MS5611_I2C
#endif

extern void baro_event(void);
#define BaroEvent baro_event

#endif /* BOARDS_OPENPILOT_REVO_BARO_H */
