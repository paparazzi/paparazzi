
/*
 * board specific interface for the KroozSD board
 *
 * It uses the subsystems/sensors/baro_ms5611_i2c.c driver
 */

#ifndef BOARDS_KROOZ_BARO_H
#define BOARDS_KROOZ_BARO_H

extern void baro_event(void);
#define BaroEvent baro_event

#endif /* BOARDS_KROOZ_SD_BARO_H */
