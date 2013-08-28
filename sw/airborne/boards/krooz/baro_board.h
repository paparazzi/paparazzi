
/*
 * board specific interface for the KroozSD board
 *
 * It uses the subsystems/sensors/baro_ms5611_i2c.c driver
 */

#ifndef BOARDS_KROOZ_BARO_H
#define BOARDS_KROOZ_BARO_H

extern void baro_event(void (*b_abs_handler)(void));
#define BaroEvent(_b_abs_handler, _b_diff_handler) baro_event(_b_abs_handler)

#endif /* BOARDS_KROOZ_SD_BARO_H */
