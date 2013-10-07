
/*
 * board specific fonctions for the PX4FMU board
 *
 */

#ifndef BOARDS_PX4FMU_BARO_H
#define BOARDS_PX4FMU_BARO_H

extern void baro_event(void (*b_abs_handler)(void));

#define BaroEvent(_b_abs_handler, _b_diff_handler) baro_event(_b_abs_handler)

#endif /* BOARDS_PX4FMU_BARO_H */
