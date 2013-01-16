/**
 *  Generic barometer interface, assuming the barometer is read through Aspirin IMU directly
 *
 * Edit by: Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
 * Utah State University, http://aggieair.usu.edu/
 */

#ifndef BOARDS_LIA_BARO_H
#define BOARDS_LIA_BARO_H

extern void baro_event(void (*b_abs_handler)(void), void (*b_diff_handler)(void));

#define BaroEvent(_b_abs_handler, _b_diff_handler) baro_event(_b_abs_handler,_b_diff_handler)

#endif /* BOARDS_LIA_BARO_H */
