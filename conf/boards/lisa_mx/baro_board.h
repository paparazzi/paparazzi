
/*
 * board specific functions for the lisa_mx board
 *
 */
//TODO: clean up - as not really necessary for ChibiOS
#ifndef BOARDS_LISA_MX_BARO_H
#define BOARDS_LISA_MX_BARO_H

extern void baro_event(void);
#define BaroEvent baro_event

#endif /* BOARDS_LISA_MX_BARO_H */
