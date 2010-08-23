/*
 * board specific fonction for the PC board ( simulator )
 *
 */

#ifndef BOARDS_PC_BARO_H
#define BOARDS_PC_BARO_H

#define BaroEvent(_b_abs_handler, _b_diff_handler) { \
    if (baro_pc_available) {			     \
      _b_abs_handler();				     \
      baro_pc_available = FALSE;		     \
    }						     \
  }


extern bool_t baro_pc_available;

extern void baro_feed_value(double value);

#endif /* BOARDS_PC_BARO_H */
