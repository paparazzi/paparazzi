#ifndef BOARDS_LISA_L_BARO_H
#define BOARDS_LISA_L_BARO_H

#include "std.h"
#include "i2c.h"

enum LisaBaroStatus {
  LBS_UNINITIALIZED, 
  LBS_RESETED, 
  LBS_INITIALIZING_ABS, 
  LBS_INITIALIZING_ABS_1, 
  LBS_INITIALIZING_DIFF, 
  LBS_INITIALIZING_DIFF_1, 
  LBS_IDLE,
  LBS_READING_ABS,
  LBS_READ_ABS,
  LBS_READING_DIFF,
  LBS_READ_DIFF
};

struct BaroBoard {
  enum LisaBaroStatus status;
  bool_t i2c_done;
};

extern struct BaroBoard baro_board;

extern void baro_board_send_reset(void);
extern void baro_board_send_config_abs(void);
extern void baro_board_send_config_diff(void);

#define BaroEvent(_b_abs_handler, _b_diff_handler) {			\
    if (baro_board.status == LBS_READING_ABS && baro_board.i2c_done) {	\
      int16_t tmp = i2c2.buf[0]<<8 | i2c2.buf[1];			\
      baro.absolute = tmp;						\
      baro_board.status = LBS_READ_ABS;					\
      _b_abs_handler();							\
    }									\
    else  if (baro_board.status == LBS_READING_DIFF && baro_board.i2c_done) { \
      int16_t tmp = i2c2.buf[0]<<8 | i2c2.buf[1];			\
      baro.differential = tmp;						\
      baro_board.status = LBS_READ_DIFF;				\
      _b_diff_handler();						\
    }									\
  }




#endif /* BOARDS_LISA_L_BARO_H */
