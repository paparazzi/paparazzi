
/*
 * board specific fonctions for the lisa_l board
 *
 */

#ifndef BOARDS_LISA_L_BARO_H
#define BOARDS_LISA_L_BARO_H

#include "std.h"
#include "mcu_periph/i2c.h"

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
};

extern struct BaroBoard baro_board;
extern struct i2c_transaction baro_trans;

extern void baro_downlink_raw(void);

extern void baro_board_send_reset(void);
extern void baro_board_send_config_abs(void);
extern void baro_board_send_config_diff(void);

#define BaroEvent(_b_abs_handler, _b_diff_handler) {			\
    if (baro_board.status == LBS_READING_ABS &&                 \
        baro_trans.status != I2CTransPending) {                 \
      baro_board.status = LBS_READ_ABS;                         \
      if (baro_trans.status == I2CTransSuccess) {               \
        int16_t tmp = baro_trans.buf[0]<<8 | baro_trans.buf[1]; \
        baro.absolute = tmp;                                    \
        _b_abs_handler();                                       \
      }                                                         \
    }                                                           \
    else  if (baro_board.status == LBS_READING_DIFF &&			\
              baro_trans.status != I2CTransPending) {			\
      baro_board.status = LBS_READ_DIFF;                        \
      if (baro_trans.status == I2CTransSuccess) {               \
      	int16_t tmp = baro_trans.buf[0]<<8 | baro_trans.buf[1]; \
      	baro.differential = tmp;                                \
      	_b_diff_handler();                                      \
      }                                                         \
    }                                                           \
  }




#endif /* BOARDS_LISA_L_BARO_H */
