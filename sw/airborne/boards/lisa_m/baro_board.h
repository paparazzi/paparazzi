
/*
 * board specific fonctions for the lisa_m board
 *
 */

#ifndef BOARDS_LISA_M_BARO_H
#define BOARDS_LISA_M_BARO_H

#include "std.h"
#include "mcu_periph/i2c.h"

// absolute
#define BMP085_ADDR  0xEE
#define BMP085_OSS 3

enum LisaBaroStatus {
  LBS_UNINITIALIZED,
  LBS_RESETED,
  LBS_INITIALIZING,
  LBS_INITIALIZING_1,
  LBS_IDLE,
  LBS_REQUEST,
  LBS_READING,
  LBS_READ,
};

struct BaroBoard {
  enum LisaBaroStatus status;
};

extern struct BaroBoard baro_board;
extern struct i2c_transaction baro_trans;

extern void baro_board_send_reset(void);
extern void baro_board_send_config(void);

#define BaroEvent(_b_abs_handler, _b_diff_handler) {			\
    if (baro_board.status == LBS_READING &&				\
	baro_trans.status != I2CTransPending) {				\
      baro_board.status = LBS_REQUEST;					\
			if (baro_trans.status == I2CTransSuccess) { \
				int32_t tmp = (baro_trans.buf[0]<<16) | (baro_trans.buf[1] << 8) | baro_trans.buf[0];		\
				baro.absolute = tmp >> ( 8 - BMP085_OSS);						\
				_b_abs_handler();							\
			} \
    }									\
  }




#endif /* BOARDS_LISA_M_BARO_H */
