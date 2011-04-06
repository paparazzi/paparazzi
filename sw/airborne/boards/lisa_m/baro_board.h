
/*
 * board specific fonctions for the lisa_m board
 *
 */

#ifndef BOARDS_LISA_M_BARO_H
#define BOARDS_LISA_M_BARO_H

#include "std.h"
#include "mcu_periph/i2c.h"

// absolute addr
#define BMP085_ADDR  0xEE
// Over sample setting (0-3)
#define BMP085_OSS 3

enum LisaBaroStatus {
  LBS_UNINITIALIZED,
  LBS_REQUEST,
  LBS_READING,
  LBS_READ,
  LBS_REQUEST_TEMP,
  LBS_READING_TEMP,
  LBS_READ_TEMP,
};

struct BaroBoard {
  enum LisaBaroStatus status;
};

struct bmp085_baro_calibration {
  int16_t ac1;
  int16_t ac2;
  int16_t ac3;
  uint16_t ac4;
  uint16_t ac5;
  uint16_t ac6;
  int16_t b1;
  int16_t b2;
  int16_t mb;
  int16_t mc;
  int16_t md;
};

extern struct BaroBoard baro_board;
extern struct i2c_transaction baro_trans;

extern void baro_board_send_reset(void);
extern void baro_board_send_config(void);

static inline void baro_event(void (*b_abs_handler)(void), void (*b_diff_handler)(void))
{
  if (baro_board.status == LBS_READING &&
      baro_trans.status != I2CTransPending && baro_trans.status != I2CTransRunning) {
    baro_board.status = LBS_REQUEST_TEMP;
    if (baro_trans.status == I2CTransSuccess) {
      int32_t tmp = (baro_trans.buf[0]<<16) | (baro_trans.buf[1] << 8) | baro_trans.buf[2];
      baro.absolute = tmp >> ( 8 - BMP085_OSS);
      b_abs_handler();
    }
  }
  if (baro_board.status == LBS_READING_TEMP &&
      baro_trans.status != I2CTransPending && baro_trans.status != I2CTransRunning) {
    baro_board.status = LBS_REQUEST;
    if (baro_trans.status == I2CTransSuccess) {
      int32_t tmp = (baro_trans.buf[0] << 8) | baro_trans.buf[1];
      baro.differential = tmp;
      b_diff_handler();
    }
  }
}

#define BaroEvent(_b_abs_handler, _b_diff_handler) baro_event(_b_abs_handler,_b_diff_handler)

#endif /* BOARDS_LISA_M_BARO_H */
