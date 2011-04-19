
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
  // These values come from EEPROM on sensor
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
  
  // These values are calculated
  int32_t b5;
};

extern struct BaroBoard baro_board;
extern struct i2c_transaction baro_trans;
extern struct bmp085_baro_calibration calibration;

extern void baro_board_send_reset(void);
extern void baro_board_send_config(void);

// Apply temp calibration and sensor calibration to raw measurement to get Pa (from BMP085 datasheet)
static inline int32_t baro_apply_calibration(int32_t raw)
{
  int32_t b6 = calibration.b5 - 4000;
  int x1 = (calibration.b2 * (b6 * b6 >> 12)) >> 11;
  int x2 = calibration.ac2 * b6 >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = (((calibration.ac1 * 4 + x3) << BMP085_OSS) + 2)/4;
  x1 = calibration.ac3 * b6 >> 13;
  x2 = (calibration.b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = (calibration.ac4 * (uint32_t) (x3 + 32768)) >> 15;
  uint32_t b7 = (raw - b3) * (50000 >> BMP085_OSS);
  int32_t p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  return p + ((x1 + x2 + 3791) >> 4);
}

static inline void baro_event(void (*b_abs_handler)(void), void (*b_diff_handler)(void))
{
  if (baro_board.status == LBS_READING &&
      baro_trans.status != I2CTransPending && baro_trans.status != I2CTransRunning) {
    baro_board.status = LBS_REQUEST_TEMP;
    if (baro_trans.status == I2CTransSuccess) {
      int32_t tmp = (baro_trans.buf[0]<<16) | (baro_trans.buf[1] << 8) | baro_trans.buf[2];
      tmp = tmp >> (8 - BMP085_OSS);
      baro.absolute = baro_apply_calibration(tmp);
      b_abs_handler();
    }
  }
  if (baro_board.status == LBS_READING_TEMP &&
      baro_trans.status != I2CTransPending && baro_trans.status != I2CTransRunning) {
    baro_board.status = LBS_REQUEST;
    if (baro_trans.status == I2CTransSuccess) {
      // abuse differential to store temp in 0.1C for now
      int32_t tmp = (baro_trans.buf[0] << 8) | baro_trans.buf[1];
      int32_t x1 = ((tmp - calibration.ac6) * calibration.ac5) >> 15;
      int32_t x2 = (calibration.mc << 11) / (x1 + calibration.md);
      calibration.b5 = x1 + x2;
      baro.differential = (calibration.b5 + 8) >> 4;
      b_diff_handler();
    }
  }
}

#define BaroEvent(_b_abs_handler, _b_diff_handler) baro_event(_b_abs_handler,_b_diff_handler)

#endif /* BOARDS_LISA_M_BARO_H */
