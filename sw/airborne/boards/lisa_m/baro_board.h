
/*
 * board specific functions for the lisa_m board
 *
 */

#ifndef BOARDS_LISA_M_BARO_H
#define BOARDS_LISA_M_BARO_H

#include "std.h"

// for right now we abuse this file for the ms5611 baro on aspirin as well
#if !BARO_MS5611_I2C && !BARO_MS5611

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

#endif  // !BARO_MS5611_xx

extern void baro_event(void (*b_abs_handler)(void), void (*b_diff_handler)(void));

#define BaroEvent(_b_abs_handler, _b_diff_handler) baro_event(_b_abs_handler,_b_diff_handler)

#endif /* BOARDS_LISA_M_BARO_H */
