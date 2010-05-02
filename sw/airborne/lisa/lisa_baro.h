#ifndef LISA_BARO_H
#define LISA_BARO_H

#include <inttypes.h>

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

struct LisaBaro {
  int16_t abs_raw;
  int16_t diff_raw;
  enum LisaBaroStatus status;
  uint8_t i2c_done;
};

extern struct LisaBaro baro;

extern void baro_init(void);
extern void baro_periodic(void);

extern void baro_send_reset(void);
extern void baro_send_config_abs(void);
extern void baro_send_config_diff(void);


#define BaroEvent(_b_abs_handler, _b_diff_handler) {			\
    if (baro.status == LBS_READING_ABS && baro.i2c_done) {		\
      baro.abs_raw = i2c2.buf[0]<<8 | i2c2.buf[1];			\
      baro.status = LBS_READ_ABS;					\
      _b_abs_handler();							\
    }									\
    else  if (baro.status == LBS_READING_DIFF && baro.i2c_done) {	\
      baro.diff_raw = i2c2.buf[0]<<8 | i2c2.buf[1];			\
      baro.status = LBS_READ_DIFF;					\
      _b_diff_handler();						\
    }									\
  }


#endif /* LISA_BARO_H */


