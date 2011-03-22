#ifndef BOARDS_BOOZ_BARO_H
#define BOARDS_BOOZ_BARO_H

#include "std.h"

#include "subsystems/sensors/baro.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/dac.h"


struct BaroBoard {
  uint16_t offset;
  uint16_t value_filtered;
  bool_t   data_available;
  struct adc_buf buf;
};

extern struct BaroBoard baro_board;

extern void baro_board_calibrate(void);

#define BaroEvent(_b_abs_handler, _b_diff_handler) {	\
    if (baro_board.data_available) {			\
      _b_abs_handler();					\
      baro_board.data_available = FALSE;		\
    }							\
  }

static inline void baro_board_SetOffset(uint16_t _o) {
  baro_board.offset = _o;
  DACSet(_o);
}


#endif /* BOARDS_BOOZ_BARO_H */
