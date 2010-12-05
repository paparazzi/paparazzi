#ifndef BOARDS_BOOZ_BARO_H
#define BOARDS_BOOZ_BARO_H

#include "std.h"

#include "subsystems/sensors/baro.h"
#include "booz/booz2_analog.h"

/* we don't need that on this board */

struct BaroBoard {
  uint16_t offset;
  uint16_t value_filtered;
  bool_t   data_available;
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
  Booz2AnalogSetDAC(_o);
}

static inline void BoozBaroISRHandler(uint16_t _val) {
  baro.absolute = _val;
  baro_board.value_filtered = (3*baro_board.value_filtered + baro.absolute)/4;
  if (baro.status == BS_UNINITIALIZED) {
    RunOnceEvery(10, { baro_board_calibrate();});
  }
  /*  else */
  baro_board.data_available = TRUE;
}




#endif /* BOARDS_BOOZ_BARO_H */
