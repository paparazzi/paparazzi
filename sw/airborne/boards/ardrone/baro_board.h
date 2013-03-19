/*
 * Copyright (C) 2012 TU Delft Quatrotor Team 1
 *
 * Paparazzi AR Drone 2 Baro Sensor implementation:
 * These functions are mostly empty because of the calibration and calculations done by the Parrot Navigation board
 */

#ifndef BOARDS_ARDRONE2_BARO_H
#define BOARDS_ARDRONE2_BARO_H

#include "navdata.h"

int baro_data_available;

static inline void baro_event(void (*b_abs_handler)(void), void (*b_diff_handler)(void)){
  if (baro_data_available) {
    b_abs_handler();
  }
}

#define BaroEvent(_b_abs_handler, _b_diff_handler) {\
  baro_event(_b_abs_handler,_b_diff_handler);\
}

#endif /* BOARDS_ARDRONE2_BARO_H */
