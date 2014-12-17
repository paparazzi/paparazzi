/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef BOOZ_FMS_TEST_SIGNAL_H
#define BOOZ_FMS_TEST_SIGNAL_H

#include "std.h"


enum fms_ts_mode {
  STEP_ROLL,
  STEP_PITCH,
  STEP_YAW,
  STEP_VERT
};

struct BoozFmsTestSignal {
  enum fms_ts_mode mode;
  uint32_t period;
  uint32_t amplitude;
  uint32_t counter;
};

extern struct BoozFmsTestSignal fms_test_signal;

#define booz_fms_test_signal_SetPeriod(_val) {    \
    fms_test_signal.period = _val;      \
    fms_test_signal.counter = 0;            \
  }

#define booz_fms_test_signal_SetMode(_val) {    \
    fms_test_signal.mode = (enum fms_ts_mode)(_val);  \
    fms_test_signal.counter = 0;      \
  }

#endif /* BOOZ_FMS_TEST_SIGNAL_H */

