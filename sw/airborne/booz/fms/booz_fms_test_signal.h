/*
 * $Id$
 *  
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

#define BOOZ_FMS_TEST_SIGNAL_MODE_ATTITUDE 0
#define BOOZ_FMS_TEST_SIGNAL_MODE_VERTICAL 1

extern uint8_t  booz_fms_test_signal_mode;

extern uint32_t booz_fms_test_signal_period;
extern uint32_t booz_fms_test_signal_amplitude;
extern uint8_t  booz_fms_test_signal_axe;
extern uint32_t booz_fms_test_signal_counter;

extern void booz_fms_impl_init(void);
extern void booz_fms_impl_periodic(void);

#define booz2_fms_test_signal_SetPeriod(_val) { \
    booz_fms_test_signal_period = _val;		\
    booz_fms_test_signal_counter = 0;		\
}

#define booz2_fms_test_signal_SetTsMode(_val) {				\
    booz_fms_test_signal_mode = _val;					\
  }

#endif /* BOOZ_FMS_TEST_SIGNAL_H */

