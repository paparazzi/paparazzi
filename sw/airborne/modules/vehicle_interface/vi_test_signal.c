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

#include "booz_fms.h"

#include "subsystems/ins.h"
#include "math/pprz_algebra_int.h"

#define FMS_TEST_SIGNAL_DEFAULT_MODE       STEP_YAW
#define FMS_TEST_SIGNAL_DEFAULT_PERIOD     40
#define FMS_TEST_SIGNAL_DEFAULT_AMPLITUDE  (((int32_t)ANGLE_BFP_OF_REAL(RadOfDeg(5)))<<8)

struct BoozFmsTestSignal fms_test_signal;

void booz_fms_impl_init(void)
{
  fms_test_signal.mode      = FMS_TEST_SIGNAL_DEFAULT_MODE;
  fms_test_signal.period    = FMS_TEST_SIGNAL_DEFAULT_PERIOD;
  fms_test_signal.amplitude = FMS_TEST_SIGNAL_DEFAULT_AMPLITUDE;
  fms_test_signal.counter   = 0;
  fms.input.h_mode = GUIDANCE_H_MODE_ATTITUDE;
  fms.input.v_mode = GUIDANCE_V_MODE_HOVER;
}

void booz_fms_impl_periodic(void)
{

  switch (fms_test_signal.mode) {

    case STEP_ROLL: {
      if (fms_test_signal.counter < fms_test_signal.period) {
        EULERS_ASSIGN(fms.input.h_sp.attitude, fms_test_signal.amplitude, 0, 0);
      } else {
        EULERS_ASSIGN(fms.input.h_sp.attitude, -fms_test_signal.amplitude, 0, 0);
      }
    }
    break;
    case STEP_YAW: {
      if (fms_test_signal.counter < fms_test_signal.period) {
        EULERS_ASSIGN(fms.input.h_sp.attitude, 0, 0, fms_test_signal.amplitude);
      } else {
        EULERS_ASSIGN(fms.input.h_sp.attitude, 0, 0, -fms_test_signal.amplitude);
      }
    }
    break;
    case STEP_PITCH:
    case STEP_VERT:
      break;
#if 0
    case BOOZ_FMS_TEST_SIGNAL_MODE_VERTICAL: {
      if (guidance_v_mode < GUIDANCE_V_MODE_HOVER) {
        booz_fms_test_signal_start_z = ins_impl.ltp_pos.z;
      } else {
        booz_fms_input.v_sp.height = (booz_fms_test_signal_counter < booz_fms_test_signal_period) ?
                                     booz_fms_test_signal_start_z :
                                     booz_fms_test_signal_start_z - 256;
        //BOOZ_INT_OF_FLOAT(-0.5, IPOS_FRAC)
      }
    }
    break;
#endif
  }
  fms_test_signal.counter++;
  if (fms_test_signal.counter >= (2 * fms_test_signal.period)) {
    fms_test_signal.counter = 0;
  }
}

void booz_fms_impl_set_enabled(bool_t enabled)
{
  if (enabled) {
    fms_test_signal.counter = 0;
  }
}
