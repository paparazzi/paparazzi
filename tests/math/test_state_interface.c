/*
 * Copyright (C) 2014 Felix Ruess <felix.ruess@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file test_state_interface.c
 * @brief Tests for Paparazzi state interface.
 *
 * Which heavily uses the math libary.
 *
 * Using libtap to create a TAP (TestAnythingProtocol) producer:
 * https://github.com/zorgnax/libtap
 *
 */

#include "tap.h"
#include "state.h"
#include "math/pprz_geodetic_double.h"

static void test_pos_lla_i(void)
{
  struct LlaCoor_d lla_ref = {.lat=0.749999999392454875,
                              .lon=0.019999999054505127,
                              .alt=180.0};
  note("LLA reference: lat: %.10f deg, lon: %.10f deg, alt %.4f m",
       DegOfRad(lla_ref.lat), DegOfRad(lla_ref.lon), lla_ref.alt);

  struct LlaCoor_i lla_ref_i;
  LLA_BFP_OF_REAL(lla_ref_i, lla_ref);

  struct EcefCoor_d ecef_ref;
  ecef_of_lla_d(&ecef_ref, &lla_ref);
  note("ECEF reference: x: %.4f m, y: %.4f m, z: %.4f m",
       ecef_ref.x, ecef_ref.y, ecef_ref.z);

  /* calc lla_i from ecef_f */
  struct EcefCoor_f ecef_f;
  VECT3_COPY(ecef_f, ecef_ref);
  stateSetPositionEcef_f(&ecef_f);

  struct LlaCoor_i *lla_result;
  lla_result = stateGetPositionLla_i();

  /* compare LLA result with reference (directly convered from double) */
  int32_t lat_err = abs(lla_result->lat - lla_ref_i.lat);
  int32_t lon_err = abs(lla_result->lon - lla_ref_i.lon);
  int32_t alt_err = abs(lla_result->alt - lla_ref_i.alt);

  /* max error is 1e-6 deg in lat/lon and 10cm in alt (lla_i is in 1e7deg and mm) */
  const int32_t max_err_i = 1e-6 * 1e7; // convert to 1e7 deg
  if (lat_err < max_err_i && lon_err < max_err_i && alt_err < 100) {
    pass("stateGetPositionLla_i() from ecef_f has max error of 1e-6deg in lat/lon and 10cm in lat");
  }
  else {
    note("lat is %d, should be %d", lla_result->lat, lla_ref_i.lat);
    note("lon is %d, should be %d", lla_result->lon, lla_ref_i.lon);
    note("alt is %d, should be %d", lla_result->alt, lla_ref_i.alt);
    fail("stateGetPositionLla_i() from ecef_f exceeded error of 1e-6deg in lat/lon and 10cm in lat");
  }
}

int main()
{
  note("\n *** running state interface tests ***");
  plan(1);

  stateInit();

  test_pos_lla_i();

  done_testing();
}
