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
 * @file test_pprz_geodetic.c
 * @brief Tests for Paparazzi geodetic math functions.
 *
 * Using libtap to create a TAP (TestAnythingProtocol) producer:
 * https://github.com/zorgnax/libtap
 *
 */

#include "tap.h"

#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_double.h"

/*
 * toulouse lat 43.6052765, lon 1.4427764, alt 180.123019274324 -> x 4624497.0 y 116475.0 z 4376563.0
 */

static void test_ecef_of_ned_int(void)
{
  const float step_ned = 100.0;
  const float range_ned = 10000.0;

  note("--- Compare ecef_of_ned int vs. double with %.fm range in %.fm steps", range_ned, step_ned);
  struct EcefCoor_d ref_coor_d = { 4624497.0, 116475.0, 4376563.0};
  struct LtpDef_d ltp_def_d;
  ltp_def_from_ecef_d(&ltp_def_d, &ref_coor_d);

  struct EcefCoor_i ref_coor_i = { rint(CM_OF_M(ref_coor_d.x)),
                                   rint(CM_OF_M(ref_coor_d.y)),
                                   rint(CM_OF_M(ref_coor_d.z))};
  note("ecef0 : (%d, %d, %d)", ref_coor_i.x, ref_coor_i.y, ref_coor_i.z);
  struct LtpDef_i ltp_def_i;
  ltp_def_from_ecef_i(&ltp_def_i, &ref_coor_i);


  double sum_err = 0;
  struct FloatVect3 max_err;
  FLOAT_VECT3_ZERO(max_err);
  struct FloatVect3 offset;
  for (offset.x = -range_ned; offset.x <= range_ned; offset.x += step_ned) {
    for (offset.y = -range_ned; offset.y <= range_ned; offset.y += step_ned) {
      for (offset.z = -range_ned; offset.z <= range_ned; offset.z += step_ned) {
        struct NedCoor_d my_ned_point_d;
        VECT3_COPY(my_ned_point_d, offset);
        struct EcefCoor_d  my_ecef_point_d;
        ecef_of_ned_point_d(&my_ecef_point_d, &ltp_def_d, &my_ned_point_d);
#if DEBUG
        note("ned to ecef double : (%.02f,%.02f,%.02f) -> (%.02f,%.02f,%.02f)\n",
             my_ned_point_d.x, my_ned_point_d.y, my_ned_point_d.z,
             my_ecef_point_d.x, my_ecef_point_d.y, my_ecef_point_d.z);
#endif

        struct NedCoor_i my_ned_point_i;
        POSITIONS_BFP_OF_REAL(my_ned_point_i, my_ned_point_d);
        struct EcefCoor_i  my_ecef_point_i;
        ecef_of_ned_pos_i(&my_ecef_point_i, &ltp_def_i, &my_ned_point_i);

#if DEBUG
        note("def->ecef (%d,%d,%d)", ltp_def_i.ecef.x, ltp_def_i.ecef.y, ltp_def_i.ecef.z);
        note("ned to ecef int   : (%.2f,%.02f,%.02f) -> (%.02f,%.02f,%.02f)\n\n",
             POS_BFP_OF_REAL(my_ned_point_i.x),
             POS_BFP_OF_REAL(my_ned_point_i.y),
             POS_BFP_OF_REAL(my_ned_point_i.z),
             M_OF_CM((double)my_ecef_point_i.x),
             M_OF_CM((double)my_ecef_point_i.y),
             M_OF_CM((double)my_ecef_point_i.z));
#endif

        float ex = my_ecef_point_d.x - M_OF_CM((double)my_ecef_point_i.x);
        if (fabs(ex) > max_err.x) { max_err.x = fabs(ex); }
        float ey = my_ecef_point_d.y - M_OF_CM((double)my_ecef_point_i.y);
        if (fabs(ey) > max_err.y) { max_err.y = fabs(ey); }
        float ez = my_ecef_point_d.z - M_OF_CM((double)my_ecef_point_i.z);
        if (fabs(ez) > max_err.z) { max_err.z = fabs(ez); }
        sum_err += ex * ex + ey * ey + ez * ez;
      }
    }
  }

  double nb_samples = (2 * range_ned / step_ned + 1) * (2 * range_ned / step_ned + 1) *
                      (2 * range_ned / step_ned + 1);

  float max_err_norm = float_vect3_norm(&max_err);
  float avg_err = sqrt(sum_err) / nb_samples;

  note("ecef_of_ned_pos int/double comparison of %d samples:", nb_samples);
  note("error max (%.3f, %.3f, %.3f) m, norm %.3f m", max_err.x, max_err.y, max_err.z,
       max_err_norm);
  note("error avg %f m", avg_err);

  ok(max_err_norm < 0.1, "ecef_of_ned (int) max error is below 10cm");
  ok(avg_err < 0.01, "ecef_of_ned (int) average error is below 1cm");
}

static void test_enu_of_ecef_int(void)
{
  const float step = 100.0;
  const float range = 10000.0;

  note("--- Compare enu_of_ecef int vs. double with %.fm range in %.fm steps", range, step);
  struct EcefCoor_f ref_coor_f = { 4624497.0 , 116475.0, 4376563.0};
  struct LtpDef_f ltp_def_f;
  ltp_def_from_ecef_f(&ltp_def_f, &ref_coor_f);

  struct EcefCoor_i ref_coor_i = { rint(CM_OF_M(ref_coor_f.x)),
                                   rint(CM_OF_M(ref_coor_f.y)),
                                   rint(CM_OF_M(ref_coor_f.z))};
  note("ecef0 : (%d, %d, %d)", ref_coor_i.x, ref_coor_i.y, ref_coor_i.z);
  struct LtpDef_i ltp_def_i;
  ltp_def_from_ecef_i(&ltp_def_i, &ref_coor_i);
  note("lla0 : float (%f deg, %f deg, %f m) , int (%f, %f, %f)",
       DegOfRad(ltp_def_f.lla.lat), DegOfRad(ltp_def_f.lla.lon), ltp_def_f.lla.alt,
       DegOfRad(RAD_OF_EM7RAD((double)ltp_def_i.lla.lat)),
       DegOfRad(RAD_OF_EM7RAD((double)ltp_def_i.lla.lon)),
       M_OF_MM((double)ltp_def_i.lla.alt));

  double sum_err = 0;
  struct FloatVect3 max_err;
  FLOAT_VECT3_ZERO(max_err);
  struct FloatVect3 offset;
  for (offset.x=-range; offset.x<=range; offset.x+=step) {
    for (offset.y=-range; offset.y<=range; offset.y+=step) {
      for (offset.z=-range; offset.z<=range; offset.z+=step) {
        struct EcefCoor_f my_ecef_point_f = ref_coor_f;
        VECT3_ADD(my_ecef_point_f, offset);
        struct EnuCoor_f  my_enu_point_f;
        enu_of_ecef_point_f(&my_enu_point_f, &ltp_def_f, &my_ecef_point_f);
#if DEBUG
        note("ecef to enu float : (%.02f,%.02f,%.02f) -> (%.02f,%.02f,%.02f)\n",
             my_ecef_point_f.x, my_ecef_point_f.y, my_ecef_point_f.z,
             my_enu_point_f.x, my_enu_point_f.y, my_enu_point_f.z );
#endif

        struct EcefCoor_i my_ecef_point_i = { rint(CM_OF_M(my_ecef_point_f.x)),
                                              rint(CM_OF_M(my_ecef_point_f.y)),
                                              rint(CM_OF_M(my_ecef_point_f.z))};;
        struct EnuCoor_i  my_enu_point_i;
        enu_of_ecef_point_i(&my_enu_point_i, &ltp_def_i, &my_ecef_point_i);

#if DEBUG
        //note("def->ecef (%d,%d,%d)", ltp_def_i.ecef.x, ltp_def_i.ecef.y, ltp_def_i.ecef.z);
        printf("ecef to enu int   : (%.2f,%.02f,%.02f) -> (%.02f,%.02f,%.02f)\n\n",
               M_OF_CM((double)my_ecef_point_i.x),
               M_OF_CM((double)my_ecef_point_i.y),
               M_OF_CM((double)my_ecef_point_i.z),
               M_OF_CM((double)my_enu_point_i.x),
               M_OF_CM((double)my_enu_point_i.y),
               M_OF_CM((double)my_enu_point_i.z));
#endif

        float ex = my_enu_point_f.x - M_OF_CM((double)my_enu_point_i.x);
        if (fabs(ex) > max_err.x) max_err.x = fabs(ex);
        float ey = my_enu_point_f.y - M_OF_CM((double)my_enu_point_i.y);
        if (fabs(ey) > max_err.y) max_err.y = fabs(ey);
        float ez = my_enu_point_f.z - M_OF_CM((double)my_enu_point_i.z);
        if (fabs(ez) > max_err.z) max_err.z = fabs(ez);
        sum_err += ex*ex + ey*ey + ez*ez;
      }
    }
  }

  double nb_samples = (2*range / step + 1) * (2*range / step + 1) *  (2*range / step + 1);

  float max_err_norm = float_vect3_norm(&max_err);
  float avg_err = sqrt(sum_err) / nb_samples;

  note("enu_of_ecef int/float comparison of %d samples:", nb_samples);
  note("error max (%.3f, %.3f, %.3f) m, norm %.3f m", max_err.x, max_err.y, max_err.z,
       max_err_norm);
  note("error avg %f m", sqrt(sum_err) / nb_samples );

  ok(max_err_norm < 0.1, "enu_of_ecef (int) max error is below 10cm");
  ok(avg_err < 0.01, "enu_of_ecef (int) average error is below 1cm");
}


static void test_lla_of_utm(void)
{
  note("--- test lla_of_utm (double and float)");

  struct UtmCoor_d utm_d = { .east=348805.71, .north=4759354.89, .zone=31 };
  note("UTM input: east: %.2f m, north: %.2f m, zone: %d", utm_d.east, utm_d.north, utm_d.zone);
  struct LlaCoor_d l_ref_d = {.lat=0.749999999392454875,
                              .lon=0.019999999054505127};
  note("LLA reference output in rad: lat %.16f rad, lon %.16f rad",
       l_ref_d.lat, l_ref_d.lon);
  note("LLA ref_d.lat=%.12f deg    ref_d.lon=%.12f deg",
       DegOfRad(l_ref_d.lat), DegOfRad(l_ref_d.lon));

  struct LlaCoor_d lla_d;
  lla_of_utm_d(&lla_d, &utm_d);

  const double max_err_rad = RadOfDeg(0.0001);

  double lat_d_err = fabs(l_ref_d.lat - lla_d.lat);
  double lon_d_err = fabs(l_ref_d.lon - lla_d.lon);
  if (lat_d_err < max_err_rad && lon_d_err < max_err_rad) {
    note("    lla_d.lat=%.12f deg    lla_d.lon=%.12f deg",
         DegOfRad(lla_d.lat), DegOfRad(lla_d.lon));
    pass("lla_of_utm_d error is below 1e-4 deg");
  }
  else {
    note("    lat=%.16f     lon=%.16f\nref_lat=%.16f ref_lon=%.16f\n",
         lla_d.lat, lla_d.lon, l_ref_d.lat, l_ref_d.lon);
    fail("lla_of_utm_d error is NOT below 1e-4 deg");
  }

  struct UtmCoor_f utm_f = { .east=348805.71, .north=4759354.89, .zone=31 };
  struct LlaCoor_f lla_f;
  struct LlaCoor_f l_ref_f = {.lat=0.749999999392454875,
                              .lon=0.019999999054505127};
  lla_of_utm_f(&lla_f, &utm_f);

  float lat_f_err = fabs(l_ref_d.lat - lla_f.lat);
  float lon_f_err = fabs(l_ref_d.lon - lla_f.lon);
  if (lat_f_err < max_err_rad && lon_f_err < max_err_rad) {
    note("    lla_f.lat=%.12f deg    lla_f.lon=%.12f deg",
         DegOfRad(lla_f.lat), DegOfRad(lla_f.lon));
    pass("lla_of_utm_f error is below 1e-4 deg");
  }
  else {
    note("    lat=%.16f     lon=%.16f\nref_lat=%.16f ref_lon=%.16f\n",
         lla_f.lat, lla_f.lon, l_ref_f.lat, l_ref_f.lon);
    fail("lla_of_utm_f error is NOT below 1e-4 deg");
  }
}

static void test_enu_of_ecef_double(void)
{

  note("--- enu_of_ecef double");
  //  struct LlaCoor_f ref_coor;
  //  ref_coor.lat = RAD_OF_DEG(43.605278);
  //  ref_coor.lon = RAD_OF_DEG(1.442778);
  //  ref_coor.alt = 180.0;

  struct EcefCoor_d ref_coor = { 4624497.0 , 116475.0, 4376563.0};
  note("ecef0 : (%.02f, %.02f, %.02f)\n", ref_coor.x, ref_coor.y, ref_coor.z);

  struct LtpDef_d ltp_def;
  ltp_def_from_ecef_d(&ltp_def, &ref_coor);

  note("lla0 : (%f, %f, %f)\n", DegOfRad(ltp_def.lla.lat), DegOfRad(ltp_def.lla.lon),
       ltp_def.lla.alt);

  struct EcefCoor_d my_ecef_point = ref_coor;
  struct EnuCoor_d  my_enu_point;
  enu_of_ecef_point_d(&my_enu_point, &ltp_def, &my_ecef_point);

  note("ecef to enu : (%f, %f, %f) -> (%f , %f, %f)\n",
       my_ecef_point.x, my_ecef_point.y, my_ecef_point.z,
       my_enu_point.x, my_enu_point.y, my_enu_point.z);

  ok(FLOAT_VECT3_NORM(my_enu_point) < 0.001, "enu_of_ecef error below 1mm");
}

static void test_ecef_to_enu_to_ecef_float(void)
{
  note("--- test ECEF -> ENU -> ECEF in float");

  struct EcefCoor_f ref_coor = { 4624497.0 , 116475.0, 4376563.0};
  note("LtpDef ecef0 : (%.02f, %.02f, %.02f)\n", ref_coor.x, ref_coor.y, ref_coor.z);

  struct LtpDef_f ltp_def;
  ltp_def_from_ecef_f(&ltp_def, &ref_coor);

  //struct EcefCoor_f ecef_p1 = ref_coor;
  struct EcefCoor_f ecef_p1 = { 4624000.42 , 116400.42, 4376000.42};
  struct EnuCoor_f  enu_p1;
  enu_of_ecef_point_f(&enu_p1, &ltp_def, &ecef_p1);
  note("ecef to enu : (%f, %f, %f) -> (%f, %f ,%f)\n",
       ecef_p1.x, ecef_p1.y, ecef_p1.z,
       enu_p1.x, enu_p1.y, enu_p1.z );

  struct EcefCoor_f ecef_p2;
  ecef_of_enu_point_f(&ecef_p2, &ltp_def, &enu_p1);
  note("enu to ecef : (%f, %f, %f) -> (%f, %f, %f)\n",
       enu_p1.x, enu_p1.y, enu_p1.z,
       ecef_p2.x, ecef_p2.y, ecef_p2.z);

  struct EcefCoor_f ecef_err;
  VECT3_DIFF(ecef_err, ecef_p1, ecef_p2);
  VECT3_ABS(ecef_err, ecef_err);

  ok(FLOAT_VECT3_NORM(ecef_err) < 0.001, "ECEF->ENU->ECEF error below 1mm");
}

static void test_lla_of_ecef(void)
{
  note("--- test lla <-> ecef");

  struct EcefCoor_d ecef_ref = { 4624497.0 , 116475.0, 4376563.0};
  note("ECEF reference: x: %.3f m, y: %.3f m, z: %.3f m",
       ecef_ref.x, ecef_ref.y, ecef_ref.z);


  struct LlaCoor_d lla_ref;
  lla_of_ecef_d(&lla_ref, &ecef_ref);
  //lla_ref.lat = RadOfDeg(43.605278);
  //lla_ref.lon = RadOfDeg(1.442778);
  //lla_ref.alt = 180.0;
  note("LLA reference: lat: %.8f deg, lon: %.8f deg, alt %.3f m",
       DegOfRad(lla_ref.lat), DegOfRad(lla_ref.lon), lla_ref.alt);

  /* check ECEF difference from ECEF -> LLA -> ECEF */
  struct EcefCoor_d ecef_check;
  ecef_of_lla_d(&ecef_check, &lla_ref);
  struct EcefCoor_d ecef_diff;
  VECT3_DIFF(ecef_diff, ecef_ref, ecef_check);
  double ecef_err = sqrt(VECT3_NORM2(ecef_diff));
  ok(ecef_err < 1e-6, "ECEF -> LLA -> ECEF in double has less than 1e-6m error");

  struct LlaCoor_d lla_check;
  lla_of_ecef_d(&lla_check, &ecef_ref);

  /* convert the reference from double to BFP (int) */
  struct LlaCoor_i lla_ref_i;
  LLA_BFP_OF_REAL(lla_ref_i, lla_ref);
  struct EcefCoor_i ecef_ref_i;
  ECEF_BFP_OF_REAL(ecef_ref_i, ecef_ref);

  /* convert ECEF to LLA in fixed point.
   * FIXME: this is a bit pointless right now,
   * since lla_of_ecef_i simply calls the double version.
   * Anyone up for writing it in fixed point??
   */
  struct LlaCoor_i lla_i;
  lla_of_ecef_i(&lla_i, &ecef_ref_i);

  /* compare LLA computed in fixed point with reference (convered from double) */
  cmp_ok(lla_i.lat, "==", lla_ref_i.lat, "latitude (int) matches reference");
  cmp_ok(lla_i.lon, "==", lla_ref_i.lon, "longitude (int) matches reference");
  cmp_ok(lla_i.alt, "==", lla_ref_i.alt, "altitude (int) matches reference");
}

int main()
{
  note("runing geodetic math tests");
  plan(12);

  test_ecef_of_ned_int();
  test_enu_of_ecef_int();
  test_enu_of_ecef_double();
  test_ecef_to_enu_to_ecef_float();
  test_lla_of_utm();
  test_lla_of_ecef();

  done_testing();
}
