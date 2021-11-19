#include <stdio.h>
// #include <stdlib.h>
// #include <math.h>

#include "std.h"

#include "pprz_algebra_print.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"

static void test_ned_of_lla_point_i(void);
static void test_ecef_of_lla_point_i(void);
static void test_rmat_ltpdef(void);

int main(int argc, char **argv)
{
  test_ned_of_lla_point_i();
  printf("==================\n");
  test_ecef_of_lla_point_i();
  printf("==================\n");
  test_rmat_ltpdef();
  printf("==================\n");
}

static void test_ned_of_lla_point_i(void)
{
  // The origin point
  struct LtpDef_i ltp_def;
  struct LlaCoor_i lla_origin = {51.989734 * 1e7, 4.375294 * 1e7, 40.0 * 1e3};
  ltp_def_from_lla_i(&ltp_def, &lla_origin);

  // The lla point
  struct LlaCoor_i lla_input_i = {51.990956 * 1e7, 4.379049 * 1e7, 45.0 * 1e3};

  // Convert lla to ned (in cm, no pos_frac!)
  struct NedCoor_i ned_output;
  ned_of_lla_point_i(&ned_output, &ltp_def, &lla_input_i);

  // Convert to floating point
  struct NedCoor_f ned_output_f;
  VECT3_SDIV(ned_output_f, ned_output, 100.0);

  // Print result
  DISPLAY_FLOAT_VECT3("lla_to_ned: ", ned_output_f);
  struct NedCoor_f ned_output_matlab = {135.976100868939, 257.939732412809, -4.99334507054411};
  DISPLAY_FLOAT_VECT3("Matlab lla to ned: ", ned_output_matlab);
}

static void test_ecef_of_lla_point_i(void)
{
  // The origin point
  struct LtpDef_i ltp_def;
  struct LlaCoor_i lla_origin = {51.989734 * 1e7, 4.375294 * 1e7, 40.0 * 1e3};
  ltp_def_from_lla_i(&ltp_def, &lla_origin);

  // ecef in cm, so divide by 100
  struct EcefCoor_f ecef_f;
  VECT3_SDIV(ecef_f, ltp_def.ecef, 100.0);

  // Print result
  DISPLAY_FLOAT_VECT3("ecef point: ", ecef_f);
  struct NedCoor_f ecef_output_matlab = {3924414.96724007, 300265.100160884, 5002131.52944732};
  DISPLAY_FLOAT_VECT3("Matlab lla to ecef: ", ecef_output_matlab);
}

static void test_rmat_ltpdef(void)
{
  // The origin point
  struct LtpDef_i ltp_def;
  struct LlaCoor_i lla_origin = {51.989734 * 1e7, 4.375294 * 1e7, 40.0 * 1e3};
  ltp_def_from_lla_i(&ltp_def, &lla_origin);

  struct Int32Vect3 ecefv_i = {1000, 1000, 1000};
  struct EcefCoor_i ecef_point_i;
  VECT3_SUM(ecef_point_i, ecefv_i, ltp_def.ecef);
  struct NedCoor_i v_ned;
  // MAT33_VECT3_MUL(v_ned, ltp_def.ltp_of_ecef, ecefv);

  /** Convert a point from ECEF to local NED.
 * @param[out] ned  NED point in cm
 * @param[in]  def  local coordinate system definition
 * @param[in]  ecef ECEF point in cm
 */
  ned_of_ecef_point_i(&v_ned, &ltp_def, &ecef_point_i);

  struct NedCoor_f v_ned_f;
  VECT3_SDIV(v_ned_f, v_ned, 100.0);

  // Print result
  DISPLAY_FLOAT_VECT3("vned from vecef: ", v_ned_f);
  struct NedCoor_f ned_output_matlab = {-2.29909833234107, 9.2079665003871, -14.4888750352115};
  DISPLAY_FLOAT_VECT3("Matlab ecef to ned: ", ned_output_matlab);
}
