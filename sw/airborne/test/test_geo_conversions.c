#include <stdio.h>
// #include <stdlib.h>
// #include <math.h>

#include "std.h"

#include "pprz_algebra_print.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"

static void test_ned_of_lla_point_i(void);

int main(int argc, char **argv)
{
  test_ned_of_lla_point_i();
}

static void test_ned_of_lla_point_i(void)
{
  // The origin point
  struct LtpDef_i ltp_def;
  struct LlaCoor_i lla_origin = {51.989734 * 1e7, 4.375294 * 1e7, 40.0 * 1e3};
  ltp_def_from_lla_i(&ltp_def, &lla_origin);

  // The lla point
  struct LlaCoor_i lla_input_i = {51.990956 * 1e7, 4.379049 * 1e7, 40.0 * 1e3};

  // Convert lla to ned
  struct NedCoor_i ned_output;
  ned_of_lla_point_i(&ned_output, &ltp_def, &lla_input_i);

  // Convert to floating point
  struct NedCoor_f ned_output_f;
  NED_FLOAT_OF_BFP(ned_output_f, ned_output);

  // Print result
  DISPLAY_FLOAT_VECT3("lla_to_ned: ", ned_output_f);
  struct NedCoor_f ned_output_matlab = {135.975994223692, 257.939530628638, 0.00665492500429821};
  DISPLAY_FLOAT_VECT3("Matlab lla to ned: ", ned_output_matlab);
}
