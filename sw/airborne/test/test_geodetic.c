/*
  http://en.wikipedia.org/wiki/Geodetic_system
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "std.h"

#include "pprz_geodetic_float.h"
//#include "pprz_geodetic_int.h"

static void test_floats(void);
//static void test_int(void);

/* 
 * toulouse 43.6052765, 1.4427764, 180.123019274324 -> 4624497.0 116475.0 4376563.0
 */

int main(int argc, char** argv) {

  test_floats();
  //  test_int();
  return 0;
}


static void test_floats(void) {

  //  struct LlaCoor_f ref_coor;
  //  ref_coor.lat = RAD_OF_DEG(43.605278);
  //  ref_coor.lon = RAD_OF_DEG(1.442778);
  //  ref_coor.alt = 180.0;
  
  struct EcefCoor_f ref_coor;
  ref_coor.x = 4624497.0;
  ref_coor.y =  116475.0;
  ref_coor.z = 4376563.0;

  struct LtpRef_f ltp_param;
  init_ltp_ref_from_ecef_f(&ltp_param, &ref_coor);

  printf("lla0 : (%f,%f,%f)\n", DegOfRad(ltp_param.lla.lat), DegOfRad(ltp_param.lla.lon), ltp_param.lla.alt); 
  

  struct EcefCoor_f my_ecef_point = ref_coor; //{ 4624491.0, 116475.0, 4376563.0};
  struct EnuCoor_f my_enu_point;
  enu_of_ecef_f(&my_enu_point, &ltp_param, &my_ecef_point);

  printf("ecef to enu : (%f,%f,%f) -> (%f,%f,%f)\n", 
	 my_ecef_point.x, my_ecef_point.y, my_ecef_point.z, 
	 my_enu_point.x, my_enu_point.y, my_enu_point.z );
}

































#if 0
static void test_int(void) {
  struct LlaCoor_i ref_coor;
  ref_coor.lat = (int32_t)(RAD_OF_DEG(43.605278)*1e7);
  ref_coor.lon = (int32_t)(RAD_OF_DEG( 1.442778)*1e7);
  ref_coor.alt = (int32_t)(180.0*1e2);
  
  struct LtpRef_i lpt_param;
  /*  FIXME: init_ltp_from_lla_f not implemented */
  lpt_param.ecef.x = (int32_t)(4624497.0*1e2);
  lpt_param.ecef.y = (int32_t)( 116475.0*1e2);
  lpt_param.ecef.z = (int32_t)(4376563.0*1e2);
  init_ltp_from_lla_i(&lpt_param, &ref_coor);

  
  struct EcefCoor_i my_ecef_point = { (int32_t)4624491.0*1e2,
				      (int32_t) 116475.0*1e2,
				      (int32_t)4376563.0*1e2 };
  struct EnuCoor_i  my_enu_point;
  enu_of_ecef_i(&lpt_param, &my_enu_point, &my_ecef_point);
  
  printf("ecef to enu : (%d,%d,%d) -> (%d,%d,%d)\n", 
	 my_ecef_point.x, my_ecef_point.y, my_ecef_point.z, 
	 my_enu_point.x, my_enu_point.y, my_enu_point.z );
}
#endif

