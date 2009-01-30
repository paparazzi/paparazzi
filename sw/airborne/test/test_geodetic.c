/*
  http://en.wikipedia.org/wiki/Geodetic_system
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define DEG_OF_RAD(_r) (_r * 180. / M_PI)
#define RAD_OF_DEG(_d) (_d / 180. * M_PI)

#include "pprz_geodetic_float.h"


/* 
 * toulouse 43.605278,1.442778,180.0 -> 4624497.0 116475.0 4376563.0
 */

int main(int argc, char** argv) {

  struct LlaCoor_f ref_coor;
  ref_coor.lat = RAD_OF_DEG(43.605278);
  ref_coor.lon = RAD_OF_DEG(1.442778);
  ref_coor.alt = 180.0;
  
  struct LtpRef_f lpt_param;
  init_ltp_from_lla_f(&lpt_param, &ref_coor);

  /*  FIXME: init_ltp_from_lla_f not implemented */
  lpt_param.ecef.x = 4624497.0;
  lpt_param.ecef.y =  116475.0;
  lpt_param.ecef.z = 4376563.0;

  struct EcefCoor_f my_ecef_point = { 4624491.0, 116475.0, 4376563.0};
  struct EnuCoor_f my_enu_point;
  enu_of_ecef_f(&lpt_param, &my_enu_point, &my_ecef_point);

  printf("ecef to enu : (%f,%f,%f) -> (%f,%f,%f)\n", 
	 my_ecef_point.x, my_ecef_point.y, my_ecef_point.z, 
	 my_enu_point.x, my_enu_point.y, my_enu_point.z );
  
  return 0;
}
