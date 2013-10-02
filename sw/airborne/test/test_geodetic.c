/*
  http://en.wikipedia.org/wiki/Geodetic_system
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "std.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_int.h"

//#define DEBUG 1


static void test_lla_of_utm(void);
static void test_floats(void);
static void test_doubles(void);
static void test_enu_of_ecef_int(void);
static void test_ecef_of_ned_int(void);
static void test_ned_to_ecef_to_ned(void);
static void test_enu_to_ecef_to_enu( void );

/*
 * toulouse lat 43.6052765, lon 1.4427764, alt 180.123019274324 -> x 4624497.0 y 116475.0 z 4376563.0
 */

int main(int argc, char** argv) {

  //test_floats();
  //test_doubles();

  //test_lla_of_utm();

  //test_ned_to_ecef_to_ned();

  //test_enu_to_ecef_to_enu();

  test_ecef_of_ned_int();

  //test_enu_of_ecef_int();

  return 0;

}

static void test_lla_of_utm(void) {
  printf("\n--- lla of UTM double ---\n");

  struct UtmCoor_d utm_d = { .east=348805.71, .north=4759354.89, .zone=31 };
  struct LlaCoor_d lla_d;
  struct LlaCoor_d l_ref_d = {.lat=0.749999999392454875,
                              .lon=0.019999999054505127};
  lla_of_utm_d(&lla_d, &utm_d);
  printf("    lat=%.16f     lon=%.16f\nref_lat=%.16f ref_lon=%.16f\n",
         lla_d.lat, lla_d.lon, l_ref_d.lat, l_ref_d.lon);

  printf("\n--- lla of UTM float ---\n");

  struct UtmCoor_f utm_f = { .east=348805.71, .north=4759354.89, .zone=31 };
  struct LlaCoor_f lla_f;
  struct LlaCoor_f l_ref_f = {.lat=0.749999999392454875,
                              .lon=0.019999999054505127};
  lla_of_utm_f(&lla_f, &utm_f);
  printf("    lat=%.16f     lon=%.16f\nref_lat=%.16f ref_lon=%.16f\n",
         lla_f.lat, lla_f.lon, l_ref_f.lat, l_ref_f.lon);
}

static void test_floats(void) {

  printf("\n--- enu_of_ecef float ---\n");
  //  struct LlaCoor_f ref_coor;
  //  ref_coor.lat = RAD_OF_DEG(43.605278);
  //  ref_coor.lon = RAD_OF_DEG(1.442778);
  //  ref_coor.alt = 180.0;

  struct EcefCoor_f ref_coor = { 4624497.0 , 116475.0, 4376563.0};
  printf("ecef0 : (%.02f,%.02f,%.02f)\n", ref_coor.x, ref_coor.y, ref_coor.z);

  struct LtpDef_f ltp_def;
  ltp_def_from_ecef_f(&ltp_def, &ref_coor);

  printf("lla0 : (%f,%f,%f)\n", DegOfRad(ltp_def.lla.lat), DegOfRad(ltp_def.lla.lon),
         ltp_def.lla.alt);

  struct EcefCoor_f my_ecef_point = ref_coor;
  struct EnuCoor_f  my_enu_point;
  enu_of_ecef_point_f(&my_enu_point, &ltp_def, &my_ecef_point);

  printf("ecef to enu : (%f,%f,%f) -> (%f,%f,%f)\n",
         my_ecef_point.x, my_ecef_point.y, my_ecef_point.z,
         my_enu_point.x, my_enu_point.y, my_enu_point.z );
  printf("\n");
}


static void test_doubles(void) {

  printf("\n--- enu_of_ecef double ---\n");
  //  struct LlaCoor_f ref_coor;
  //  ref_coor.lat = RAD_OF_DEG(43.605278);
  //  ref_coor.lon = RAD_OF_DEG(1.442778);
  //  ref_coor.alt = 180.0;

  struct EcefCoor_d ref_coor = { 4624497.0 , 116475.0, 4376563.0};
  printf("ecef0 : (%.02f,%.02f,%.02f)\n", ref_coor.x, ref_coor.y, ref_coor.z);

  struct LtpDef_d ltp_def;
  ltp_def_from_ecef_d(&ltp_def, &ref_coor);

  printf("lla0 : (%f,%f,%f)\n", DegOfRad(ltp_def.lla.lat), DegOfRad(ltp_def.lla.lon),
         ltp_def.lla.alt);

  struct EcefCoor_d my_ecef_point = ref_coor;
  struct EnuCoor_d  my_enu_point;
  enu_of_ecef_point_d(&my_enu_point, &ltp_def, &my_ecef_point);

  printf("ecef to enu : (%f,%f,%f) -> (%f,%f,%f)\n",
         my_ecef_point.x, my_ecef_point.y, my_ecef_point.z,
         my_enu_point.x, my_enu_point.y, my_enu_point.z );
  printf("\n");
}

static void test_ecef_of_ned_int(void) {

  printf("\n--- ecef_of_ned int ---\n");
  struct EcefCoor_d ref_coor_d = { 4624497.0 , 116475.0, 4376563.0};
  struct LtpDef_d ltp_def_d;
  ltp_def_from_ecef_d(&ltp_def_d, &ref_coor_d);

  struct EcefCoor_i ref_coor_i = { rint(CM_OF_M(ref_coor_d.x)),
                                   rint(CM_OF_M(ref_coor_d.y)),
                                   rint(CM_OF_M(ref_coor_d.z))};
  printf("ecef0 : (%d,%d,%d)\n", ref_coor_i.x, ref_coor_i.y, ref_coor_i.z);
  struct LtpDef_i ltp_def_i;
  ltp_def_from_ecef_i(&ltp_def_i, &ref_coor_i);


#define STEP_NED  100.0
#define RANGE_NED 10000.
  double sum_err = 0;
  struct FloatVect3 max_err;
  FLOAT_VECT3_ZERO(max_err);
  struct FloatVect3 offset;
  for (offset.x=-RANGE_NED; offset.x<=RANGE_NED; offset.x+=STEP_NED) {
    for (offset.y=-RANGE_NED; offset.y<=RANGE_NED; offset.y+=STEP_NED) {
      for (offset.z=-RANGE_NED; offset.z<=RANGE_NED; offset.z+=STEP_NED) {
        struct NedCoor_d my_ned_point_d;
        VECT3_COPY(my_ned_point_d, offset);
        struct EcefCoor_d  my_ecef_point_d;
        ecef_of_ned_point_d(&my_ecef_point_d, &ltp_def_d, &my_ned_point_d);
#if DEBUG
        printf("ned to ecef double : (%.02f,%.02f,%.02f) -> (%.02f,%.02f,%.02f)\n",
               my_enu_point_f.x, my_enu_point_f.y, my_enu_point_f.z,
               my_ecef_point_f.x, my_ecef_point_f.y, my_ecef_point_f.z);
#endif

        struct NedCoor_i my_ned_point_i;
        POSITIONS_BFP_OF_REAL(my_ned_point_i, my_ned_point_d);
        struct EcefCoor_i  my_ecef_point_i;
        ecef_of_ned_pos_i(&my_ecef_point_i, &ltp_def_i, &my_ned_point_i);

#if DEBUG
        //printf("def->ecef (%d,%d,%d)\n", ltp_def_i.ecef.x, ltp_def_i.ecef.y, ltp_def_i.ecef.z);
        printf("ned to ecef int   : (%.2f,%.02f,%.02f) -> (%.02f,%.02f,%.02f)\n\n",
               POS_BFP_OF_REAL(my_ned_point_i.x),
               POS_BFP_OF_REAL(my_ned_point_i.y),
               POS_BFP_OF_REAL(my_ned_point_i.z),
               M_OF_CM((double)my_ecef_point_i.x),
               M_OF_CM((double)my_ecef_point_i.y),
               M_OF_CM((double)my_ecef_point_i.z));
#endif

        float ex = my_ecef_point_d.x - M_OF_CM((double)my_ecef_point_i.x);
        if (fabs(ex) > max_err.x) max_err.x = fabs(ex);
        float ey = my_ecef_point_d.y - M_OF_CM((double)my_ecef_point_i.y);
        if (fabs(ey) > max_err.y) max_err.y = fabs(ey);
        float ez = my_ecef_point_d.z - M_OF_CM((double)my_ecef_point_i.z);
        if (fabs(ez) > max_err.z) max_err.z = fabs(ez);
        sum_err += ex*ex + ey*ey + ez*ez;
      }
    }
  }

  double nb_samples = (2*RANGE_NED / STEP_NED + 1) * (2*RANGE_NED / STEP_NED + 1) *
    (2*RANGE_NED / STEP_NED + 1);


  printf("ecef_of_ned_pos int/float comparison:\n");
  printf("error max (%f,%f,%f) m\n", max_err.x, max_err.y, max_err.z );
  printf("error avg (%f ) m \n", sqrt(sum_err) / nb_samples );
  printf("\n");

}


static void test_enu_of_ecef_int(void) {

  printf("\n--- enu_of_ecef int ---\n");
  struct EcefCoor_f ref_coor_f = { 4624497.0 , 116475.0, 4376563.0};
  struct LtpDef_f ltp_def_f;
  ltp_def_from_ecef_f(&ltp_def_f, &ref_coor_f);

  struct EcefCoor_i ref_coor_i = { rint(CM_OF_M(ref_coor_f.x)),
                                   rint(CM_OF_M(ref_coor_f.y)),
                                   rint(CM_OF_M(ref_coor_f.z))};
  printf("ecef0 : (%d,%d,%d)\n", ref_coor_i.x, ref_coor_i.y, ref_coor_i.z);
  struct LtpDef_i ltp_def_i;
  ltp_def_from_ecef_i(&ltp_def_i, &ref_coor_i);
  printf("lla0 : float (%f deg, %f deg, %f m) , int (%f, %f, %f)\n",
         DegOfRad(ltp_def_f.lla.lat), DegOfRad(ltp_def_f.lla.lon), ltp_def_f.lla.alt,
         DegOfRad(RAD_OF_EM7RAD((double)ltp_def_i.lla.lat)),
         DegOfRad(RAD_OF_EM7RAD((double)ltp_def_i.lla.lon)),
         M_OF_MM((double)ltp_def_i.lla.alt));

#define STEP    1000.
#define RANGE 100000.
  double sum_err = 0;
  struct FloatVect3 max_err;
  FLOAT_VECT3_ZERO(max_err);
  struct FloatVect3 offset;
  for (offset.x=-RANGE; offset.x<=RANGE; offset.x+=STEP) {
    for (offset.y=-RANGE; offset.y<=RANGE; offset.y+=STEP) {
      for (offset.z=-RANGE; offset.z<=RANGE; offset.z+=STEP) {
        struct EcefCoor_f my_ecef_point_f = ref_coor_f;
        VECT3_ADD(my_ecef_point_f, offset);
        struct EnuCoor_f  my_enu_point_f;
        enu_of_ecef_point_f(&my_enu_point_f, &ltp_def_f, &my_ecef_point_f);
#if DEBUG
        printf("ecef to enu float : (%.02f,%.02f,%.02f) -> (%.02f,%.02f,%.02f)\n",
               my_ecef_point_f.x, my_ecef_point_f.y, my_ecef_point_f.z,
               my_enu_point_f.x, my_enu_point_f.y, my_enu_point_f.z );
#endif

        struct EcefCoor_i my_ecef_point_i = { rint(CM_OF_M(my_ecef_point_f.x)),
                                              rint(CM_OF_M(my_ecef_point_f.y)),
                                              rint(CM_OF_M(my_ecef_point_f.z))};;
        struct EnuCoor_i  my_enu_point_i;
        enu_of_ecef_point_i(&my_enu_point_i, &ltp_def_i, &my_ecef_point_i);

#if DEBUG
        //printf("def->ecef (%d,%d,%d)\n", ltp_def_i.ecef.x, ltp_def_i.ecef.y, ltp_def_i.ecef.z);
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

  double nb_samples = (2*RANGE / STEP + 1) * (2*RANGE / STEP + 1) *  (2*RANGE / STEP + 1);


  printf("enu_of_ecef int/float comparison:\n");
  printf("error max (%f,%f,%f) m\n", max_err.x, max_err.y, max_err.z );
  printf("error avg (%f ) m \n", sqrt(sum_err) / nb_samples );
  printf("\n");

}





void test_ned_to_ecef_to_ned( void ) {

  printf("\n--- NED -> ECEF -> NED in double ---\n");

  struct EcefCoor_d ref_coor = { 4624497.0 , 116475.0, 4376563.0};
  printf("ecef0 : (%.02f,%.02f,%.02f)\n", ref_coor.x, ref_coor.y, ref_coor.z);

  struct LtpDef_d ltp_def;
  ltp_def_from_ecef_d(&ltp_def, &ref_coor);

  struct EcefCoor_d ecef_p1 = ref_coor;
  struct NedCoor_d  ned_p1;
  ned_of_ecef_point_d(&ned_p1, &ltp_def, &ecef_p1);
  printf("ecef to ned : (%f,%f,%f) -> (%f,%f,%f)\n",
         ecef_p1.x, ecef_p1.y, ecef_p1.z,
         ned_p1.x, ned_p1.y, ned_p1.z );

  struct EcefCoor_d ecef_p2;
  ecef_of_ned_point_d(&ecef_p2, &ltp_def, &ned_p1);
  printf("ned to ecef : (%f,%f,%f) -> (%f,%f,%f)\n",
         ned_p1.x, ned_p1.y, ned_p1.z,
         ecef_p2.x, ecef_p2.y, ecef_p2.z);

  printf("\n");

}






void test_enu_to_ecef_to_enu( void ) {

  printf("\n--- ENU -> ECEF -> ENU in float ---\n");

  struct EcefCoor_f ref_coor = { 4624497.0 , 116475.0, 4376563.0};
  printf("ecef0 : (%.02f,%.02f,%.02f)\n", ref_coor.x, ref_coor.y, ref_coor.z);

  struct LtpDef_f ltp_def;
  ltp_def_from_ecef_f(&ltp_def, &ref_coor);

  struct EcefCoor_f ecef_p1 = ref_coor;
  struct EnuCoor_f  enu_p1;
  enu_of_ecef_point_f(&enu_p1, &ltp_def, &ecef_p1);
  printf("ecef to enu : (%f,%f,%f) -> (%f,%f,%f)\n",
         ecef_p1.x, ecef_p1.y, ecef_p1.z,
         enu_p1.x, enu_p1.y, enu_p1.z );

  struct EcefCoor_f ecef_p2;
  ecef_of_enu_point_f(&ecef_p2, &ltp_def, &enu_p1);
  printf("enu to ecef : (%f,%f,%f) -> (%f,%f,%f)\n",
         enu_p1.x, enu_p1.y, enu_p1.z,
         ecef_p2.x, ecef_p2.y, ecef_p2.z);

  printf("\n");

}

