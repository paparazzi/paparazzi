#include "pprz_geodetic_int.h"


#include "pprz_algebra_int.h"
#include "booz_geometry_int.h"
#include "booz_geometry_mixed.h"

#define CM_OF_M(_m)  ((_m)*1e2)
#define M_OF_CM(_cm) ((_cm)/1e2)
#define EM7RAD_OF_RAD(_r) (_r*1e7)
#define RAD_OF_EM7RAD(_r) (_r/1e7)
#define HIGH_RES_TRIG_FRAC  20

void ltp_def_from_ecef_i(struct LtpDef_i* def, struct EcefCoor_i* ecef) {

  /* store the origin of the tangeant plane */
  VECT3_COPY(def->ecef, *ecef);
  /* compute the lla representation of the origin */
  lla_of_ecef_i(&def->lla, &def->ecef);
  /* store the rotation matrix                    */

#if 1
  int32_t sin_lat = rint(BOOZ_INT_OF_FLOAT(sinf(RAD_OF_EM7RAD((float)def->lla.lat)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lat = rint(BOOZ_INT_OF_FLOAT(cosf(RAD_OF_EM7RAD((float)def->lla.lat)), HIGH_RES_TRIG_FRAC)); 
  int32_t sin_lon = rint(BOOZ_INT_OF_FLOAT(sinf(RAD_OF_EM7RAD((float)def->lla.lon)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lon = rint(BOOZ_INT_OF_FLOAT(cosf(RAD_OF_EM7RAD((float)def->lla.lon)), HIGH_RES_TRIG_FRAC)); 
#else
  int32_t sin_lat = rint(BOOZ_INT_OF_FLOAT(sin(RAD_OF_EM7RAD((double)def->lla.lat)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lat = rint(BOOZ_INT_OF_FLOAT(cos(RAD_OF_EM7RAD((double)def->lla.lat)), HIGH_RES_TRIG_FRAC)); 
  int32_t sin_lon = rint(BOOZ_INT_OF_FLOAT(sin(RAD_OF_EM7RAD((double)def->lla.lon)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lon = rint(BOOZ_INT_OF_FLOAT(cos(RAD_OF_EM7RAD((double)def->lla.lon)), HIGH_RES_TRIG_FRAC)); 
#endif


  def->ltp_of_ecef.m[0] = -sin_lon;
  def->ltp_of_ecef.m[1] =  cos_lon;
  def->ltp_of_ecef.m[2] =  0;
  def->ltp_of_ecef.m[3] = (int32_t)((-(int64_t)sin_lat*(int64_t)cos_lon)>>HIGH_RES_TRIG_FRAC);
  def->ltp_of_ecef.m[4] = (int32_t)((-(int64_t)sin_lat*(int64_t)sin_lon)>>HIGH_RES_TRIG_FRAC);
  def->ltp_of_ecef.m[5] =  cos_lat;
  def->ltp_of_ecef.m[6] = (int32_t)(( (int64_t)cos_lat*(int64_t)cos_lon)>>HIGH_RES_TRIG_FRAC);
  def->ltp_of_ecef.m[7] = (int32_t)(( (int64_t)cos_lat*(int64_t)sin_lon)>>HIGH_RES_TRIG_FRAC);
  def->ltp_of_ecef.m[8] =  sin_lat;

}


//void init_ltp_from_lla_i(struct LtpRef_i* ref_param, struct LlaCoor_i* ref_pos) {
//}

void enu_of_ecef_point_i(struct EnuCoor_i* enu, struct LtpDef_i* def, struct EcefCoor_i* ecef) {

  struct EcefCoor_i delta;
  VECT3_DIFF(delta, *ecef, def->ecef);
  const int64_t tmpx = (int64_t)def->ltp_of_ecef.m[0]*delta.x +
                       (int64_t)def->ltp_of_ecef.m[1]*delta.y +
                       0;                                     
  enu->x = (int32_t)(tmpx>>HIGH_RES_TRIG_FRAC);
  const int64_t tmpy = (int64_t)def->ltp_of_ecef.m[3]*delta.x +
                       (int64_t)def->ltp_of_ecef.m[4]*delta.y +
                       (int64_t)def->ltp_of_ecef.m[5]*delta.z;
  enu->y = (int32_t)(tmpy>>HIGH_RES_TRIG_FRAC);
  const int64_t tmpz = (int64_t)def->ltp_of_ecef.m[6]*delta.x +
		       (int64_t)def->ltp_of_ecef.m[7]*delta.y +
                       (int64_t)def->ltp_of_ecef.m[8]*delta.z;
  enu->z = (int32_t)(tmpz>>HIGH_RES_TRIG_FRAC);

}


void ned_of_ecef_point_i(struct NedCoor_i* ned, struct LtpDef_i* def, struct EcefCoor_i* ecef) {
  
  struct EnuCoor_i enu;
  enu_of_ecef_point_i(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);

}

void enu_of_ecef_vect_i(struct EnuCoor_i* enu, struct LtpDef_i* def, struct EcefCoor_i* ecef) {

  const int64_t tmpx = (int64_t)def->ltp_of_ecef.m[0]*ecef->x +
                       (int64_t)def->ltp_of_ecef.m[1]*ecef->y +
                       0;                                     
  enu->x = (int32_t)(tmpx>>HIGH_RES_TRIG_FRAC);
  const int64_t tmpy = (int64_t)def->ltp_of_ecef.m[3]*ecef->x +
                       (int64_t)def->ltp_of_ecef.m[4]*ecef->y +
                       (int64_t)def->ltp_of_ecef.m[5]*ecef->z;
  enu->y = (int32_t)(tmpy>>HIGH_RES_TRIG_FRAC);
  const int64_t tmpz = (int64_t)def->ltp_of_ecef.m[6]*ecef->x +
		       (int64_t)def->ltp_of_ecef.m[7]*ecef->y +
                       (int64_t)def->ltp_of_ecef.m[8]*ecef->z;
  enu->z = (int32_t)(tmpz>>HIGH_RES_TRIG_FRAC);

}


void ned_of_ecef_vect_i(struct NedCoor_i* ned, struct LtpDef_i* def, struct EcefCoor_i* ecef) {
  
  struct EnuCoor_i enu;
  enu_of_ecef_vect_i(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);

}




/* 
   For now we cheat and call the floating point version
   Anyone up for writing it in fixed point ? 
*/
#include "pprz_geodetic_float.h"

void lla_of_ecef_i(struct LlaCoor_i* out, struct EcefCoor_i* in) {

  /* convert our input to floating point */
  struct EcefCoor_f in_f;
  in_f.x = M_OF_CM((float)in->x);
  in_f.y = M_OF_CM((float)in->y);
  in_f.z = M_OF_CM((float)in->z);
  /* calls the floating point transformation */
  struct LlaCoor_f out_f;
  lla_of_ecef_f(&out_f, &in_f);
  /* convert the output to fixed point       */
  out->lon = (int32_t)rint(EM7RAD_OF_RAD(out_f.lon));
  out->lat = (int32_t)rint(EM7RAD_OF_RAD(out_f.lat));
  out->alt = (int32_t)CM_OF_M(out_f.alt);

}

void ecef_of_lla_i(struct EcefCoor_i* out, struct LlaCoor_i* in) {

  /* convert our input to floating point */
  struct LlaCoor_f in_f;
  in_f.lon = RAD_OF_EM7RAD((float)in->lon);
  in_f.lat = RAD_OF_EM7RAD((float)in->lat);
  in_f.alt = M_OF_CM((float)in->alt);
  /* calls the floating point transformation */
  struct EcefCoor_f out_f;
  ecef_of_lla_f(&out_f, &in_f);
  /* convert the output to fixed point       */
  out->x = (int32_t)CM_OF_M(out_f.x);
  out->y = (int32_t)CM_OF_M(out_f.y);
  out->z = (int32_t)CM_OF_M(out_f.z);

}

//#include "stdio.h"
void enu_of_lla_point_i(struct EnuCoor_i* enu, struct LtpDef_i* def, struct LlaCoor_i* lla) {
  struct EcefCoor_i ecef;
  ecef_of_lla_i(&ecef,lla);
  //printf("sim %d %d %d, def %d %d %d\n",ecef.x,ecef.y,ecef.z,def->ecef.x,def->ecef.y,def->ecef.z);
  //printf("sim lla def %d %d %d\n",def->lla.lat,def->lla.lon,def->lla.alt);
  enu_of_ecef_point_i(enu,def,&ecef);
}

void ned_of_lla_point_i(struct NedCoor_i* ned, struct LtpDef_i* def, struct LlaCoor_i* lla) {
  struct EcefCoor_i ecef;
  ecef_of_lla_i(&ecef,lla);
  ned_of_ecef_point_i(ned,def,&ecef);
}

