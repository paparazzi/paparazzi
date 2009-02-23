#include "booz2_ins.h"

#include "booz2_imu.h"
#include "booz2_analog_baro.h"
#include "booz2_gps.h"

#include "airframe.h"
#include "booz_geometry_mixed.h"

#ifdef USE_VFF
#include "booz2_vf_float.h"
#endif
//#ifdef SITL
#include "booz2_filter_attitude.h"
#include "booz2_hf_float.h"
//#endif

#include "pprz_geodetic_int.h"


/* gps transformed to LTP-NED  */
struct LtpDef_i  booz_ins_ltp_def;
         bool_t  booz_ins_ltp_initialised;
struct NedCoor_i booz_ins_gps_pos_cm_ned;
struct NedCoor_i booz_ins_gps_speed_cm_s_ned;

/* barometer                   */
#ifdef USE_VFF
int32_t  booz_ins_qfe;
bool_t   booz_ins_baro_initialised;
int32_t  booz_ins_baro_alt;
#endif

/* output                      */
struct NedCoor_i booz_ins_ltp_pos;
struct NedCoor_i booz_ins_ltp_speed;
struct NedCoor_i booz_ins_ltp_accel;


struct Pprz_int32_lla booz_ins_position_init_lla;  // LLA

struct Pprz_int32_lla booz_ins_position_lla;  // LLA
struct Pprz_int32_vect3 booz_ins_position;    // NED
struct Pprz_int32_vect3 booz_ins_speed_earth; // NED
struct Pprz_int32_vect3 booz_ins_accel_earth; // NED


void booz_ins_init() {
#ifdef USE_VFF
  booz_ins_ltp_initialised  = FALSE;
  booz_ins_baro_initialised = FALSE;
  b2_vff_init(0., 0., 0.);
#endif
  //#ifdef SITL
  b2ins_init();
  //#endif
}

void booz_ins_propagate() {

#ifdef USE_VFF
  if (booz2_analog_baro_status == BOOZ2_ANALOG_BARO_RUNNING && booz_ins_baro_initialised) {
    FLOAT_T accel_float = BOOZ_ACCEL_F_OF_I(booz2_imu_accel.z);
    b2_vff_propagate(accel_float);
    booz_ins_accel_earth.z = BOOZ_ACCEL_I_OF_F(b2_vff_zdotdot);
    booz_ins_speed_earth.z = BOOZ_SPEED_I_OF_F(b2_vff_zdot);
    booz_ins_position.z = BOOZ_POS_I_OF_F(b2_vff_z);
  }
#endif
  //#ifdef SITL
  if (booz_ahrs.status == BOOZ2_AHRS_RUNNING &&
      booz_gps_state.fix == BOOZ2_GPS_FIX_3D && booz_ins_ltp_initialised )
    b2ins_propagate();
  //#endif
}

void booz_ins_update_baro() {

#ifdef USE_VFF
  if (booz2_analog_baro_status == BOOZ2_ANALOG_BARO_RUNNING) {
    if (!booz_ins_baro_initialised) {
      booz_ins_qfe = booz2_analog_baro_value;
      booz_ins_baro_initialised = TRUE;
    }
    booz_ins_baro_alt = (((int32_t)booz2_analog_baro_value - booz_ins_qfe) * BOOZ_INS_BARO_SENS_NUM)/BOOZ_INS_BARO_SENS_DEN;
    FLOAT_T alt_float = BOOZ_POS_F_OF_I(booz_ins_baro_alt);
    b2_vff_update(alt_float);
  }
#endif
}


void booz_ins_update_gps(void) {
  static bool_t first_pos_init_done = FALSE;
  if (booz_gps_state.fix == BOOZ2_GPS_FIX_3D) {
    if (!first_pos_init_done) {
      PPRZ_INT32_LLA_ASSIGN(booz_ins_position_init_lla, booz2_gps_lat, booz2_gps_lon, booz_ins_position.z);
      first_pos_init_done = TRUE;
    }
    PPRZ_INT32_LLA_ASSIGN(booz_ins_position_lla, booz2_gps_lat, booz2_gps_lon, booz_ins_position.z);
    BOOZ_IVECT2_ASSIGN(booz_ins_speed_earth, booz2_gps_vel_n, booz2_gps_vel_e);
    // copy to pos NED
    PPRZ_INT32_VECT2_OF_LL(booz_ins_position, booz_ins_position_lla);
  }
  

  if (booz_gps_state.fix == BOOZ2_GPS_FIX_3D) {
    if (!booz_ins_ltp_initialised) {
      ltp_def_from_ecef_i(&booz_ins_ltp_def, &booz_gps_state.ecef_pos);
      booz_ins_ltp_initialised = TRUE;
    }
    ned_of_ecef_point_i(&booz_ins_gps_pos_cm_ned, &booz_ins_ltp_def, &booz_gps_state.ecef_pos);
    ned_of_ecef_vect_i(&booz_ins_gps_speed_cm_s_ned, &booz_ins_ltp_def, &booz_gps_state.ecef_speed);

    //#ifdef SITL
    b2ins_update_gps();
#ifdef USE_H_FILTER
    VECT2_SDIV(booz_ins_ltp_pos, (1<<(B2INS_POS_LTP_FRAC-IPOS_FRAC)), b2ins_pos_ltp);
    VECT2_SDIV(booz_ins_ltp_speed, (1<<(B2INS_SPEED_LTP_FRAC-ISPEED_RES)), b2ins_speed_ltp);
#else
    VECT3_COPY(booz_ins_ltp_pos,   b2ins_meas_gps_pos_ned);
    VECT3_COPY(booz_ins_ltp_speed, b2ins_meas_gps_speed_ned);
#endif
  
  }

}


