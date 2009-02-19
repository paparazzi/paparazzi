#include "booz2_ins.h"

#include "booz2_imu.h"
#include "booz2_analog_baro.h"
#include "booz2_gps.h"

#include "airframe.h"
#include "booz_geometry_mixed.h"

#ifdef USE_VFF
#include "booz2_vf_float.h"
#endif
#ifdef SITL
#include "booz2_filter_attitude.h"
#include "booz2_hf_float.h"
#endif

struct Pprz_int32_lla booz_ins_position_init_lla;  // LLA

struct Pprz_int32_lla booz_ins_position_lla;  // LLA
struct Pprz_int32_vect3 booz_ins_position;    // NED
struct Pprz_int32_vect3 booz_ins_speed_earth; // NED
struct Pprz_int32_vect3 booz_ins_accel_earth; // NED

#ifdef USE_VFD
int32_t booz_ins_g;
#define Z_EST_MAG 14
#define K_UPDATE_G 1
#endif
#ifdef USE_VFF
int32_t  booz_ins_baro_alt;
int32_t  booz_ins_qfe;
bool_t   booz_ins_baro_initialised;
#endif

void booz_ins_init() {
#ifdef USE_VFD
  const int32_t g_init = BOOZ_ACCEL_I_OF_F(9.81);
  booz_ins_g = g_init << Z_EST_MAG ;
#endif
#ifdef USE_VFF
  booz_ins_baro_initialised = FALSE;
  b2_vff_init(0., 0., 0.);
#endif
#ifdef SITL
  b2ins_init();
#endif
}

void booz_ins_propagate() {

#ifdef USE_VFD
  booz_ins_accel_earth.z = booz2_imu_accel.z + (booz_ins_g >> Z_EST_MAG);
  booz_ins_g -= booz_ins_accel_earth.z / K_UPDATE_G;
  booz_ins_speed_earth.z += booz_ins_accel_earth.z;
  booz_ins_speed_earth.z -= (booz_ins_speed_earth.z >> 10);
#endif
#ifdef USE_VFF
  if (booz2_analog_baro_status == BOOZ2_ANALOG_BARO_RUNNING && booz_ins_baro_initialised) {
    FLOAT_T accel_float = BOOZ_ACCEL_F_OF_I(booz2_imu_accel.z);
    b2_vff_propagate(accel_float);
    booz_ins_accel_earth.z = BOOZ_ACCEL_I_OF_F(b2_vff_zdotdot);
    booz_ins_speed_earth.z = BOOZ_SPEED_I_OF_F(b2_vff_zdot);
    booz_ins_position.z = BOOZ_POS_I_OF_F(b2_vff_z);
  }
#endif
#ifdef SITL
  if (booz2_filter_attitude_status == BOOZ2_FILTER_ATTITUDE_RUNNING)
    b2ins_propagate();
#endif
}

void booz_ins_update_baro() {

#ifdef USE_VFD
  booz_ins_position.z = booz2_analog_baro_value_filtered;
#endif
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
  


}


