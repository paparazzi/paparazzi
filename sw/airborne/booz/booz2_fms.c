#include "booz2_fms.h"

#include "booz2_imu.h"
#include "booz2_gps.h"
#include "booz2_filter_attitude.h"

bool_t  booz_fms_on;
bool_t  booz_fms_timeout;
uint8_t booz_fms_last_msg;
struct Booz_fms_info    booz_fms_info;
struct Booz_fms_command booz_fms_input;

#define BOOZ_FMS_TIMEOUT 100

void booz_fms_init(void) {

  booz_fms_on = FALSE;
  booz_fms_timeout = TRUE;
  booz_fms_last_msg = BOOZ_FMS_TIMEOUT;

  booz_fms_input.h_mode = BOOZ2_GUIDANCE_H_MODE_ATTITUDE;
  BOOZ_IEULER_ZERO(booz_fms_input.h_sp.attitude);
  booz_fms_input.v_mode = BOOZ2_GUIDANCE_V_MODE_CLIMB;
  booz_fms_input.v_sp.climb = 0;
  booz_fms_impl_init();
}

void booz_fms_periodic(void) {
  if (booz_fms_last_msg < BOOZ_FMS_TIMEOUT) 
    booz_fms_last_msg++;
  else {
    booz_fms_timeout = TRUE;
    booz_fms_input.h_mode = BOOZ2_GUIDANCE_H_MODE_ATTITUDE;
    BOOZ_IEULER_ZERO(booz_fms_input.h_sp.attitude);
    booz_fms_input.v_mode = BOOZ2_GUIDANCE_V_MODE_CLIMB;
    booz_fms_input.v_sp.climb = 0;
  }
  booz_fms_impl_periodic();
}

void booz_fms_set_on_off(bool_t state) {


}

void booz_fms_update_info(void) {

  PPRZ_INT16_OF_INT32_VECT3(booz_fms_info.imu.gyro,  booz_imu_state.gyro);
  PPRZ_INT16_OF_INT32_VECT3(booz_fms_info.imu.accel, booz_imu_state.accel);
  PPRZ_INT16_OF_INT32_VECT3(booz_fms_info.imu.mag,   booz_imu_state.mag);

  PPRZ_INT32_VECT3_COPY(booz_fms_info.gps.pos, booz_gps_state.ecef_pos);
  PPRZ_INT16_OF_INT32_VECT3(booz_fms_info.gps.speed, booz_gps_state.ecef_speed);
  booz_fms_info.gps.pacc = booz_gps_state.pacc;
  booz_fms_info.gps.num_sv = booz_gps_state.num_sv;
  booz_fms_info.gps.fix = booz_gps_state.fix;

  PPRZ_INT16_OF_INT32_EULER(booz_fms_info.ahrs.euler, booz_ahrs_state.euler)
  PPRZ_INT16_OF_INT32_RATE (booz_fms_info.ahrs.rate,  booz_ahrs_state.rate)

}
