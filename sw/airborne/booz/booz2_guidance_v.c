#include "booz2_guidance_v.h"

#define B2_GUIDANCE_V_C
#define B2_GUIDANCE_V_USE_REF
#include "booz2_guidance_v_ref.h"

#include "radio_control.h"
#include "airframe.h"
#include "booz2_stabilization.h"
#include "booz2_fms.h"

#include "booz2_ins.h"
#include "booz_geometry_mixed.h"

uint8_t booz2_guidance_v_mode;
int32_t booz2_guidance_v_ff_cmd;
int32_t booz2_guidance_v_fb_cmd;
int32_t booz2_guidance_v_delta_t;
int32_t booz2_guidance_v_rc_delta_t;

/* altitude setpoint in meter (input)             */
/* Q23.8 : accuracy 0.0039, range 8388km          */
int32_t booz2_guidance_v_z_sp;
/* altitude reference in meter                    */
/* Q23.8 : accuracy 0.0039, range 8388km          */
int32_t booz2_guidance_v_z_ref;
/* vertical speed reference in meter/s            */
/* Q12.19 : accuracy 0.0000038, range 4096        */
int32_t booz2_guidance_v_zd_ref;
/* vertical acceleration reference in meter/s^2   */
/* Q21.10 : accuracy 0.0009766, range 2097152     */
int32_t booz2_guidance_v_zdd_ref;

int32_t booz2_guidance_v_kp;
int32_t booz2_guidance_v_kd;
int32_t booz2_guidance_v_ki;

int32_t booz2_guidance_v_z_sum_err;

static inline void run_hover_loop(bool_t in_flight);


void booz2_guidance_v_init(void) {

  booz2_guidance_v_mode = BOOZ2_GUIDANCE_V_MODE_KILL;

  booz2_guidance_v_kp = BOOZ2_GUIDANCE_V_HOVER_KP;
  booz2_guidance_v_kd = BOOZ2_GUIDANCE_V_HOVER_KD;
  booz2_guidance_v_ki = BOOZ2_GUIDANCE_V_HOVER_KI;

  booz2_guidance_v_z_sum_err = 0;

}


void booz2_guidance_v_read_rc(void) {

  booz2_guidance_v_rc_delta_t = (int32_t)rc_values[RADIO_THROTTLE] * 200 / MAX_PPRZ;

  switch (booz2_guidance_v_mode) {
    case BOOZ2_GUIDANCE_V_MODE_DIRECT:
      booz2_guidance_v_z_sp = booz_ins_position.z;
      break;
    case BOOZ2_GUIDANCE_V_MODE_HOVER:
      if (booz_fms_on && booz_fms_input.v_mode >= BOOZ2_GUIDANCE_V_MODE_HOVER)
        booz2_guidance_v_z_sp = booz_fms_input.v_sp.height;
      break;
  }

}

void booz2_guidance_v_mode_changed(uint8_t new_mode) {
  
  if (new_mode == booz2_guidance_v_mode)
    return;

  //  switch ( booz2_guidance_v_mode ) {
  //  
  //  }
   
  switch (new_mode) {
    
  case BOOZ2_GUIDANCE_V_MODE_HOVER:
    booz2_guidance_v_z_sum_err = 0;
    booz2_guidance_v_z_sp = booz_ins_position.z;
    break;
  }
  
  booz2_guidance_v_mode = new_mode;

}


void booz2_guidance_v_run(bool_t in_flight) {

  switch (booz2_guidance_v_mode) {
  case BOOZ2_GUIDANCE_V_MODE_DIRECT:
    booz2_stabilization_cmd[COMMAND_THRUST] = booz2_guidance_v_rc_delta_t;
    break;
  case BOOZ2_GUIDANCE_V_MODE_HOVER:
    run_hover_loop(in_flight);
    if (booz2_guidance_v_delta_t < booz2_guidance_v_rc_delta_t)
      booz2_stabilization_cmd[COMMAND_THRUST] = booz2_guidance_v_delta_t;
    else
      booz2_stabilization_cmd[COMMAND_THRUST] = booz2_guidance_v_rc_delta_t;
    break;
  }
}


#define MAX_Z_SUM_ERR (1<<23)

static inline void run_hover_loop(bool_t in_flight) {

#ifdef B2_GUIDANCE_V_USE_REF
  b2_gv_update_ref(booz2_guidance_v_z_sp);
#else
  b2_gv_set_ref(booz2_guidance_v_z_sp, 0, 0);
#endif
  int64_t tmp  = b2_gv_z_ref>>(B2_GV_Z_REF_FRAC - IPOS_FRAC);
  booz2_guidance_v_z_ref = (int32_t)tmp;
  booz2_guidance_v_zd_ref = b2_gv_zd_ref<<(ISPEED_RES - B2_GV_ZD_REF_FRAC);
  booz2_guidance_v_zdd_ref = b2_gv_zdd_ref<<(IACCEL_RES - B2_GV_ZDD_REF_FRAC);
  int32_t err_z  =  booz_ins_position.z - booz2_guidance_v_z_ref;
  int32_t err_zd =  booz_ins_speed_earth.z - booz2_guidance_v_zd_ref;

  if (in_flight) {
    booz2_guidance_v_z_sum_err += err_z;
    Bound(booz2_guidance_v_z_sum_err, -MAX_Z_SUM_ERR, MAX_Z_SUM_ERR);
  }
  else
    booz2_guidance_v_z_sum_err = 0;

  const int32_t inv_m = BOOZ_INT_OF_FLOAT(0.140, IACCEL_RES);
  booz2_guidance_v_ff_cmd = (BOOZ_INT_OF_FLOAT(9.81, IACCEL_RES) - booz2_guidance_v_zdd_ref) / inv_m;
  //  booz2_guidance_v_ff_cmd = BOOZ2_GUIDANCE_V_HOVER_POWER;

  booz2_guidance_v_fb_cmd = ((-booz2_guidance_v_kp * err_z) >> 12) + 
                            ((-booz2_guidance_v_kd * err_zd) >> 21) +
                            ((-booz2_guidance_v_ki * booz2_guidance_v_z_sum_err) >> 24);

  
  booz2_guidance_v_delta_t = booz2_guidance_v_ff_cmd + booz2_guidance_v_fb_cmd;


}

