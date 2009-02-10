#include "booz2_stabilization_rate.h"

#include "booz2_filter_aligner.h"

#include "booz2_stabilization.h"
#include "booz2_imu.h"
#include "radio_control.h"
#include "airframe.h"

struct booz_ivect booz2_stabilization_rate_measure;
struct booz_ivect booz2_stabilization_rate_sp;
struct booz_ivect booz2_stabilization_rate_gain;

void booz2_stabilization_rate_init(void) {

  BOOZ_IVECT_ZERO(booz2_stabilization_rate_sp);

  BOOZ_IVECT_ASSIGN(booz2_stabilization_rate_gain,
		    BOOZ_STABILIZATION_RATE_GAIN_P,
		    BOOZ_STABILIZATION_RATE_GAIN_Q,
		    BOOZ_STABILIZATION_RATE_GAIN_R);

}


void booz2_stabilization_rate_read_rc( void ) {

  BOOZ_IVECT_ASSIGN(booz2_stabilization_rate_sp,
		    (int32_t)-rc_values[RADIO_ROLL]  * BOOZ_STABILIZATION_RATE_SP_MAX_P / MAX_PPRZ,
		    (int32_t) rc_values[RADIO_PITCH] * BOOZ_STABILIZATION_RATE_SP_MAX_Q / MAX_PPRZ,
		    (int32_t)-rc_values[RADIO_YAW]   * BOOZ_STABILIZATION_RATE_SP_MAX_R / MAX_PPRZ);

}


void booz2_stabilization_rate_run(void) {

  /* low pass */
  BOOZ_IVECT_SUM(booz2_stabilization_rate_measure, booz2_stabilization_rate_measure, booz2_imu_gyro);
  if (booz2_filter_aligner_status == BOOZ2_FILTER_ALIGNER_LOCKED)
    BOOZ_IVECT_DIFF(booz2_stabilization_rate_measure, booz2_stabilization_rate_measure, 
		    booz2_filter_aligner_lp_gyro);
  BOOZ_IVECT_SDIV(booz2_stabilization_rate_measure, booz2_stabilization_rate_measure, 2);


  struct booz_ivect _error;
  BOOZ_IVECT_DIFF(_error, booz2_stabilization_rate_measure, booz2_stabilization_rate_sp);
  struct booz_ivect _cmd;
  BOOZ_IVECT_EWMULT(_cmd, _error, booz2_stabilization_rate_gain, 16);

  booz2_stabilization_cmd[COMMAND_ROLL]  = _cmd.x;
  booz2_stabilization_cmd[COMMAND_PITCH] = _cmd.y;
  booz2_stabilization_cmd[COMMAND_YAW]   = _cmd.z;
 
}

