#include "booz2_fms.h"

#include "booz2_ins.h"
#include "booz_geometry_mixed.h"

#define BOOZ2_FMS_TEST_SIGNAL_DEFAULT_PERIOD    300;
#define BOOZ2_FMS_TEST_SIGNAL_DEFAULT_AMPLITUDE 20;
#define BOOZ2_FMS_TEST_SIGNAL_DEFAULT_AXE  0

uint8_t  booz_fms_test_signal_mode;

uint32_t booz_fms_test_signal_period;
uint32_t booz_fms_test_signal_amplitude;
uint8_t  booz_fms_test_signal_axe;
uint32_t booz_fms_test_signal_counter;

uint32_t booz_fms_test_signal_start_z;

void booz_fms_impl_init(void) {
  booz_fms_test_signal_mode = BOOZ_FMS_TEST_SIGNAL_MODE_ATTITUDE;
  booz_fms_test_signal_period = BOOZ2_FMS_TEST_SIGNAL_DEFAULT_PERIOD;
  booz_fms_test_signal_amplitude = BOOZ2_FMS_TEST_SIGNAL_DEFAULT_AMPLITUDE;
  booz_fms_test_signal_axe = BOOZ2_FMS_TEST_SIGNAL_DEFAULT_AXE;
  booz_fms_test_signal_counter = 0;
  booz_fms_input.h_mode = BOOZ2_GUIDANCE_H_MODE_ATTITUDE;
  booz_fms_input.v_mode = BOOZ2_GUIDANCE_V_MODE_HOVER;
}

void booz_fms_impl_periodic(void) {

  switch (booz_fms_test_signal_mode) {
    
  case BOOZ_FMS_TEST_SIGNAL_MODE_ATTITUDE: {
    if (booz_fms_test_signal_counter < booz_fms_test_signal_period) {
      PPRZ_INT32_EULER_ASSIGN(booz_fms_input.h_sp.attitude, booz_fms_test_signal_amplitude, 0, 0);
    }
    else {
      PPRZ_INT32_EULER_ASSIGN(booz_fms_input.h_sp.attitude, -booz_fms_test_signal_amplitude, 0, 0);
    }
  }
    break;
    
  case BOOZ_FMS_TEST_SIGNAL_MODE_VERTICAL: {
    if (booz2_guidance_v_mode < BOOZ2_GUIDANCE_V_MODE_HOVER)
      booz_fms_test_signal_start_z = booz_ins_position.z;
    else {
      booz_fms_input.v_sp.height = (booz_fms_test_signal_counter < booz_fms_test_signal_period) ?
	booz_fms_test_signal_start_z : 
	booz_fms_test_signal_start_z - 256;
      //BOOZ_INT_OF_FLOAT(-0.5, IPOS_FRAC)
    }
  }
  break;

  }
  booz_fms_test_signal_counter++;
  if (booz_fms_test_signal_counter >= 2 * booz_fms_test_signal_period)
    booz_fms_test_signal_counter = 0;
}
