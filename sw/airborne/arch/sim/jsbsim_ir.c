/**
 *  \brief Regroup functions to simulate autopilot/infrared.c
 *
 * Infrared soft simulation.
 */


#include "jsbsim_hw.h"
#include <math.h>
#include "estimator.h"

#ifndef JSBSIM_IR_ROLL_NEUTRAL
#define JSBSIM_IR_ROLL_NEUTRAL 0.
#endif
#ifndef JSBSIM_IR_PITCH_NEUTRAL
#define JSBSIM_IR_PITCH_NEUTRAL 0.
#endif

void set_ir(double roll __attribute__ ((unused)), double pitch __attribute__ ((unused))) {

#ifdef USE_INFRARED
  double ir_contrast = 150; //FIXME
  double roll_sensor = roll + JSBSIM_IR_ROLL_NEUTRAL; // ir_roll_neutral;
  double pitch_sensor = pitch + JSBSIM_IR_PITCH_NEUTRAL; // ir_pitch_neutral;
  infrared.roll = sin(roll_sensor) * ir_contrast;
  infrared.pitch = sin(pitch_sensor) * ir_contrast;
  infrared.top = cos(roll_sensor) * cos(pitch_sensor) * ir_contrast;
#else
  estimator_phi = roll; // - ins_roll_neutral;
  estimator_theta = pitch; // - ins_pitch_neutral;
#endif

}


void ir_gain_calib(void) {}

/** Required by infrared.c:ir_init() */
void adc_buf_channel(uint8_t adc_channel __attribute__ ((unused)), struct adc_buf* s __attribute__ ((unused)), uint8_t av_nb_sample __attribute__ ((unused))) {}
