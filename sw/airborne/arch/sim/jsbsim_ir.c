/**
 *  \brief Regroup functions to simulate autopilot/infrared.c
 *
 * Infrared soft simulation.
 */


#include "jsbsim_hw.h"
#include <math.h>

#ifndef JSBSIM_IR_ROLL_NEUTRAL
#define JSBSIM_IR_ROLL_NEUTRAL 0.
#endif
#ifndef JSBSIM_IR_PITCH_NEUTRAL
#define JSBSIM_IR_PITCH_NEUTRAL 0.
#endif

void set_ir(double roll __attribute__((unused)), double pitch __attribute__((unused)))
{
  // INFRARED_TELEMETRY : Stupid hack to use with modules
#if USE_INFRARED || USE_INFRARED_TELEMETRY
  double ir_contrast = 150; //FIXME
  double roll_sensor = roll + JSBSIM_IR_ROLL_NEUTRAL; // ir_roll_neutral;
  double pitch_sensor = pitch + JSBSIM_IR_PITCH_NEUTRAL; // ir_pitch_neutral;
  infrared.roll = sin(roll_sensor) * ir_contrast;
  infrared.pitch = sin(pitch_sensor) * ir_contrast;
  infrared.top = cos(roll_sensor) * cos(pitch_sensor) * ir_contrast;
#endif
}


void ir_gain_calib(void) {}

/** Required by infrared.c:ir_init() */
void adc_buf_channel(uint8_t adc_channel __attribute__((unused)), struct adc_buf *s __attribute__((unused)),
                     uint8_t av_nb_sample __attribute__((unused))) {}
