/** 
 *  \brief Regroup functions to simulate autopilot/infrared.c
 *
 * Infrared soft simulation.
 */


#include "jsbsim_hw.h"
#include <math.h>

void set_ir(double roll, double pitch) {

  double ir_contrast = 150; //FIXME
  double roll_sensor = roll + ir_roll_neutral;
  double pitch_sensor = pitch + ir_pitch_neutral;
#ifdef INFRARED
  ir_roll = sin(roll_sensor) * ir_contrast;
  ir_pitch = sin(pitch_sensor) * ir_contrast;
  ir_top = cos(roll_sensor) * cos(pitch_sensor) * ir_contrast;
#endif

}


void ir_gain_calib(void) {}

/** Required by infrared.c:ir_init() */
void adc_buf_channel(uint8_t adc_channel __attribute__ ((unused)), struct adc_buf* s __attribute__ ((unused)), uint8_t av_nb_sample __attribute__ ((unused))) {}
