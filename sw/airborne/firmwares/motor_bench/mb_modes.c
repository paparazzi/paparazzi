#include "mb_modes.h"

//#include "mb_static.h"

#include "mcu_periph/adc.h"
#include "mcu_periph/sys_time.h"


uint8_t mb_modes_mode;
float mb_modes_throttle;

float mb_modes_last_change_time;

float mb_modes_ramp_duration;

float mb_modes_step_low_throttle;
float mb_modes_step_high_throttle;
float mb_modes_step_duration;

float mb_modes_sine_freq;
float mb_modes_sine_mean;
float mb_modes_sine_ampl;

static void mb_modes_manual(void);
static void mb_modes_ramp(void);
static void mb_modes_step(void);
static void mb_modes_prbs(void);
static void mb_modes_sine(void);

static struct adc_buf mb_modes_adc_buf; /* manual mode */

void mb_mode_init(void)
{
  adc_buf_channel(1, &mb_modes_adc_buf, 16);
  mb_modes_mode = MB_MODES_IDLE;
  mb_modes_throttle = 0.;

  mb_modes_ramp_duration = 200;

  mb_modes_step_low_throttle = 0.48;
  mb_modes_step_high_throttle = 0.65;
  mb_modes_step_duration = 0.5;

  mb_modes_sine_freq = 80.;
  mb_modes_sine_mean = 0.5;
  mb_modes_sine_ampl = 0.1;

  //  mb_static_init();

}

void mb_mode_event(void) {}

void mb_mode_periodic(float rpm, float thrust, float current)
{
  switch (mb_modes_mode) {
    case MB_MODES_IDLE :
      mb_modes_throttle = 0.;
      break;
    case MB_MODES_MANUAL :
      mb_modes_manual();
      break;
    case MB_MODES_RAMP :
      //    mb_static_periodic(rpm, thrust, current);
      //    mb_modes_throttle = (float)mb_static_throttle / (float)MB_STATIC_MAX_THROTTLE;
      mb_modes_ramp();
      break;
    case MB_MODES_STEP :
      mb_modes_step();
      break;
    case MB_MODES_SINE :
      mb_modes_sine();
      break;
      //  case MB_MODES_FIXED_RPM :
      //    mb_mode_fixed_rpm_periodic(rpm, thrust, current);
      //    mb_modes_throttle = mb_mode_fixed_rpm_throttle;
      //    break;
    default:
      mb_modes_throttle = 0.;
  }
}


static void mb_modes_manual(void)
{
  uint16_t poti =  mb_modes_adc_buf.sum;
  mb_modes_throttle = (float)poti / (16.*1024.);
}

static void mb_modes_ramp(void)
{
  float now = get_sys_time_float();
  float elapsed = now - mb_modes_last_change_time;
  if (elapsed < mb_modes_ramp_duration) {
    mb_modes_throttle = elapsed / mb_modes_ramp_duration;
  } else if (elapsed < 2 * mb_modes_ramp_duration) {
    mb_modes_throttle = 2 - elapsed / mb_modes_ramp_duration;
  } else {
    mb_modes_last_change_time = now;
    mb_modes_throttle = 0.;
  }
}


static void mb_modes_step(void)
{
  float now = get_sys_time_float();
  float elapsed = now - mb_modes_last_change_time;
  if (elapsed < mb_modes_step_duration) {
    mb_modes_throttle = mb_modes_step_low_throttle;
  } else if (elapsed < 2 * mb_modes_step_duration) {
    mb_modes_throttle = mb_modes_step_high_throttle;
  } else {
    mb_modes_last_change_time = now;
    mb_modes_throttle = mb_modes_step_low_throttle;
  }
}

static void mb_modes_sine(void)
{
  float now = get_sys_time_float();
  float alpha = 2. * M_PI * mb_modes_sine_freq * now;
  mb_modes_throttle = mb_modes_sine_mean + mb_modes_sine_ampl * sin(alpha);
}
