#include "subsystems/electrical.h"

#include "mcu_periph/adc.h"
#include "commands.h"

#include "generated/airframe.h"
#include BOARD_CONFIG

struct Electrical electrical;

static struct {
  struct adc_buf vsupply_adc_buf;
#ifdef ADC_CHANNEL_CURRENT
  struct adc_buf current_adc_buf;
#endif
#ifdef MILLIAMP_AT_FULL_THROTTLE
  float nonlin_factor;
#endif
} electrical_priv;

#ifndef VoltageOfAdc
#define VoltageOfAdc(adc) DefaultVoltageOfAdc(adc)
#endif
#ifndef MilliAmpereOfAdc
#define MilliAmpereOfAdc(adc) DefaultMilliAmpereOfAdc(adc)
#endif

#ifndef CURRENT_ESTIMATION_NONLINEARITY
#define CURRENT_ESTIMATION_NONLINEARITY 1.2
#endif

void electrical_init(void) {
  electrical.vsupply = 0;
  electrical.current = 0;

  adc_buf_channel(ADC_CHANNEL_VSUPPLY, &electrical_priv.vsupply_adc_buf, DEFAULT_AV_NB_SAMPLE);
#ifdef ADC_CHANNEL_CURRENT
  adc_buf_channel(ADC_CHANNEL_CURRENT, &electrical_priv.current_adc_buf, DEFAULT_AV_NB_SAMPLE);
#endif

#ifdef MILLIAMP_AT_FULL_THROTTLE
  electrical_priv.nonlin_factor = CURRENT_ESTIMATION_NONLINEARITY;
#endif
}

void electrical_periodic(void) {
#ifndef SITL
  electrical.vsupply = VoltageOfAdc((10*(electrical_priv.vsupply_adc_buf.sum/electrical_priv.vsupply_adc_buf.av_nb_sample)));
#endif

#ifdef ADC_CHANNEL_CURRENT
#ifndef SITL
  electrical.current = MilliAmpereOfAdc((electrical_priv.current_adc_buf.sum/electrical_priv.current_adc_buf.av_nb_sample));
#endif
#else
#if defined MILLIAMP_AT_FULL_THROTTLE && defined COMMAND_THROTTLE
      /*
       * Superellipse: abs(x/a)^n + abs(y/b)^n = 1
       * with a = 1
       * b = mA at full throttle
       * n = 1.2     This defines nonlinearity (1 = linear)
       * x = throttle
       * y = current
       *
       * define CURRENT_ESTIMATION_NONLINEARITY in your airframe file to change the default nonlinearity factor of 1.2
       */
      float b = (float)MILLIAMP_AT_FULL_THROTTLE;
      float x = ((float)commands[COMMAND_THROTTLE]) / ((float)MAX_PPRZ);
      /* electrical.current y = ( b^n - (b* x/a)^n )^1/n
       * a=1, n = electrical_priv.nonlin_factor
       */
      electrical.current = b - pow((pow(b,electrical_priv.nonlin_factor)-pow((b*x),electrical_priv.nonlin_factor)), (1./electrical_priv.nonlin_factor));
#endif
#endif /* ADC_CHANNEL_CURRENT */

}
